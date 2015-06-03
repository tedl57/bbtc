/*
initialize display
boot
...
count attached 1-wire sensors
display that count: 1Fnd (found)
	if none found, display 0Fnd

delay 3s

loop
	check for programming
		if so, do programming
	check if next sense is due
		if so, sense
	check if time for to display next reading
		if so, display next reading
	check if temperatures changed
		if so, control relays as necessary

settings:
remove hyst, num boxes
add control (1=control relays), 0=read/display temps only) (default 1)
	output is 1-44 if off, 1 44 if on
don't allow max <= min, or min >= max
add interval: number of seconds between temp reads (default 60) (max 0-240)

h/w design change: use display decimal points to indicate on instead of separate LEDs

allow for plug-in logger (to capture output of readings/events)
	ground/power/serial out (9600)
	Serial is not just debug, but also loggable, output something parsable for the readings and events (data output lines start with :)
	:read:ch:temp:millis
	:on:ch:temp:millis
	:off:ch:temp:millis

parts:
	blue fourdigit led sfe serial sparkfun
		datasheet http://localhost/apps/pdfadd/get.php?2488
		elpart http://localhost/forms/el_parts.php?action=search&search_button=ID&qs=527
*/

//// defines
#define EE_ADR_NUM_SETTINGS 0
#define EE_ADR_F_OR_C 1
#define EE_ADR_CTRL 2
#define EE_ADR_TEMP 3
#define EE_ADR_SENSE_INTERVAL 4
#define EE_ADR_DISPLAY_INTERVAL 5
#define EE_ADR_RESET 6

#define EE_ADR_NUM_DEVICES 31
#define EE_ADR_DEVICES 32

#define DEF_NUM_SETTINGS 6
#define MAX_CHANNELS 4
#define DISPLAY_INTERVAL 3000L	// display next reading every 3 seconds

//// includes
#include <EEPROM.h>
#include <Bounce2.h>
#include <SoftwareSerial.h>

// allow reset from software
#include <avr/wdt.h>
#include <SoftReset.h>

//// globals
SoftwareSerial fourdigitDisplay(6,5);
byte fourdigit_special_state = 0;

unsigned long time_sensed;
unsigned long time_displayed;
int last_displayed = -1;

// i/o pins

// programming buttons
int pin_button_program = 2;
int pin_button_value = 7;

// controllable output pins
int pins_ctrl[MAX_CHANNELS] = {A0,A1,A2,A3};
boolean ctrl_states[MAX_CHANNELS];

// objects to debounce programming buttons
Bounce button_program = Bounce();
Bounce button_value = Bounce();

//// settings
#define SETTING_TYP_NUMBER 0
#define SETTING_TYP_BOOLEAN 1
#define SETTING_TYP_BOOLEAN_SPECIAL 2
#define SETTING_TYP_SENSOR_COUNT 3

typedef struct asetting {
	char *name;
	byte typ;		// type: number, bool (on/off), special bool (c/f)
	byte val;		// current value
	byte valmin;	// number min or special off value
	byte valmax;	// number max or special on value
} SETTING;

#define F_OR_C_F 'F'	// above all other setting values so 'F' or 'C' can be displayed instead 0 or 1
#define F_OR_C_C 'C'

SETTING _settings[DEF_NUM_SETTINGS+1] = {
	{ "num_settings", SETTING_TYP_NUMBER, DEF_NUM_SETTINGS, 0, 0 },
	{ "f_or_c", SETTING_TYP_BOOLEAN_SPECIAL, F_OR_C_F, F_OR_C_F, F_OR_C_C },
	{ "ctrl", SETTING_TYP_BOOLEAN, 1, 0, 1 },
	{ "temp", SETTING_TYP_NUMBER, 65, 0, 99 },
	{ "sense_interval", SETTING_TYP_NUMBER, 3, 0, 255 },
	{ "display_interval", SETTING_TYP_NUMBER, 3, 1, 60 },
	{ "reset_devices", SETTING_TYP_BOOLEAN, 0, 0, 1 }
};

//// temperature sensing and display
#define NO_TEMP_SENSED -111
int readings[MAX_CHANNELS];
int _num_sensors = 0;

//// logging functions
void log_state()
{
	Serial.print(":state:");
	for(int i=0;i<MAX_CHANNELS;i++)
	{
		Serial.print(ctrl_states[i]);
	}
	Serial.println();
}
void log_readings()
{
	// output per sensor:
	// :read:ch:temp:millis
	for(int i = 0 ; i < MAX_CHANNELS ; i++ )
	{
		if ( ! sensoradr_inuse( i ) )
			continue;
		Serial.print(":read:");
		Serial.print(i+1);
		Serial.print(":");
		Serial.print(readings[i]);
		Serial.print(":");
		Serial.println(millis());
	}
}
//// settings functions
void settings_read_saved()
{
	// initialize memory copies of settings from EEPROM
	int num_settings = EEPROM.read(EE_ADR_NUM_SETTINGS);
	for ( int i = 1 ; i <= num_settings ; i++ )
		_settings[i].val = EEPROM.read(i);
	Serial.println("#Read saved settings");
}
void settings_save_setting( int nsetting, byte val)
{
	EEPROM.write(nsetting,val);
}
void settings_save_defaults()
{
	// save default settings into eeprom
	int num_settings = _settings[0].val;
	for ( int i = 0 ; i <= num_settings; i++ )
		EEPROM.write(i,_settings[i].val);

	// reset number of stored devices (temperature sensors)
	EEPROM.write(EE_ADR_NUM_DEVICES,0);
	Serial.println("#Saved default settings");
}
void settings_dump()
{
	Serial.print("#Settings: ");
	int nsettings = _settings[0].val;
 
	for ( int i = 1 ; i <= nsettings ; i++ )
	{
		Serial.print(i);
		Serial.print(")");
		Serial.print(_settings[i].name);
		Serial.print("=");
		if ( _settings[i].typ == SETTING_TYP_BOOLEAN_SPECIAL )
			Serial.write(_settings[i].val);
		else
			Serial.print(_settings[i].val);
		if ( i != nsettings )
			Serial.print(",");
	}
	Serial.println();
}
//// fourdigit functions
#define FOURDIGIT_BRIGHTNESS_NORMAL 10
#define FOURDIGIT_BRIGHTNESS_MID_BRIGHT 2
#define FOURDIGIT_BRIGHTNESS_BRIGHTEST 0
#define FOURDIGIT_SPECIAL_COLON 4

	void fourdigit_error_freeze(int err) { // does not return

		// display error
		fourdigitDisplay.write('E');
		fourdigitDisplay.write('r');
		fourdigitDisplay.write('r');
		fourdigitDisplay.write(err);

		// freeze processor (do nothing forever)
		while(true)
			;
	}

void fourdigit_sensors_found(int nsensors)	// 0fnd 1fnd 2fnd
{
	fourdigitDisplay.write(nsensors);
	fourdigitDisplay.write('F'); // 'F'
	fourdigitDisplay.write('n'); // 'n'
	fourdigitDisplay.write('D'); // 'D'
}
void fourdigit_sensor_using(int nsensor,boolean bNew)	// 0 n (new) 1 e (existing)
{
	fourdigitDisplay.write(nsensor);
	fourdigitDisplay.write(0x78); // blank
	byte b = bNew ? 'n' : 'e';
	fourdigitDisplay.write(b);
	fourdigitDisplay.write(0x78); // blank
}
void fourdigit_setting(byte settingn, int val, byte typ)
{
	// left to right: digit1 digit2 digit3 digit4

	// display a setting or channel number followed by a blank and a number

	// handle negative numbers by displaying a - on digit 2 as necessary

	fourdigitDisplay.write(settingn);

	if ( typ == SETTING_TYP_BOOLEAN_SPECIAL )
	{
		fourdigitDisplay.write(0x78); // blank
		fourdigitDisplay.write(0x78); // blank
		fourdigitDisplay.write((byte)val);
		return;
	}
	boolean bNeg = false;
	if ( val < 0 )
	{
		val *= -1;
		bNeg = true;
	}
	if ( val < 10 )
	{
		fourdigitDisplay.write(0x78); // blank
		if ( bNeg )
			fourdigitDisplay.write('-');	// negative sign
		else
			fourdigitDisplay.write(0x78);	// blank
		fourdigitDisplay.write(val);
	}
	else
	{
		if ( val < 100 )
		{
			if ( bNeg )
				fourdigitDisplay.write('-');	// negative sign
			else
				fourdigitDisplay.write(0x78);	// blank
			fourdigitDisplay.write(val/10);
			fourdigitDisplay.write(val%10);
		}
		else
		{
			fourdigitDisplay.write(val/100);
			fourdigitDisplay.write((val/10)%10);
			fourdigitDisplay.write(val%10);
		}
	}
}
void fourdigit_brightness(byte n)
{
	fourdigitDisplay.write(0x7A);
	fourdigitDisplay.write(n);	// 0 is brightest, 254 is dimmest - doesn't seem linear
}
void fourdigit_alldigits(byte n)
{
	for(int i = 0 ; i < 4 ; i++)
		fourdigitDisplay.write(n);
}
void fourdigit_clear()
{
	fourdigitDisplay.write(0x76);	// reset display - set cursor to left digit
	fourdigitDisplay.write(0x77);	// clear all special chars decimal points, colon, degrees
	fourdigitDisplay.write((byte)0);
	fourdigit_special_state = 0;
}
void fourdigit_colon_fast_blink()
{
	int numblinks = 1;
	for(int i = 0 ; i< numblinks ; i++)
	{
		fourdigit_special(FOURDIGIT_SPECIAL_COLON,true);
		delay(50);
		fourdigit_special(FOURDIGIT_SPECIAL_COLON,false);
	}
}
//// temperature sensors
/*
output of DallaTemperature/examples/Muliple/Multiple.ino

Dallas Temperature IC Control Library Demo
Locating devices...Found 2 devices.
5arasite power is: ON
Device 0 Address: 1043AAB801080009
Device 1 Address: 108789B8010800FC
Device 0 Resolution: 9
Device 1 Resolution: 9
Requesting temperatures...DONE
Device Address: 1043AAB801080009 Temp C: 21.56 Temp F: 70.81
Device Address: 108789B8010800FC Temp C: 21.00 Temp F: 69.80
Requesting temperatures...DONE
Device Address: 1043AAB801080009 Temp C: 21.56 Temp F: 70.81
Device Address: 108789B8010800FC Temp C: 21.06 Temp F: 69.91

*/
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 8	// arduino pin with 1-wire temperature sensors connected (with a 1k pullup)

OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// memory copy of device addresses - index is FIXED LOGICAL channel, not the physical index from the dallas temperature library
DeviceAddress sensoraddrs[MAX_CHANNELS];

// function to print a device address
void printAddress(DeviceAddress deviceAddress);
void printAddress(DeviceAddress deviceAddress)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		// zero pad the address if necessary
		if (deviceAddress[i] < 16)
			Serial.print("0");
		Serial.print(deviceAddress[i], HEX);
	}
}
boolean sensoradr_inuse(int ndx) {	// returns true if sensor address has been set (not all initialized to 0xFF )
	for(int j = 0 ; j < 8 ; j++ )
		if ( sensoraddrs[ndx][j] != 0xFF )
			return true;
	return false;
}
void sensoradr_dump() {

	Serial.println("#sensoradr_dump:");
	boolean b;
	for(int i = 0 ; i < MAX_CHANNELS ; i++ )
	{
		Serial.print("#");
		Serial.print(i+1);	// human-friendly "box number"
		Serial.print(")[");
		b = sensoradr_inuse(i);
		Serial.print(b);
		Serial.print("] ");
		printAddress(sensoraddrs[i]);
		if ( b )
		{
			Serial.print(",resolution=");
			Serial.print(sensors.getResolution(sensoraddrs[i]), DEC); 
		}
		Serial.println();
	}
}
int sensoradr_ee_find(DeviceAddress devaddr);
int sensoradr_ee_find(DeviceAddress devaddr)
{
	int naddrs = EEPROM.read(EE_ADR_NUM_DEVICES);
	int b, j;
	// return -1 if not found, else index of slot found (0 - MAX_CHANNELS-1)
	for ( int i = 0 ; i < naddrs ; i++ )
	{
		int eeaddr = EE_ADR_DEVICES + i * 8;
		for(j=0;j<8;j++)
		{
			if ( EEPROM.read(eeaddr++) != devaddr[j] )
				break;
		}
		if ( j >= 8 )
			return i;
	}
	return -1;
}
int sensoradr_ee_save(DeviceAddress devaddr);
int sensoradr_ee_save(DeviceAddress devaddr) {
	int naddrs = EEPROM.read(EE_ADR_NUM_DEVICES);
	if ( naddrs >= MAX_CHANNELS )
	{
		// log and display error and freeze
		Serial.println("!ERROR: MAX sensors stored already");
		fourdigit_error_freeze(1);	// does not return
		return -1;
	}
	int eeaddr = EE_ADR_DEVICES + naddrs * 8;
	for(int j=0;j<8;j++)
		EEPROM.write(eeaddr++,devaddr[j]);
	EEPROM.write( EE_ADR_NUM_DEVICES, naddrs+1);
	Serial.print("#Saved sensor ");
	printAddress(devaddr);
	Serial.print(" @ ");
	Serial.println(naddrs+1);	// human-friendly
	return naddrs;
}
void sensoradr_ee_dump() {
	int naddrs = EEPROM.read(EE_ADR_NUM_DEVICES);
	Serial.print("#sensoradr_ee_dump: device(s) stored: ");
	Serial.println(naddrs);
	int eeaddr = EE_ADR_DEVICES;
	DeviceAddress devaddr;
	for(int i=0;i<naddrs;i++)
	{
		for(int j=0;j<8;j++)
			devaddr[j]= EEPROM.read(eeaddr++);
		Serial.print("#");
		Serial.print(i+1);	// human-friendly "box number"
		Serial.print(") ");
		printAddress(devaddr);
		Serial.println();
	}
}
void sensoradr_init() {
	// really not needed, sensoraddrs is global and is initialized to zeroes already,
	// but just to be complete, using 0xFF as "not set" instead of 0x00
	for(int i = 0 ; i < MAX_CHANNELS ; i++ )
		for(int j = 0 ; j < 8 ; j++ )
			sensoraddrs[i][j] = 0xFF;
}
void setup_sensors()
{
	// clear previous memory copy of sensor readings
	clear_readings();

	int i;

	sensoradr_init();
	sensoradr_ee_dump();

	// Start up the library
	sensors.begin();
	// locate devices on the bus
	Serial.print("#Locating devices...");
	_num_sensors = sensors.getDeviceCount();
	Serial.print("found ");
	Serial.print(_num_sensors, DEC);
	Serial.print(" device");
	if ( _num_sensors != 1 )
		Serial.print("s");
	Serial.println(".");

	fourdigit_sensors_found(_num_sensors);
	delay(2000);

	// don't continue setup if no sensors are connected
	if ( _num_sensors == 0 )
		return;

	// check if device has already been stored in eeprom, if so, use it's index
	// else, store it and use it's new index

	int ndx;
	int j;
	boolean bNew;
	DeviceAddress devaddr;
	for ( i = 0 ; i < _num_sensors ; i++ )
	{
		if (! sensors.getAddress(devaddr, i))
		{
			// log and display error and freeze
			Serial.print("!ERROR: unable to find address for device "); 
			Serial.println(i); 
			fourdigit_error_freeze(2);	 // does not return
		}
		ndx = sensoradr_ee_find(devaddr);
		bNew = (ndx < 0);

		if ( bNew )
			ndx = sensoradr_ee_save(devaddr);

		// copy device address into memory copy of sensor addresses
		for(j=0;j<8;j++)
			sensoraddrs[ndx][j] = devaddr[j];

		// display if sensor is 'e'xisting or 'n'ew
		fourdigit_sensor_using(ndx+1,bNew);	// human-friendly
		delay(2000);

		// set sensor resolution to 9 bits
		sensors.setResolution(devaddr, 9);
	}

	// show the logical addresses of the devices found on bus
	sensoradr_dump();

	// report parasitic power status
	Serial.print("#Sensor parasitic power is: "); 
	if (sensors.isParasitePowerMode())
		Serial.println("ON");
	else
		Serial.println("OFF");
}
unsigned long do_display()
{
	if ( _num_sensors == 0 )
		return millis();

	// find index of "present" sensor to display next (not always sequential)
	
	int nlast = last_displayed;

	do
	{
		if ( ++nlast >= MAX_CHANNELS )
			nlast = 0;

		if ( sensoradr_inuse( nlast ) )
			break;
	}
	while ( nlast != last_displayed );
	last_displayed = nlast;


	if ( readings[last_displayed] == NO_TEMP_SENSED )
		return millis();

	fourdigit_setting(last_displayed+1,readings[last_displayed],SETTING_TYP_NUMBER);
	return millis();
}
void onChangeState(int ch, boolean state)
{
	// output state change event to log
	Serial.print(":ch_state:");
	Serial.print(ch+1);
	Serial.print(":");
	Serial.print(state);
	Serial.print(":");
	Serial.println(millis());

	// update control state
	ctrl_states[ch] = state;
	//log_state();
	
	// control relay (or other device) via i/o pin change
	digitalWrite(pins_ctrl[ch],state?HIGH:LOW);
	

	// update decimal point to reflect control state change
	fourdigit_special(ch,state);
}
void onChangeTemperature( int ch, int otemp, int ntemp )
{
	if ( _settings[EE_ADR_CTRL].val == 0 )
		return;

	// output temperature change event to log
	Serial.print(":ch_temp:");
	Serial.print(ch+1);
	Serial.print(":");
	Serial.print(otemp);
	Serial.print(":");
	Serial.print(ntemp);
	Serial.print(":");
	Serial.println(millis());

	int threshold_temp = _settings[EE_ADR_TEMP].val;
	// if on and needs to be off
	if ( ctrl_states[ch] && ntemp >= threshold_temp )
		onChangeState(ch,false);
	else
	// if off and needs to be on
		if ( ctrl_states[ch] == false && ntemp < threshold_temp )
			onChangeState(ch,true);
}
unsigned long do_sense()
{
	if ( _num_sensors == 0 )
	{
		fourdigit_sensors_found(0);
		return millis();
	}
	// call sensors.requestTemperatures() to issue a 
	// synchronous temperature request from all devices on the bus
//Serial.print("Requesting temperatures...");
	sensors.requestTemperatures();
//Serial.println("done.");

	// copy sensed value into local cache for display
	float temp;
	for ( int i = 0 ; i < MAX_CHANNELS ; i++ )
	{
		if ( ! sensoradr_inuse(i) )
			continue;
		// get reading from device in appropriate units
		if ( _settings[ EE_ADR_F_OR_C ].val == F_OR_C_C )
			temp = sensors.getTempC(sensoraddrs[i]);
		else
			temp = sensors.getTempF(sensoraddrs[i]);

		// handle temperature changes
		int ntemp = (int) temp;
		if ( readings[i] != ntemp )
			onChangeTemperature( i, readings[i], ntemp );

		// save for display and later "change" processing
		readings[i] = ntemp;
	}
	// output new readings to log
	log_readings();

	// give user feedback about reading event
	fourdigit_colon_fast_blink();

	return millis();
}
void fourdigit_special(byte special,boolean b)
{
	if ( b )
		fourdigit_special_state |= (0x01 << special);
	else
		fourdigit_special_state &= ~(0x01 << special);
	fourdigitDisplay.write(0x77);
	fourdigitDisplay.write(fourdigit_special_state);
}
//// programming functions
void do_programming()
{
	// begin programming

	// force all controlled devices off while programming
	clear_ctrl_states();

	// reset display (all special chars off)
	fourdigit_clear();

	Serial.println("#Programming started...");
	int val;
	int typ;
	fourdigit_brightness(FOURDIGIT_BRIGHTNESS_MID_BRIGHT);
	fourdigit_special(FOURDIGIT_SPECIAL_COLON,true);
 
	for ( int nsettings = 1; nsettings <= _settings[0].val ; nsettings++ )
	{
		Serial.print("#Setting ");
		Serial.print(nsettings);
		Serial.print(": ");
		Serial.print(_settings[nsettings].name);
		Serial.print("=");
		val = _settings[nsettings].val;
		typ = _settings[nsettings].typ;
		if ( typ == SETTING_TYP_BOOLEAN_SPECIAL )
			Serial.write(val);
		else
			Serial.print(val);
		Serial.println();

		fourdigit_setting(nsettings,(int)val,typ);
		for ( ; ; )
		{
			if ( button_program.update() && button_program.read() == 0 )
				break;

			if ( button_value.update() && button_value.read() == 0 )
			{
				if ( typ == SETTING_TYP_BOOLEAN_SPECIAL )
				{
					val = ( val == _settings[nsettings].valmax ) ?
								_settings[nsettings].valmin :
								_settings[nsettings].valmax;
				}
				else
				{
					if ( ++val > _settings[nsettings].valmax )
						val = _settings[nsettings].valmin;
				}
				fourdigit_setting(nsettings,(int)val,typ);

				Serial.print("#New value: " );
				if ( typ == SETTING_TYP_BOOLEAN_SPECIAL )
					Serial.write(val);
				else
					Serial.print(val);
				Serial.println();
			}
		}

		if ( val != _settings[nsettings].val )
		{
			if ( nsettings == EE_ADR_RESET )
			{
				// reset number of stored devices (temperature sensors)
				EEPROM.write(EE_ADR_NUM_DEVICES,0);
				Serial.println("#Reset device count" );
				soft_restart();
			}
			Serial.println("#Saved new value" );
			settings_save_setting( nsettings, _settings[nsettings].val = val );
		}
	}
 
	fourdigit_brightness(FOURDIGIT_BRIGHTNESS_NORMAL);
	fourdigit_clear();
	Serial.println("#Programming completed.");	// end programming ended

	// clear cached readings so new settings will restablish state
	clear_readings();

	// force sensing just after programming to restablish state
	time_sensed = 0L;
	time_displayed = 0L;
}
//// convenience functions
void clear_readings()
{
	// force all readings to the unread state to (re)initialize the system
	for(int i = 0 ; i < MAX_CHANNELS ; i++ )
		readings[i] = NO_TEMP_SENSED;
}
void clear_ctrl_states()
{
	// force all control states to off
	for(int i = 0 ; i < MAX_CHANNELS ; i++ )
	{
		digitalWrite(pins_ctrl[i],LOW);
		ctrl_states[i] = false;
	}
}
#define APP_VERSION "0.9.0"
void boot_start() // show boot sequence start
{
	Serial.print("#Booting version ");
	Serial.print(APP_VERSION);
	Serial.print(": ");
	int b = 6;
	do
	{
		fourdigit_alldigits(--b);
		Serial.print(" ");
		Serial.print(b);
		delay(500);
	} while(b>0);
	Serial.println(" done.");
}
//// setup //////////////////////////////////////////////////////
void setup()
{
	//// setup debugging/logging console

	Serial.begin(9600);

	//// setup 4 digit LED display for user feedback

	fourdigitDisplay.begin(9600);	// 4 digit LED display
	fourdigit_clear();
	fourdigit_brightness(FOURDIGIT_BRIGHTNESS_NORMAL);

	// give user boot sequence feedback

	boot_start();

	//// setup programming buttons 

	// programming buttons are inputs
	pinMode(pin_button_program,INPUT);
	pinMode(pin_button_value,INPUT);

	// programming buttons use internal pull-up resistors
	pinMode(pin_button_program, INPUT_PULLUP);
	pinMode(pin_button_value, INPUT_PULLUP);

	// initialize button debouncer instances
	button_program.attach(pin_button_program);
	button_program.interval(5); // interval in ms
	button_value.attach(pin_button_value);
	button_value.interval(5); // interval in ms
 
	// setup controllable outputs
	for(int i = 0 ; i < MAX_CHANNELS ; i++ )
		pinMode(pins_ctrl[i],OUTPUT);
	clear_ctrl_states();

	//// setup persistent settings

	// uncomment to force reprogramming of default settings
	//EEPROM.write(EE_ADR_NUM_SETTINGS,0);

	// read number of settings, if not set, initialize all settings (new chip or forced reprogramming)
	int num_settings = EEPROM.read(EE_ADR_NUM_SETTINGS);
	if (num_settings != DEF_NUM_SETTINGS)
		settings_save_defaults();
	settings_read_saved();
	settings_dump(); 

	//// setup temperature sensors

	setup_sensors();
	delay(1000);
	fourdigit_clear();
	time_sensed = 0L;	// do a sense right away
}
//// loop //////////////////////////////////////////////////////
void loop()
{
	// enter programming state when programming button pressed
	// function does not return until programming is completed (after programming all the settings)
	if ( button_program.update() && button_program.read() == 0 )
		do_programming();

	// todo: optimization, calculate this only once (or when setting changed)
	long sense_interval = 1000L * (long) _settings[EE_ADR_SENSE_INTERVAL].val;
	if ( millis() - time_sensed > sense_interval )
		time_sensed = do_sense();

	long display_interval = 1000L * (long) _settings[EE_ADR_DISPLAY_INTERVAL].val;
	if ( millis() - time_displayed > display_interval )
		time_displayed = do_display();
}
