/*
    Name:       HeatedPitot.ino
    Created:	1/18/2023 2:20:21 PM
    Author:     ROTORS-005\bandi
*/

//TODO : Move it to hwdef

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 14

//Thermocouple pins
#define MAX6675_DO  12
#define MAX6675_CS  8
#define MAX6675_CLK 11

//Heater switch FET on pin9, lo -- off ??
#define HEATER_PIN 9


//TODO: make it parameters
#define __Kp 30 // Proportional constant
#define __Ki 0.7 // Integral Constant
#define __Kd 200 // Derivative Constant


#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "heatedpitot.h"

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

//Thermocouple library init
MAX6675 thermocouple(MAX6675_CLK, MAX6675_CS, MAX6675_DO);

//Init pid controller library
PIDController pid;


extern settings g;
unsigned long now;
uint8_t system_id = 1;          //Mavlink system id, get it from the heartbeat


double ambient_temp;
double probe_temp;
double target_temp;
double output_duty;

extern unsigned long last_telemetry_sent;

void setup()
{
    // Open USB serial port for debug and UI
    Serial.begin(115200);
    delay(500);

    debug("System start\r\n");


    //Settings from EEPROM
    load_settings();
    if ((g.eeprom_version != EEPROM_VERSION) || (g.settings_size != sizeof(g)))
    {
        default_settings();
        save_settings();
    }

    //INIT Paremeter Mavlink microservice
    get_parameter_count();
    param_send_index = total_params;

// Open Serial1 port for MavLink communication
    // TODO: make it configurable
    Serial1.begin(115200);

    // Start up the OneWire Sensor library
    sensors.begin();

    //Init PID controller
    pid.begin();          // initialize the PID instance
    pid.setpoint(50);    // The "goal" the PID controller tries to "reach"
    pid.tune(__Kp, __Ki, __Kd);    // Tune the PID, arguments: kP, kI, kD
    pid.limit(0, 255);    // Limit the PID output between 0 and 255, this is important to get rid of integral windup!

   


}

void loop()
{
    now = millis();
    sensors.requestTemperatures();
    ambient_temp = sensors.getTempCByIndex(0);
    debug("Ambient : %i \r\n", (int)ambient_temp);
    read_mavlink();


    //debug("Total Params:%i\r\n", total_params);
    //debug("send_index:%i\r\n", param_send_index);

    //heartbeat_validation();





    // Send eight params at once, to speed up parameter download time
    if (param_send_index != total_params)
    {
        for (int i = 0; i < 5; i++) send_param_list();
    }

    // Send out mavlink heartbeat
    if ((millis() - last_heartbeat_sent) >= 5000)
    {
        heartbeat_out();
        last_heartbeat_sent = millis();
    }




    

    if (ambient_temp == DEVICE_DISCONNECTED_C) ambient_temp = 10;

    if (g.ambient_offset == 0) target_temp = g.target_temp;
    else target_temp = ambient_temp + g.ambient_offset;


    probe_temp = thermocouple.readCelsius(); // Read the Temperature using the readCelsius methode from MAX6675 Library.


    debug("Probe:%i\r\n", (long)probe_temp);
    

    int output = pid.compute(probe_temp);    // Let the PID compute the value, returns the optimal output
    analogWrite(HEATER_PIN, output);           // Write the output to the output pin
    output_duty = output;


    pid.setpoint(target_temp); // Use the setpoint methode of the PID library to

    if ((millis() - last_telemetry_sent) >= 1000)
    {
        send_telemetry();
        last_telemetry_sent = millis();
    }

}
