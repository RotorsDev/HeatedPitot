#ifndef HEATEDPITOT_H
#define	HEATEDPITOT_H


#define VERSION "1.0b"
#define BUILD __DATE__
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include <Arduino.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <EEPROM.h>
#include <OneWire.h>


#include "mavlink_local.h"
#include "params.h"
#include "settings.h"
#include "debug.h"
#include "MAX6675.h"
#include "DallasTemperature.h"
#include "PIDController.h"

#endif
