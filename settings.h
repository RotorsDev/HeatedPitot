#ifndef _SETTINGS_H_
#define _SETTINGS_H_


#define EEPROM_VERSION 0x01


struct settings {


	uint8_t   eeprom_version;								// version for eeporm content version.
	uint16_t  settings_size;

	float Kp;
	float Ki;
	float Kd;

	float	  target_temp;
	float	  ambient_offset;								//If this is not null, then the target temp is always ambient plus offset;

};

void default_settings();
uint8_t   load_settings();
void save_settings();



#endif
