

#include "heatedpitot.h"


struct settings g;               //This will contain all osd setting


void save_settings()
{
	unsigned int address = 0;
	char* ptr;


	ptr = (char*)&g;

	for (address = 0; address < sizeof(g); address++)
	{
		EEPROM.write(address, *ptr++);
	}

}

uint8_t load_settings()
{
	unsigned int address = 0;
	char* ptr;


	ptr = (char*)&g;

	for (address = 0; address < sizeof(g); address++)
	{
		*ptr++ = EEPROM.read(address);
	}

	return g.eeprom_version;

}



void default_settings()
{

	debug("Default Settings invoked\n");


	g.eeprom_version = EEPROM_VERSION;
	g.settings_size = sizeof(g);

	g.Kp = 30;
	g.Ki = 0.7;
	g.Kd = 200;
	g.target_temp = 80;
	g.ambient_offset = 0;

}
