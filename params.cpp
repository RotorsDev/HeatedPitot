/*
	OSDeluxe - Color PIP Mavlink OSD
	Copyright (C) 2018  Andras Schaffer - Dronedoktor.eu

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

	With Grateful Acknowledgements to the projects:
		AlceOSD by Luis Alves
		PlayUAV OSD
		Brain FPV Flight Controller(https://github.com/BrainFPV/TauLabs) by Tau Labs

 */

#include "heatedpitot.h"

#define PARAM(n, t, p, c) {  n, t, (void *) p,  c }
#define PARAM_END { "" }

int total_params;						// Number of all params (int, to eliminate warning about unsigned, signed comparisions
int param_send_index;

extern settings g;

struct param_def parameters[] = {
	//Generic parameters

	//PARAM("Name",type, &valtozo, &update fuggveny)

	PARAM("PID_P", MAV_PARAM_TYPE_REAL32, &g.Kp, NULL),
	PARAM("PID_I", MAV_PARAM_TYPE_REAL32, &g.Ki, NULL),
	PARAM("PID_D", MAV_PARAM_TYPE_REAL32, &g.Kd, NULL),

	PARAM("TARGET_TMP", MAV_PARAM_TYPE_REAL32, &g.target_temp, NULL),
	PARAM("AMB_OFFSET", MAV_PARAM_TYPE_REAL32, &g.ambient_offset, NULL),
	
	PARAM_END
};

void do_settings_save()
{
	save_settings();

}

const char *mavdata_type_name[] = {
	"CHAR", "UINT8_T", "INT8_T",
	"UINT16_T", "INT16_T",
	"UINT32_T", "INT32_T",
	"UINT64_T", "INT64_T",
	"FLOAT", "DOUBLE"
};


void get_parameter_count()
{
	total_params = 0;

	while (parameters[total_params].name[0] != '\0')
	{
		total_params++;

	}
}

unsigned int get_parameter_index(char *name)
{
	int idx;

	if (total_params == 0) get_parameter_count();

	for (idx = 0; idx < total_params; idx++) {
		if (strcmp(name, parameters[idx].name) == 0)
			break;
	}
	return idx;
}

float get_parameter_value(int idx, char *name)
{
//	struct param_def p;
//	struct param_value pv;

//	p.value = &pv;

	if (total_params == 0) get_parameter_count();

	if (idx == -1)
		idx = get_parameter_index(name);

	if (idx < total_params) {
		strcpy(name, parameters[idx].name);
		return cast2float(parameters[idx].value, parameters[idx].type);
	}
	else
	{
		return 0;
	}
}

int params_set_value(char *name, float value, uint8_t trigger_cbk)
{
	struct param_def *p;
	int idx;

	idx = get_parameter_index(name);
	//debug("name='%s' idx=%d\n", name, idx);
	if (idx < total_params) {
		p = &parameters[idx];
		cast2param(p, value);
		if ((p->cbk != NULL) && trigger_cbk)
			p->cbk();
	}
	do_settings_save();
	return idx;
}




float cast2float(void *value, uint8_t type)
{
	switch (type) {
	case MAV_PARAM_TYPE_UINT8:
		return (float) *((uint8_t*)(value));
	case MAV_PARAM_TYPE_INT8:
		return (float) *((char*)(value));
	case MAV_PARAM_TYPE_UINT16:
		return (float) *((uint16_t*)(value));
	case MAV_PARAM_TYPE_INT16:
		return (float) *((short*)(value));
	case MAV_PARAM_TYPE_REAL32:
		return (float) *((float*)(value));
	default:
		return 0;
	}
}

void cast2param(struct param_def *p, float v)
{
	switch (p->type) {
	case MAV_PARAM_TYPE_UINT8:
		*((uint8_t*)(p->value)) = (uint8_t)v;
		break;
	case MAV_PARAM_TYPE_INT8:
		*((char*)(p->value)) = (char)v;
		break;
	case MAV_PARAM_TYPE_UINT16:
		*((uint16_t*)(p->value)) = (uint16_t)v;
		break;
	case MAV_PARAM_TYPE_INT16:
		*((short*)(p->value)) = (short)v;
		break;
	case MAV_PARAM_TYPE_REAL32:
		*((float*)(p->value)) = (float)v;
		break;
	default:
		break;
	}
}
