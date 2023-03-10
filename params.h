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

#ifndef _PARAMS_H_
#define _PARAMS_H_

extern int			total_params;
extern int			param_send_index;

struct param_value {
    union {
            float param_float;
            long param_int32;
            unsigned long param_uint32;
            int param_int16;
            unsigned int param_uint16;
            char param_int8;
            uint8_t param_uint8;
    };
};


struct param_def {
    char name[17];
    uint8_t type;
    void *value;
	void(*cbk)(void);
};

extern struct param_def parameters[];

void get_parameter_count();
unsigned int get_parameter_index(char *name);
float cast2float(void *value, uint8_t type);
void cast2param(struct param_def *p, float v);
float get_parameter_value(int idx, char *name);

void send_param_list();


int params_set_value(char *name, float value, uint8_t trigger_cbk);
void do_settings_save();


#endif