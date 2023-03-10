#ifndef _MAVLINK_H_
#define _MAVLINK_H_

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "./mavlink/mavlink_types.h"

extern mavlink_system_t mavlink_system; //Ardupilot:7,1  Pixhawk:100,50
extern mavlink_message_t msg; 
extern mavlink_status_t mv_status;
extern unsigned long last_heartbeat_sent;		//Timestamp of the last heartbeat sent out by the OSD

extern unsigned long mavlink_seen[256];          //Timestamp of the [id] message last seen.

void comm_send_ch(mavlink_channel_t chan, uint8_t ch);

#include "./mavlink/ardupilotmega/mavlink.h"
#include "./mavlink/ardupilotmega/version.h"


#define OSD_SYS_ID 200              //OSD System id for sending messages


extern unsigned int mav_seq;

void read_mavlink();
void request_mavlink_rates(void);
void request_mavlink_battery_capacity(void);
void heartbeat_validation(void);
unsigned long mavdata_age(unsigned int id);
void mavlink_send_msg(mavlink_message_t *msg);
void send_telemetry(void);

void heartbeat_out(void);

#endif