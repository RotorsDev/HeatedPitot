#include "heatedpitot.h"

mavlink_system_t mavlink_system = { 1, 111 }; // Ardupilot:7,1  Pixhawk:100,50
mavlink_message_t msg;
mavlink_status_t mv_status;

unsigned long mavlink_seen[256];   // Timestamp of the [id] message last seen.
unsigned long last_heartbeat_sent; // Timestamp of the last heartbeat sent out by the OSD
unsigned long last_telemetry_sent;

extern uint8_t system_id;

extern double target_temp;
extern double ambient_temp;
extern double probe_temp;
extern double output_duty;


void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    switch (chan)
    {
    case MAVLINK_COMM_0:
        Serial1.write(ch); //  mavlink_usart_send_byte(ch);
        break;
    default:
        break;
    }
}

const static uint8_t mavlink_stream_map[] = {
    MAV_DATA_STREAM_ALL,
    /* SCALED_IMU2, SCALED_PRESSURE, SENSOR_OFFSETS */
    MAV_DATA_STREAM_RAW_SENSORS,
    /* MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, LIMITS_STATUS */
    MAV_DATA_STREAM_EXTENDED_STATUS,
    /* SERVO_OUTPUT_RAW, RC_CHANNELS_RAW */
    MAV_DATA_STREAM_RC_CHANNELS,
    /* RC_CHANNELS_SCALED (HIL) */
    MAV_DATA_STREAM_RAW_CONTROLLER,
    /* GLOBAL_POSITION_INT */
    MAV_DATA_STREAM_POSITION,
    /* ATTITUDE, SIMSTATE (SITL) */
    MAV_DATA_STREAM_EXTRA1,
    /* VFR_HUD */
    MAV_DATA_STREAM_EXTRA2,
    /* AHRS, HWSTATUS, SYSTEM_TIME */
    MAV_DATA_STREAM_EXTRA3,
};

struct mavlink_config
{
    uint8_t streams[8];
    uint8_t uav_sysid, osd_sysid;
    uint8_t heartbeat;
    uint8_t shell_rate;
};

uint8_t mav_message[52]; // in MavLink max size is 50
uint8_t mav_msg_severity;

unsigned int mav_seq = 0;

mavlink_config config_mav;
uint8_t mavbeat = 0;
uint32_t last_mav_beat = 0;
uint32_t last_nobeat_message = 0;
uint32_t lastWritePanel = 0;

uint8_t waitingMAVBeats = 1;

uint8_t mav_type;

uint8_t enable_mav_request = 0;
uint32_t sys_start_time = 0;
uint32_t heatbeat_start_time = 0;
uint32_t armed_start_time = 0;
uint32_t total_armed_time = 0;

float osd_pitch = 0.0;
float osd_roll = 0.0;

float vibex, vibey, vibez;

uint8_t mavlink_requested = 0;
uint32_t osd_mode = 0;
bool motor_armed = false;
bool last_motor_armed = false;
uint8_t base_mode = 0;

// true when we have received at least 1 MAVLink packet
bool mavlink_active;

int packet_drops = 0;
int parse_error = 0;

#define MAX_STREAMS 7

bool getBit(uint8_t byte, int position) // position in range 0-7
{
    return (byte >> position) & 0x1;
}

void heartbeat_validation(void)
{
    unsigned long now;

    now = millis();
    // if no mavlink update for 3 secs, show warning and request mavlink rate again
    if (now > (last_mav_beat + 3000))
    {
        if (waitingMAVBeats && (now > (last_nobeat_message + 5000))) // Do not flood message queue with No heartbeat messages
        {
            last_nobeat_message = now;
        }
        heatbeat_start_time = 0;
        waitingMAVBeats = 1;
    }

    if (enable_mav_request == 1)
    {
        for (int n = 0; n < 3; n++)
        {
            request_mavlink_rates(); // Three times to make sure it will be readed
            delay(200);              // wait for 200ms
        }
        enable_mav_request = 0;
        waitingMAVBeats = 0;
    }
}

void request_mavlink_rates(void)
{
    //const uint8_t MAVStreams[MAX_STREAMS] = { MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_RC_CHANNELS, MAV_DATA_STREAM_POSITION,
    //                                          MAV_DATA_STREAM_EXTRA1,      MAV_DATA_STREAM_EXTRA2,          MAV_DATA_STREAM_EXTRA3 };
    //uint16_t MAVRates[MAX_STREAMS] = { 0x01, 0x05, 0x05, 0x05, 0x0a, 0x05, 0x02 };

    //for (uint32_t i = 0; i < MAX_STREAMS; i++)
    //{
    //    //        mavlink_msg_request_data_stream_send(MAVLINK_COMM_0, mavlink_system.sysid, mavlink_system.compid, MAVStreams[i], MAVRates[i], 1);
    //    mavlink_msg_request_data_stream_send(MAVLINK_COMM_0, 0, 0, MAVStreams[i], MAVRates[i], 1);
    //}
}

void request_mavlink_battery_capacity(void)
{
    ////    mavlink_msg_param_request_read_send(MAVLINK_COMM_0,mavlink_system.sysid, mavlink_system.compid, "BATT_CAPACITY",-1);
    //mavlink_msg_param_request_read_send(MAVLINK_COMM_0, 0, 0, "BATT_CAPACITY", -1);
}

void read_mavlink()
{
    mavlink_message_t msg;
    mavlink_status_t mv_status;
    // grabing data
    while (Serial1.available() > 0)
    {
        uint8_t c = Serial1.read();
        //debug("%c", c);
        // trying to grab msg
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &mv_status))
        {   
            // It returned non -1, so we have a valid message in the &msg buffer
            mavlink_seen[(uint8_t)msg.msgid] = millis(); // Update the last seen pointer for the message;
            mavlink_active = 1;                             // We are having active mavlink communication

            // handle msg

            switch (msg.msgid)
            {
            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                if (msg.compid != 1)
                {
                    // Message is not from ardupilot, discard
                    break;
                }
                mav_type = mavlink_msg_heartbeat_get_type(&msg);

                if (mav_type == MAV_TYPE_GCS)
                {
                    // Message is from a GCS, ignore it
                    break;
                }

                // Ok it is from an arupilot UAV, go ahead a process
                mavbeat = 1; // heartbeat is received
                last_mav_beat = millis();
                system_id = msg.sysid;  //TODO to check is it is not changind during flight

            }
            break;

            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            {

                debug("Mavlink_request_param\r\n");

                uint8_t sys, comp;

                sys = mavlink_msg_param_request_list_get_target_system(&msg);
                comp = mavlink_msg_param_request_list_get_target_component(&msg);

                //if ((sys != system_id) || (comp != MAV_COMP_ID_USER1)) break;

                param_send_index = 0; // init sending
            }
            break;
            case MAVLINK_MSG_ID_PARAM_SET:
            {
                uint8_t sys, comp;
                mavlink_message_t msg2;
                unsigned int len;
                char param_name[17];
                float param_value;
                int idx;

                sys = mavlink_msg_param_set_get_target_system(&msg);
                comp = mavlink_msg_param_set_get_target_component(&msg);

                //if ((sys != system_id) || (comp != MAV_COMP_ID_USER1)) break;

                len = mavlink_msg_param_set_get_param_id(&msg, param_name);
                if (len == 16) param_name[16] = '\0';

                param_value = mavlink_msg_param_set_get_param_value(&msg);

                debug("set_param: %s - %f\n", param_name, param_value);

                idx = params_set_value(param_name, param_value, 0);

                /* broadcast new parameter value */
                mavlink_msg_param_value_pack(system_id, MAV_COMP_ID_USER1, &msg2, param_name, param_value, MAVLINK_TYPE_FLOAT, total_params, idx);
                mavlink_send_msg(&msg2);
            }
            break;

            default:
                // Do nothing
                break;
            }
            // Check for timeslot expiry
            // Abandon MavLink reading if we used up our timeslot.
        }
    }
    // Update global packet drops counter
    // packet_drops += mv_status.packet_rx_drop_count;
    // parse_error += mv_status.parse_error;

    // debug("Attitude messages processed:%u\n", mm);
}

void send_param_list()
{

    if (param_send_index == total_params) return;

    mavlink_message_t msg;
    float param_value;
    char param_name[17];
    debug("Param %i sent\r\n", param_send_index);
    param_value = get_parameter_value(param_send_index, param_name);
    mavlink_msg_param_value_pack(system_id, MAV_COMP_ID_USER1, &msg, param_name, param_value, MAVLINK_TYPE_FLOAT, total_params, param_send_index++);
    mavlink_send_msg(&msg);

   
}

void mavlink_send_msg(mavlink_message_t *msg)
{

    unsigned int len;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    len = mavlink_msg_to_send_buffer(buf, msg);

    Serial1.write(buf, len);
}

unsigned long mavdata_age(unsigned int id)
{
    if (mavlink_seen[id] != 0)
        return millis() - mavlink_seen[id];
    else
        return 99999999;
}

void heartbeat_out(void)
{
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(system_id, MAV_COMP_ID_USER1, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // base_mode
                               0,                                 // custom_mode
                               MAV_STATE_ACTIVE);

    mavlink_send_msg(&msg);
}



void send_telemetry()
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_generator_status_pack(system_id, MAV_COMP_ID_USER1, &msg,0,0,probe_temp,ambient_temp,target_temp,output_duty,0,0,0,0,0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
}

