#include "libraries\mavlink\common\mavlink.h"
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <VL53L0X.h>

#define PIN1 2
#define PIN2 3
#define PIN3 4
#define PIN4 5
#define PIN5 6
#define PIN6 7

int sens_orie[] = {MAV_SENSOR_ROTATION_NONE, MAV_SENSOR_ROTATION_YAW_90, MAV_SENSOR_ROTATION_YAW_180, MAV_SENSOR_ROTATION_YAW_270, MAV_SENSOR_ROTATION_PITCH_90, MAV_SENSOR_ROTATION_PITCH_270};

VL53L0X v1, v2, v3, v4, v5, v6;

int dist[6];

struct Coor {
  float x = 0;
  float y = 0;
  float z = 0;
};

long boot_time = 0;
long targ_time = 0;
long sens_last = 0;
long iter1000ms = 0;
long iter50ms = 0;
int TARG_SYS_ID = -1, TARG_COMP_ID = -1;
bool firstMessage = true;
Coor vel;
Coor vel1;
Coor vel2;

#define h   30
#define hh  (h / 2)

#define trm 5
#define tlm 5
#define brm 5
#define blm 5
#define um  5
#define dm  15
#define M   1200
#define a   0.6181
#define s   20

#define GSC_SYS_ID 255

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(500000);
  while (! Serial)
    delay(1);
  Serial.println(F("Hello"));

  //  init_vl();

  Serial3.begin(57600);

  iter1000ms = millis() / 1000;
  iter50ms = millis() / 50;

  boot_time = millis();
}


void loop() {
  receive_msg();

  if (millis() / 1000 > iter1000ms) {
    send_hb();
    iter1000ms = millis() / 1000;
  }

  //  if (millis() / 50 > iter50ms) {
  //    get_vl();
  //    send_dsts();
  //    iter50ms = millis() / 50;
  //  }
}

void send_dsts() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  for (int i = 0; i < 6; i++) {
    //    mavlink_msg_distance_sensor_pack(TARG_SYS_ID, MAV_COMP_ID_PERIPHERAL, &msg, millis() - boot_time, 1, 120, dist[i] / 10, MAV_DISTANCE_SENSOR_INFRARED, i, sens_orie[i], i);
    mavlink_msg_distance_sensor_pack(TARG_SYS_ID, MAV_COMP_ID_ALL, &msg, millis() - boot_time, 1, 1200, dist[i], MAV_DISTANCE_SENSOR_INFRARED, i, sens_orie[i], i);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial3.write(buf, len);
  }

  //  Serial.print(dist[0]);
  //  Serial.print(" ");
  //  Serial.print(millis() - sens_last);
  //  Serial.print(" ");
  //  Serial.println(millis() - boot_time);
  sens_last = millis();
}

void send_hb() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_STANDBY);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial3.write(buf, len);
  //  Serial.println(F(">> hb"));
}


void first_msg() {

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;

  mavlink_msg_set_mode_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, 221, 9);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial3.write(buf, len);

  mavlink_msg_request_data_stream_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, MAV_DATA_STREAM_ALL, 1000, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial3.write(buf, len);

  //  mavlink_msg_command_long_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, MAV_CMD_NAV_PATHPLANNING, 1, 2, 0, 0, 0, 0, 0, 0);
  mavlink_msg_param_set_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, "RNGFND_LANDING\0", 1, 6);
//  mavlink_msg_param_request_read_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, "RNGFND_LANDING\0", -1);
//  mavlink_msg_param_request_list_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial3.write(buf, len);


  //  mavlink_msg_message_interval_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, 33, 500);
  //  mavlink_msg_command_long_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, MAV_CMD_SET_MESSAGE_INTERVAL, 0, 30, 1000, 0, 0, 0, 0, 0);
  //  mavlink_msg_mission_item_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, 1, MAV_FRAME_LOCAL_NED, MAV_CMD_DO_SET_MODE, 1, 1, MAV_MODE_GUIDED_ARMED, 0, 0, 0, 0, 0, 0, MAV_MISSION_TYPE_MISSION);
  //  mavlink_msg_mission_request_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, 0);
  //  mavlink_msg_set_local_position_setpoint_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, 0, 2.5, -2.5, 2.39, 2.28);
  //  mavlink_msg_mission_request_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, 1);         ///// last param 1 (not 0)
  //&&  mavlink_msg_mission_clear_all_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID);
  //&&  len = mavlink_msg_to_send_buffer(buf, &msg);
  //&&  Serial3.write(buf, len);
  //&&  mavlink_msg_mission_count_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, 2);
  //&&  len = mavlink_msg_to_send_buffer(buf, &msg);
  //&&  Serial3.write(buf, len);
}

void receive_msg() {

  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial3.available() > 0 ) {

    uint8_t c = Serial3.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {         //**MAVLINK_COMM_[x] -- uart[x]

      if (firstMessage) {

        TARG_SYS_ID = msg.sysid;
        TARG_COMP_ID = msg.compid;

        //@@Serial.print(F("targ_sys : "));//@@Serial.println(TARG_SYS_ID);
        //@@Serial.print(F("targ_comp : "));//@@Serial.println(TARG_COMP_ID);
        //@@Serial.print(F("gsc_sys : "));//@@Serial.println(GSC_SYS_ID);
        //@@Serial.print(F("gsc_comp : "));//@@Serial.println(MAV_COMP_ID_ALL);

        first_msg();

        firstMessage = false;
      }

      switch (msg.msgid)
      {
        case MAVLINK_MSG_ID_PARAM_VALUE:
          {
            mavlink_param_value_t pv;
            mavlink_msg_param_value_decode(&msg, &pv);

            Serial.print(F("pv  :  "));
            Serial.print(pv.param_value); Serial.print(F("  "));
            Serial.print(pv.param_count); Serial.print(F("  "));
            Serial.print(pv.param_index); Serial.print(F("  "));
            Serial.print(pv.param_id); Serial.print(F("  "));
            Serial.println(pv.param_type); 
          }
          break;
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
          {
            //            mavlink_nav_controller_output_t nco;
            //            mavlink_msg_nav_controller_output_decode(&msg, &nco);
            //
            //            Serial.print(F("nco :  "));
            //            Serial.print(nco.nav_roll); Serial.print(F("  "));
            //            Serial.print(nco.nav_pitch); Serial.print(F("  "));
            //            Serial.print(nco.alt_error); Serial.print(F("  "));
            //            Serial.print(nco.aspd_error); Serial.print(F("  "));
            //            Serial.print(nco.xtrack_error); Serial.print(F("  "));
            //            Serial.print(nco.nav_bearing); Serial.print(F("  "));
            //            Serial.print(nco.target_bearing); Serial.print(F("  "));
            //            Serial.println(nco.wp_dist);
          }
          break;
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
          {
            //            mavlink_distance_sensor_t ds;
            //            mavlink_msg_distance_sensor_decode(&msg, &ds);
            //
            //            Serial.print(F("ds  :  "));
            //            Serial.print(ds.time_boot_ms); Serial.print(F("  "));
            //            Serial.print(ds.min_distance); Serial.print(F("  "));
            //            Serial.print(ds.max_distance); Serial.print(F("  "));
            //            Serial.print(ds.current_distance); Serial.print(F("  "));
            //            Serial.print(ds.type); Serial.print(F("  "));
            //            Serial.print(ds.id); Serial.print(F("  "));
            //            Serial.print(ds.orientation); Serial.print(F("  "));
            //            Serial.println(ds.covariance);
          }
          break;
        case MAVLINK_MSG_ID_SYSTEM_TIME:
          {
            mavlink_system_time_t st;
            mavlink_msg_system_time_decode(&msg, &st);

            targ_time = st.time_boot_ms;
          }
          break;
        case MAVLINK_MSG_ID_MISSION_ITEM:
          {
            mavlink_mission_item_t mi;
            mavlink_msg_mission_item_decode(&msg, &mi);

            //@@Serial.print(F("mi  :  "));
            //@@Serial.print(mi.target_system);//@@Serial.print(F("  "));
            //@@Serial.print(mi.target_component);//@@Serial.print(F("  "));
            //@@Serial.print(mi.frame);//@@Serial.print(F("  "));
            //@@Serial.print(mi.current);//@@Serial.print(F("  "));
            //@@Serial.print(mi.seq);//@@Serial.print(F("  "));
            //@@Serial.print(mi.autocontinue);//@@Serial.print(F("  "));
            //@@Serial.print(mi.command);//@@Serial.print(F("  "));
            //@@Serial.print(mi.param1);//@@Serial.print(F("  "));
            //@@Serial.print(mi.param2);//@@Serial.print(F("  "));
            //@@Serial.print(mi.param3);//@@Serial.print(F("  "));
            //@@Serial.print(mi.param4);//@@Serial.print(F("  "));
            //@@Serial.print(mi.x);//@@Serial.print(F("  "));
            //@@Serial.print(mi.y);//@@Serial.print(F("  "));
            //@@Serial.println(mi.z);

            switch (mi.seq) {
              case 0:
                {
                  //&&                  mavlink_message_t msg;
                  //&&                  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                  //&&                  uint16_t len;

                  //&&                  mavlink_msg_mission_request_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, 1);

                  //&&                  len = mavlink_msg_to_send_buffer(buf, &msg);
                  //&&                  Serial3.write(buf, len);
                }
                break;
              case 1:
                {
                  //&&                  mavlink_message_t msg;
                  //&&                  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                  //&&                  uint16_t len;

                  //&&                  mavlink_msg_mission_ack_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, 0);

                  //&&                  len = mavlink_msg_to_send_buffer(buf, &msg);
                  //&&                  Serial3.write(buf, len);
                }
                break;
            }
          }
          break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:           //**or MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED
          {
            mavlink_local_position_ned_t lpn;
            mavlink_msg_local_position_ned_decode(&msg, &lpn);

            //@@Serial.println(F("lpnlpnlpnlpnlpnlpn  :  "));
            //@@Serial.print(lpn.time_boot_ms);//@@Serial.print(F("  "));
            //@@Serial.print(lpn.x);//@@Serial.print(F("  "));
            //@@Serial.print(lpn.y);//@@Serial.print(F("  "));
            //@@Serial.print(lpn.z);//@@Serial.print(F("  "));
            //@@Serial.print(lpn.vx);//@@Serial.print(F("  "));
            //@@Serial.print(lpn.vy);//@@Serial.print(F("  "));
            //@@Serial.println(lpn.vz);
          }
          break;
        case MAVLINK_MSG_ID_STATUSTEXT:
          {
            mavlink_statustext_t st;
            mavlink_msg_statustext_decode(&msg, &st);

            //@@Serial.print(F("st  :  "));
            //@@Serial.print(st.severity);//@@Serial.print(F("  "));
            //@@Serial.println(st.text);
          }
          break;
        case MAVLINK_MSG_ID_MISSION_REQUEST:
          {
            mavlink_mission_request_t mr;
            mavlink_msg_mission_request_decode(&msg, &mr);

            //@@Serial.print(F("mr  :  "));
            //@@Serial.print(mr.target_system);//@@Serial.print(F("  "));
            //@@Serial.print(mr.target_component);//@@Serial.print(F("  "));
            //@@Serial.println(mr.seq);

            // why gsc_id == 0?

            switch (mr.seq) {
              case 0:
                {
                  //&&                  mavlink_message_t msg;
                  //&&                  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                  //&&                  uint16_t len;

                  //&&                  mavlink_msg_mission_item_pack(mr.target_system, mr.target_component, &msg, TARG_SYS_ID, TARG_COMP_ID,
                  //&&                                                0, MAV_FRAME_GLOBAL, MAV_CMD_NAV_TAKEOFF, 1, 1,                                  ////////must GLOBAL???
                  //&&                                                0, 0, 0, 0, 2.28, 2.39, 2.56);

                  //&&                  len = mavlink_msg_to_send_buffer(buf, &msg);
                  //&&                  Serial3.write(buf, len);
                }
                break;
              case 1:
                {
                  //&&                  mavlink_message_t msg;
                  //&&                  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                  //&&                  uint16_t len;

                  //&&                  mavlink_msg_mission_item_pack(mr.target_system, mr.target_component, &msg, TARG_SYS_ID, TARG_COMP_ID,
                  //&&                                                1, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 0, 1,
                  //&&                                                1, 1, 0, 0, 3.28, 3.39, 3.56);

                  //&&                  len = mavlink_msg_to_send_buffer(buf, &msg);
                  //&&                  Serial3.write(buf, len);


                  //&&                  mavlink_msg_mission_request_list_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID);

                  //&&                  len = mavlink_msg_to_send_buffer(buf, &msg);
                  //&&                  Serial3.write(buf, len);
                }
                break;
            }
          }
          break;
        case MAVLINK_MSG_ID_MISSION_ACK:
          {
            mavlink_mission_ack_t ma;
            mavlink_msg_mission_ack_decode(&msg, &ma);

            //@@Serial.print(F("ma  :  "));
            //@@Serial.print(ma.target_system);//@@Serial.print(F("  "));
            //@@Serial.print(ma.target_component);//@@Serial.print(F("  "));
            //@@Serial.println(ma.type);
          }
          break;
        case MAVLINK_MSG_ID_MISSION_COUNT:
          {
            mavlink_mission_count_t mc;
            mavlink_msg_mission_count_decode(&msg, &mc);

            //@@Serial.print(F("mc  :  "));
            //@@Serial.print(mc.target_system);//@@Serial.print(F("  "));
            //@@Serial.print(mc.target_component);//@@Serial.print(F("  "));
            //@@Serial.println(mc.count);

            //&&            mavlink_message_t msg;
            //&&            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            //&&            uint16_t len;

            //&&            mavlink_msg_mission_request_pack(GSC_SYS_ID, MAV_COMP_ID_ALL, &msg, TARG_SYS_ID, TARG_COMP_ID, 0);

            //&&            len = mavlink_msg_to_send_buffer(buf, &msg);
            //&&            Serial3.write(buf, len);
          }
          break;
        case MAVLINK_MSG_ID_COMMAND_ACK:
          {
            mavlink_command_ack_t ca;
            mavlink_msg_command_ack_decode(&msg, &ca);

            Serial.print(F("ca  :  "));
            Serial.print(ca.command); Serial.print(F("  "));
            Serial.println(ca.result);
          }
          break;
        case MAVLINK_MSG_ID_HEARTBEAT:
          {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);

            //            Serial.print(F("hb  :  "));
            //            Serial.print(hb.custom_mode); Serial.print(F("  "));
            //            Serial.print(hb.type); Serial.print(F("  "));
            //            Serial.print(hb.autopilot); Serial.print(F("  "));
            //            Serial.print(hb.base_mode); Serial.print(F("  "));
            //            Serial.print(hb.system_status); Serial.print(F("  "));
            //            Serial.println(hb.mavlink_version);
          }
          break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          {
            //            mavlink_global_position_int_t gpi;
            //            mavlink_msg_global_position_int_decode(&msg, &gpi);
            //
            //@@Serial.println(F("gpi :  "));
            //           //@@Serial.print(gpi.time_boot_ms);//@@Serial.print(F("  "));
            //           //@@Serial.print(gpi.lat);//@@Serial.print(F("  "));
            //           //@@Serial.print(gpi.lon);//@@Serial.print(F("  "));
            //           //@@Serial.print(gpi.alt);//@@Serial.print(F("  "));
            //           //@@Serial.print(gpi.relative_alt);//@@Serial.print(F("  "));
            //           //@@Serial.print(gpi.vx);//@@Serial.print(F("  "));
            //           //@@Serial.print(gpi.vy);//@@Serial.print(F("  "));
            //           //@@Serial.print(gpi.vz);//@@Serial.print(F("  "));
            //           //@@Serial.println(gpi.hdg);
          }
          break;
        case MAVLINK_MSG_ID_ATTITUDE:
          {
            //            mavlink_attitude_t at;
            //            mavlink_msg_attitude_decode(&msg, &at);
            //
            //@@Serial.println(F("at  :  "));
            //           //@@Serial.print(at.time_boot_ms);//@@Serial.print(F("  "));
            //           //@@Serial.print(at.roll);//@@Serial.print(F("  "));
            //           //@@Serial.print(at.pitch);//@@Serial.print(F("  "));
            //           //@@Serial.print(at.yaw);//@@Serial.print(F("  "));
            //           //@@Serial.print(at.rollspeed);//@@Serial.print(F("  "));
            //           //@@Serial.print(at.pitchspeed);//@@Serial.print(F("  "));
            //           //@@Serial.println(at.yawspeed);
          }
          break;
        case MAVLINK_MSG_ID_SYS_STATUS:
          {
            mavlink_sys_status_t ss;
            mavlink_msg_sys_status_decode(&msg, &ss);

            //@@Serial.print(F("ss  :  "));
            //@@Serial.println(ss.battery_remaining);
          }
          break;
        case MAVLINK_MSG_ID_MISSION_CURRENT:
          {
            //            mavlink_mission_current_t mc;
            //            mavlink_msg_mission_current_decode(&msg, &mc);
            //
            //                       //@@Serial.print(F("mc  :  "));
            //           //@@Serial.println(mc.seq);
          }
          break;
        default:
          //          Serial.println(msg.msgid);
          break;
      }
    }
  }
}

void init_vl() {

  Serial.print(F("Test sensors : "));
  pinMode(PIN6, OUTPUT);
  pinMode(PIN5, OUTPUT);
  pinMode(PIN4, OUTPUT);
  pinMode(PIN3, OUTPUT);
  pinMode(PIN2, OUTPUT);
  pinMode(PIN1, OUTPUT);
  digitalWrite(PIN1, LOW);
  digitalWrite(PIN2, LOW);
  digitalWrite(PIN3, LOW);
  digitalWrite(PIN4, LOW);
  digitalWrite(PIN5, LOW);
  digitalWrite(PIN6, LOW);
  delay(100);

  digitalWrite(PIN1, HIGH);  delay(100);
  v1.setAddress(0x30);  v1.init();  v1.setTimeout(500);

  digitalWrite(PIN2, HIGH);  delay(100);
  v2.setAddress(0x31);  v2.init();  v2.setTimeout(500);

  digitalWrite(PIN3, HIGH);  delay(100);
  v3.setAddress(0x32);  v3.init();  v3.setTimeout(500);

  digitalWrite(PIN4, HIGH);  delay(100);
  v4.setAddress(0x33);  v4.init();  v4.setTimeout(500);

  digitalWrite(PIN5, HIGH);  delay(100);
  v5.setAddress(0x34);  v5.init();  v5.setTimeout(500);

  digitalWrite(PIN6, HIGH);  delay(100);
  v6.setAddress(0x35);  v6.init();  v6.setTimeout(500);

  v1.startContinuous();
  v2.startContinuous();
  v3.startContinuous();
  v4.startContinuous();
  v5.startContinuous();
  v6.startContinuous();

  Serial.println(F("Success"));
}

char myStr[6];

void get_vl() {

  int mill = millis();

  dist[0] = v1.readRangeContinuousMillimeters();
  sprintf(myStr, "%6d", dist[0]);
  Serial.print(myStr);
  dist[1] = v2.readRangeContinuousMillimeters();
  sprintf(myStr, "%6d", dist[1]);
  Serial.print(myStr);
  dist[2] = v3.readRangeContinuousMillimeters();
  sprintf(myStr, "%6d", dist[2]);
  Serial.print(myStr);
  dist[3] = v4.readRangeContinuousMillimeters();
  sprintf(myStr, "%6d", dist[3]);
  Serial.print(myStr);
  dist[4] = v5.readRangeContinuousMillimeters();
  sprintf(myStr, "%6d", dist[4]);
  Serial.print(myStr);
  dist[5] = v6.readRangeContinuousMillimeters();
  sprintf(myStr, "%6d", dist[5]);
  Serial.print(myStr);

  sprintf(myStr, "%8d", millis() - mill);
  Serial.println(myStr);

  if (v1.timeoutOccurred() || v2.timeoutOccurred() || v3.timeoutOccurred() || v4.timeoutOccurred() || v5.timeoutOccurred() || v6.timeoutOccurred())
    Serial.println(F("TIMEOUT"));
}
