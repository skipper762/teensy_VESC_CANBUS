#include "vesc_can.h"



// Global CANBUS variables
FlexCAN CAN(500000);

//Can processing examples
int can_idle = 0;
uint8_t rx_buffer[RX_BUFFER_SIZE];
unsigned int rx_buffer_last_id;

//short value messages 
can_status_msg stat_msgs[CAN_STATUS_MSGS_TO_STORE];

void vesc_can_begin() {
  CAN.begin();
}

void vesc_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void comm_can_set_current_brake(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

void comm_can_get_values(uint8_t controller_id) {
  can_idle = 0;
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer[send_index++] = 1;
  buffer[send_index++] = 0;         // make bollean send to false on VESC
  buffer[send_index++] = COMM_GET_VALUES;
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), buffer, send_index);
}


int vesc_can_read() {
  CAN_message_t  inMsg;
  int32_t ind = 0;
  unsigned int rxbuf_len;
  unsigned int rxbuf_ind;
  uint8_t crc_low;
  uint8_t crc_high;

  //can_status_message storage
  int ijk;
  can_status_msg *stat_tmp;

  if (!CAN.read(inMsg)) {
    return 0; // if the message cant be read return 0
  }
  if (inMsg.ext == 1) {
    uint8_t id = inMsg.id & 0xFF; //take the lower 8 bits for the ID
    CAN_PACKET_ID cmd = (CAN_PACKET_ID) (inMsg.id >> 8); // Take the upper bits as the comand
    switch (cmd) {
      case CAN_PACKET_SET_DUTY:
        ind = 0;

        break;

      case CAN_PACKET_SET_CURRENT:
        ind = 0;

        break;

      case CAN_PACKET_SET_CURRENT_BRAKE:
        ind = 0;
        break;

      case CAN_PACKET_SET_RPM:
        ind = 0;
        break;

      case CAN_PACKET_SET_POS:
        ind = 0;
        break;
      case CAN_PACKET_FILL_RX_BUFFER:
        memcpy(rx_buffer + inMsg.buf[0], inMsg.buf + 1, inMsg.len - 1);
        break;
      case CAN_PACKET_FILL_RX_BUFFER_LONG:
        rxbuf_ind = (unsigned int)inMsg.buf[0] << 8;
        rxbuf_ind |= inMsg.buf[1];
        if (rxbuf_ind < RX_BUFFER_SIZE) {
          memcpy(rx_buffer + rxbuf_ind, inMsg.buf + 2, inMsg.len - 2);
        }
        break;

      case CAN_PACKET_PROCESS_RX_BUFFER:
        ind = 0;
        rx_buffer_last_id = inMsg.buf[ind++];
        inMsg.buf[ind++];
        rxbuf_len = (unsigned int)inMsg.buf[ind++] << 8;
        rxbuf_len |= (unsigned int)inMsg.buf[ind++];

        if (rxbuf_len > RX_BUFFER_SIZE) {
          break;
        }

        crc_high = inMsg.buf[ind++];
        crc_low = inMsg.buf[ind++];

        if (crc16(rx_buffer, rxbuf_len)
            == ((unsigned short) crc_high << 8
                | (unsigned short) crc_low)) {

          //                  if (commands_send) {
          //                     send_packet_wrapper(rx_buffer, rxbuf_len);
          //                  } else {
          can_process_packet(rx_buffer, rxbuf_len);
          //                  }
        }
        break;

      case CAN_PACKET_PROCESS_SHORT_BUFFER:
        ind = 0;
        rx_buffer_last_id = inMsg.buf[ind++];
        inMsg.buf[ind++];
        can_process_packet(inMsg.buf + ind, inMsg.len - ind);
        break;

      case CAN_PACKET_STATUS:
        for (ijk = 0; ijk < CAN_STATUS_MSGS_TO_STORE; ijk++) {
          stat_tmp = &stat_msgs[ijk];
          if (stat_tmp->id == id || stat_tmp->id == -1) {
            ind = 0;
            stat_tmp->id = id;
            stat_tmp->rpm = (float)buffer_get_int32(inMsg.buf, &ind);
            stat_tmp->current = (float)buffer_get_int16(inMsg.buf, &ind) / 10.0;
            stat_tmp->duty = (float)buffer_get_int16(inMsg.buf, &ind) / 1000.0;
            break;
          }
          
        }
        

      default:
        break;

    }


  }
  return 1;



}



//Helper methods
bool sendPacket(uint8_t id, uint8_t packet[], int32_t len) {
  CAN_message_t  msg;
  msg.id = id;
  msg.len = len;
  memcpy(msg.buf, packet, len * sizeof(uint8_t));
  msg.timeout = 0;
  return (bool) CAN.write(msg);
}

void can_process_packet(unsigned char *data, unsigned int len){
   bldc_interface_process_packet(data,len);
   can_idle=1;
}

unsigned short crc16(unsigned char *buf, unsigned int len) {
  unsigned int i;
  unsigned short cksum = 0;
  for (i = 0; i < len; i++) {
    cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
  }
  return cksum;
}


