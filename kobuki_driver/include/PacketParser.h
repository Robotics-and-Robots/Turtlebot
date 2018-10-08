#ifndef _PACKETPARSER_H
#define _PACKETPARSER_H

#include "Buffer.h"



/** PARSE_PACKET
  * Consume XX bytes from the buffer and allocate memory
  * to a new packet. */
void parse_packet(Buffer* buf);


/*  
**  enum State { //Basic Sensor Data Feedback - 50 Hz  
**     
**      IDLE,               HEADER,         LENGTH,   
**      TIMESTAMP_0,        TIMESTAMP_1,    BUMPER,             WHEEL_DROP, CLIFF,  
**      ENCODER_LEFT_0,     ENCODER_LEFT_1,    
**      ENCODER_RIGHT_0,    ENCODER_RIGHT_1,   
**      PWM_LEFT,           PWM_RIGHT,      BUTTON,   
**      CHARGER,            BATTERY,        OVERCURRENT_FLAGS   
**  }; 
*/  

enum State { //Basic Sensor Data Feedback - 50 Hz

    IDLE,               HEADER_0, HEADER_1,         LENGTH,
    SUB_HEADER, SUB_LENGTH,  
    TIMESTAMP_0,        TIMESTAMP_1,    BUMPER,             WHEEL_DROP, CLIFF,
    ENCODER_LEFT_0,     ENCODER_LEFT_1, 
    ENCODER_RIGHT_0,    ENCODER_RIGHT_1,
    PWM_LEFT,           PWM_RIGHT,      BUTTON,
    CHARGER,            BATTERY,        OVERCURRENT_FLAGS
};

void MountPacket_Twist(unsigned char*, unsigned int, int, int, int, int, int, int);
void MountPacket_Sound(unsigned char*, unsigned int);

#endif /* _PACKETPARSER_H */