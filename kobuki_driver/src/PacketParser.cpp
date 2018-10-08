#include "PacketParser.h"
#include "KobukiPacket.h"
#include "Buffer.h"

#include <stdio.h>
#include <string.h>

#include <ios>
#include <iostream>
#include <cstdlib>

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



void MountPacket_Twist( unsigned char* packet,
                        unsigned int packet_size,
                        int velocity_x=0, int angle_x=0,
                        int velocity_y=0, int angle_y=0,
                        int velocity_z=0, int angle_z=0){

    unsigned char packet_checksum = 0;
    packet = new unsigned char[packet_size];

    /* PACOTE QUE FAZ MOVER */
    packet[0] = 0xAA; // Header 0
    packet[1] = 0x55; // Header 1
    packet[2] = 0x09; // Lenght
    
    //payload move
    packet[3] = 0x01; // Sub-Payload 0 - Header
    packet[4] = 0x04; // Sub-Payload 1 - Lenght

    //packet[5] = 0x90; // Sub-Payload 2 - Speed
    //packet[6] = 0xFF; // Sub-Payload 3 - Speed
    uint16_t* vel = (uint16_t*)&packet[5];
    *vel = velocity_y * 50;
    
    //packet[7] = 0x70; // Sub-Payload 4 - Radius
    //packet[8] = 0x20; // Sub-Payload 5 - Radius
    uint16_t* rad = (uint16_t*)&packet[7];
    // *rad = i * 50;
    *rad = 5 * 0;

    //payload musiquinha
    packet[9] = 0x04;
    packet[10] = 0x01;
    packet[11] = 0x02;

    packet_checksum = 0;
    for (unsigned int i = 2; i < packet_size; i++)
        packet_checksum ^= packet[i];

    packet[packet_size -1] = packet_checksum;

}

void MountPacket_Sound(unsigned char* packet, unsigned int packet_size){

    unsigned char packet_checksum = 0;

    /* PACOTE QUE TOCA MUSIQUINHA */
    packet[0] = 0xAA;  //header 0
    packet[1] = 0x55;  //header 1
    packet[2] = 0x03;  //length
    packet[3] = 0x04;  //id
    packet[4] = 0x01;  //size 
    packet[5] = 0x01;  //tune

    packet_checksum = 0;
    for (unsigned int i = 2; i < packet_size; i++)
        packet_checksum ^= packet[i];

    packet[packet_size -1] = packet_checksum;
}

#define PACKET_LEN 77
void parse_packet(Buffer* buffer){

    //a new packet to be parsed
    union {
        KobukiPacket data;
        uint8_t raw[PACKET_LEN];
    } packet;

    //pops packets from the buffer
    int parsed_bytes = 0;
    
    int x; 

    while(true){

        if(parsed_bytes == PACKET_LEN)
            break;

        if(buffer->size() == 0)
            continue;

        packet.raw[parsed_bytes] = buffer->top();
        buffer->pop();
        
        //std::cout << std::hex << (x = packet.raw[parsed_bytes]) << std::endl;
        
        parsed_bytes++;
    }
    
	std::cout << "header0 = 0x" << std::hex << (x = packet.raw[0]) << std::endl;
	std::cout << "header1 = 0x" << std::hex << (x = packet.raw[1]) << std::endl;
	std::cout << "payload = 0x" << std::hex << (x = packet.raw[2]) << std::endl;
}


/*
=======
void parse_packet(Buffer* buffer){

>>>>>>> 6519131129b8b6def4554cb708eeac33c12b78ea
	State read_current_state = HEADER_0;

    uint8_t data;
    int bumper;
    int encoder_left;
    int encoder_right;
    data = buffer->pop();
    
    switch(read_current_state){

        case HEADER_0:

            if(data == 0xAA){
                // printf("Achei 0xAA");
                read_current_state = HEADER_1;
            }
            break;

        case HEADER_1:

            if(data == 0x55){
                read_current_state = LENGTH;
            }else{
                read_current_state = HEADER_0;
            }
            break;

        case LENGTH:
        
            if(data == 0x4d){
                printf("\nLength: 0x%02x ", data);
                read_current_state = SUB_HEADER;
            }else
                read_current_state = HEADER_0;
            break;

        case SUB_HEADER:

            printf("\n==========ID: 0x%02x ", data);
            
            if(data == 0x01){
                read_current_state = SUB_LENGTH;
            }
            break;

        case SUB_LENGTH:

            if(data == 0x0F)
                read_current_state = TIMESTAMP_0;
            else
                read_current_state = HEADER_0;
            break;

        case TIMESTAMP_0:

            printf("\n=============================");
            // printf("\nTimestamp 0: 0x%02x ", data);
            read_current_state = TIMESTAMP_1;
            break;
        
        case TIMESTAMP_1:

            // printf("\nTimestamp 1: 0x%02x ", data);
            read_current_state = BUMPER;
            break;

        case BUMPER:
            bumper = data;
            
            if (data > 7)

                read_current_state = HEADER_0;

            else{

                read_current_state = WHEEL_DROP;
                printf("\nBumper: 0x%02x ", bumper);

            }
            break;

        case WHEEL_DROP:

            printf("\nWheel Drop: 0x%02x ", data);
            if (data > 3)
                read_current_state = HEADER_0;
            else
                read_current_state = CLIFF;
            break;

        case CLIFF:

            // printf("\nCliff: 0x%02x ", data);
            read_current_state = ENCODER_LEFT_0;
            break;

        case ENCODER_LEFT_0:
            encoder_left = data;
            //printf("\nEncoder Left 0: 0x%02x ", data);
            read_current_state = ENCODER_LEFT_1;
            break;

        case ENCODER_LEFT_1:
            encoder_left = encoder_left << 8;
            encoder_left = encoder_left & data; //?
            printf("\nEncoder Left 1: 0x%02x ", encoder_left);
            read_current_state = ENCODER_RIGHT_0;
            break;

        case ENCODER_RIGHT_0:

            printf("\nEncoder Right 0: 0x%02x ", data);
            read_current_state = ENCODER_RIGHT_1;
            break;

        case ENCODER_RIGHT_1:

            printf("\nEncoder Right 1: 0x%02x ", data);
            read_current_state = PWM_LEFT;
            break;

        case PWM_LEFT:

            // printf("\nPWM Left: 0x%02x ", data);
            read_current_state = PWM_RIGHT;
            break;
        
        case PWM_RIGHT:

            // printf("\nPWM Right: 0x%02x ", data);
            read_current_state = BUTTON;
            break;

        case BUTTON:

            // printf("\nButton: 0x%02x ", data);
            read_current_state = CHARGER;
            break;

        case CHARGER:

            // printf("\nCharger: 0x%02x ", data);
            read_current_state = BATTERY;
            break;

        case BATTERY:

            // printf("\nBattery: 0x%02x ", data);
            read_current_state = OVERCURRENT_FLAGS;
            break;

        case OVERCURRENT_FLAGS:

            // printf("\nOvercurrent Flags: 0x%02x ", data);
            read_current_state = HEADER_0;
            break;

        default: break;
    }

<<<<<<< HEAD
}
*/