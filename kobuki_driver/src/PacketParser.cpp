#include "PacketParser.h"
#include "KobukiPacket.h"
#include "Buffer.h"

#include <stdio.h>
#include <string.h>

#include <ios>
#include <iostream>
#include <cstdlib>

#define PACKET_LEN 77

int counterxx = 0;
void treat_packet(KobukiPacket packet){

    counterxx = (counterxx + 1) % 50;

    if(counterxx != 30) return;

    unsigned int x; 
	// std::cout << "Header 0  : " << std::hex << (x = packet.header_0) << std::endl;
	// std::cout << "Header 1  : " << std::hex << (x = packet.header_1) << std::endl;
	// std::cout << "Payload   : " << std::hex << (x = packet.payload_len) << std::endl;

    std::cout << "BD Header : " << std::hex << (x = packet.basic_sensor_data.header)        << std::endl;
    // std::cout << "Timestamp : " << std::hex << (x = packet.basic_sensor_data.timestamp)     << std::endl;
    std::cout << "Bumper    : " << std::hex << (x = packet.basic_sensor_data.bumper)        << std::endl;
    // std::cout << "Wheel Drop: " << std::hex << (x = packet.basic_sensor_data.wheel_drop)    << std::endl;
    // std::cout << "Cliff     : " << std::hex << (x = packet.basic_sensor_data.cliff)    << std::endl;
    // std::cout << "Encoder L : " << std::hex << (x = packet.basic_sensor_data.encoder_left)  << std::endl;
    // std::cout << "Encoder R : " << std::hex << (x = packet.basic_sensor_data.encoder_right) << std::endl;
    std::cout << "Buttonn   : " << std::hex << (x = packet.basic_sensor_data.button) << std::endl;

}

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

KobukiPacket parse_packet(Buffer* buffer){

    //a new packet to be parsed
    union {
        KobukiPacket data;
        uint8_t raw[PACKET_LEN];
    } packet;

    //pops packets from the buffer
    int parsed_bytes = 0;
    
    while(true){

        if(parsed_bytes == PACKET_LEN)
            break;

        if(buffer->size() == 0)
            continue;

        packet.raw[parsed_bytes] = buffer->top();
        buffer->pop();

        parsed_bytes++;
    }
    
    packet.data.header_0 = 0xAA;
    treat_packet(packet.data);
}