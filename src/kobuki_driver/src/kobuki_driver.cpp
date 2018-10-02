/*====================================================================================================*/
/* Kobuki driver                                                                                      */
/* Using:                                                                                             */
/* Serial Port Programming in C - Non Cannonical mode                                                 */
/*----------------------------------------------------------------------------------------------------*/
/* Program ... <brief description>                                                                    */
/*----------------------------------------------------------------------------------------------------*/
/* Compiler/IDE  : gcc 4.6.3                                                                          */
/* Library       :                                                                                    */
/* Commands      : gcc -o kobuki_driver kobuki_driver.c                                               */
/* OS            : Linux(x86) (Linux Mint 13 Maya)(Linux Kernel 3.x.x)                                */                              
/* Programmer    : Anderson Domingues & Darlan Alves Jurak                                            */
/* Date	         : 24 September 2018                                                                  */
/*====================================================================================================*/

/*====================================================================================================*/
/* Basic serial communication from:  					                              */
/* www.xanthium.in				                                                      */
/* Copyright (C) 2014 Rahul.S                                                                         */
/*====================================================================================================*/

/*====================================================================================================*/
/* Running the executable                                                                             */
/* ---------------------------------------------------------------------------------------------------*/ 
/* 1) Compile the  kobuki_driver.c  file using gcc on the terminal (without quotes)                   */
/*                                                                                                    */
/*	" gcc -o kobuki_driver kobuki_driver.c "                                                      */
/*                                                                                                    */
/* 2) Linux will not allow you to access the serial port from user space,you have to be root.So use   */
/*    "sudo" command to execute the compiled binary as super user.                                    */
/*                                                                                                    */
/*       "sudo ./kobuki_driver"                                                                       */
/*                                                                                                    */
/*====================================================================================================*/
/*====================================================================================================*/
/* Sellecting the Serial port Number on Linux                                                         */
/* ---------------------------------------------------------------------------------------------------*/ 
/* /dev/ttyUSBx - when using USB to Serial Converter, where x can be 0,1,2...etc                      */
/* /dev/ttySx   - for PC hardware based Serial ports, where x can be 0,1,2...etc                      */
/*====================================================================================================*/

/*-------------------------------------------------------------*/
/* termios structure -  /usr/include/asm-generic/termbits.h    */ 
/* use "man termios" to get more info about  termios structure */
/*-------------------------------------------------------------*/

#include <cstring>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	       */ 
#include <errno.h>   /* ERROR Number Definitions           */
#include <stdint.h>
#include <cstdlib>

#include "Include/Buffer.h"

using namespace std;

#define SOUND_PACKET_SIZE 7
#define IN_BUFFER_SIZE 40     /* in buffer has 20 words */
#define CFG_DELAY 1           /* wait for 3 seconds before recvn' packages */

enum State { //Basic Sensor Data Feedback - 50 Hz

    IDLE,               HEADER,         LENGTH,
    TIMESTAMP_0,        TIMESTAMP_1,    BUMPER,             WHEEL_DROP, CLIFF,
    ENCODER_LEFT_0,     ENCODER_LEFT_1, 
    ENCODER_RIGHT_0,    ENCODER_RIGHT_1,
    PWM_LEFT,           PWM_RIGHT,      BUTTON,
    CHARGER,            BATTERY,        OVERCURRENT_FLAGS
};

State read_current_state = IDLE;

void SerialConfig(int*);
void MountPacket_Twist(unsigned char*, unsigned int, int, int, int, int, int, int);
void MountPacket_Sound(unsigned char*, unsigned int);
// serial data receive
static void* p_read(void*);
void state_machine();
void p_write(int* serial_handler);
void p_read(int* serial_handler);

Buffer* buffer;     // stores serail packets
int serial_handler; // serial handler (file descriptor)

//startup
int main(void)
{
    SerialConfig(&serial_handler);  /* Serial configuration */

    printf("\n +----------------------------------+");
    printf("\n |        Kobuki Driver             |");
    printf("\n +----------------------------------+\n");

	// buffer initialization
	buffer = new Buffer();

	// in a separated thread, bufferize the serial data
    pthread_t t;
    if(pthread_create(&t, NULL, &p_read, NULL)){
        cout << "error creating thread" <<endl;
    }

    for (;;){   
       
        state_machine();
        //sleep(CFG_DELAY);
    }

    close(serial_handler); /* Close the serial port */
    return 0;

}

// serial data receive
static void* p_read(void* dummy){

    uint8_t data;   // byte read
    int read_bytes; // amount of read bytes

    while(true){
        
        // serial read
        read_bytes = read(serial_handler, &data, 1);
        
        // avaliates serial read
        if(read_bytes < 1){

            cout << "";

        }else {

            buffer->push(data);

        }
        
    }
}

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

void state_machine(){

    uint8_t data;

    data = buffer->pop();
    
    switch(read_current_state){

        case IDLE:

            if(data == 0x01){
                read_current_state = HEADER;
            }
            break;

        case HEADER:

            if(data == 0x0F)
                read_current_state = LENGTH;
            else
                read_current_state = IDLE;

        case LENGTH:

            printf("\n=============================");
            // printf("\nTimestamp 0: 0x%02x ", data);
            read_current_state = TIMESTAMP_0;
            break;
        
        case TIMESTAMP_0:

            // printf("\nTimestamp 1: 0x%02x ", data);
            read_current_state = TIMESTAMP_1;
            break;

        case TIMESTAMP_1:

            printf("\nBumper: 0x%02x ", data);
            read_current_state = BUMPER;
            break;

        case BUMPER:

            printf("\nWheel Drop: 0x%02x ", data);
            read_current_state = WHEEL_DROP;
            break;

        case WHEEL_DROP:

            // printf("\nCliff: 0x%02x ", data);
            read_current_state = CLIFF;
            break;

        case CLIFF:

            printf("\nEncoder Left 0: 0x%02x ", data);
            read_current_state = ENCODER_LEFT_0;
            break;

        case ENCODER_LEFT_0:

            printf("\nEncoder Left 1: 0x%02x ", data);
            read_current_state = ENCODER_LEFT_1;
            break;

        case ENCODER_LEFT_1:

            printf("\nEncoder Right 0: 0x%02x ", data);
            read_current_state = ENCODER_RIGHT_0;
            break;

        case ENCODER_RIGHT_0:

            printf("\nEncoder Right 1: 0x%02x ", data);
            read_current_state = ENCODER_RIGHT_1;
            break;

        case ENCODER_RIGHT_1:

            // printf("\nPWM Left: 0x%02x ", data);
            read_current_state = PWM_LEFT;
            break;
        
        case PWM_LEFT:

            // printf("\nPWM Right: 0x%02x ", data);
            read_current_state = PWM_RIGHT;
            break;

        case PWM_RIGHT:

            // printf("\nButton: 0x%02x ", data);
            read_current_state = BUTTON;
            break;

        case BUTTON:

            // printf("\nCharger: 0x%02x ", data);
            read_current_state = CHARGER;
            break;

        case CHARGER:

            // printf("\nBattery: 0x%02x ", data);
            read_current_state = BATTERY;
            break;

        case BATTERY:

            // printf("\nOvercurrent Flags: 0x%02x ", data);
            read_current_state = IDLE;
            break;

        default: break;
    }

}


void p_write(int* serial_handler){

    unsigned char* out_pkt = (unsigned char*) malloc(sizeof(unsigned char) * SOUND_PACKET_SIZE);
    MountPacket_Sound(out_pkt, SOUND_PACKET_SIZE);

    write(*serial_handler, out_pkt, SOUND_PACKET_SIZE);
    free(out_pkt);
}

void SerialConfig(int* serial_handler){

    *serial_handler = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
                            
    if (serial_handler < 0)
		cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << endl;

    struct termios tty;	                /* Create the structure                                     */
    tcgetattr(*serial_handler, &tty);   /* Get the current attributes of the Serial port            */

    /* Setting the Baud rate */                     
    cfsetispeed(&tty, B115200);         /* Set Read  Speed as 9600                                  */
    cfsetospeed(&tty, B115200);         /* Set Write Speed as 9600                                  */
        
    /* 8N1 Mode */                  
    tty.c_cflag &= ~PARENB;             /* Disables the Parity Enable bit(PARENB),So No Parity      */
    tty.c_cflag &= ~CSTOPB;             /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit    */
    tty.c_cflag |=  CS8;                /* Set the data bits = 8                                    */

    printf(
        ((tcsetattr(*serial_handler, TCSANOW, &tty)) != 0)
            ? "\n  ERROR ! in Setting attributes"
            : "\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none"
    );

    tcflush(*serial_handler, TCIFLUSH);                     /* Discards old data in the rx buffer            */
}

void MountPacket_Twist( unsigned char* packet,
                        unsigned int packet_size,
                        int velocity_x=0, int angle_x=0,
                        int velocity_y=0, int angle_y=0,
                        int velocity_z=0, int angle_z=0){

    unsigned char packet_checksum = 0;
    packet = (unsigned char*) malloc(packet_size * sizeof(unsigned char));

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
