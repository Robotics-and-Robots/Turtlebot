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

#include <cstring>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	       */ 
#include <errno.h>   /* ERROR Number Definitions           */
#include <stdint.h>
#include <cstdlib>

#include "Buffer.h"
#include "KobukiPacket.h"
#include "PacketParser.h"


using namespace std;

#define serial_close(A) close(A)

#define SOUND_PACKET_SIZE 7
#define IN_BUFFER_SIZE 40     /* in buffer has 20 words */
#define CFG_DELAY 1           /* wait for 3 seconds before recvn' packages */


void serial_init(int*);
void state_machine();

Buffer* buffer;     // stores serial packets
int serial_handler; // serial handler (file descriptor)

/** SERIAL_INIT
  * Function that configures serial port to
  * interact with kobuki.*/
void serial_init(int* serial_handler){

    *serial_handler = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
                            
    if (serial_handler < 0)
		cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << endl;

    struct termios tty;
    tcgetattr(*serial_handler, &tty);

    //Setting the Baud rate
    cfsetispeed(&tty, B115200);  //read speed  = 115200
    cfsetospeed(&tty, B115200);  //write speed = 115200
        
    //8N1 Mode
    tty.c_cflag &= ~PARENB;  //no parity
    tty.c_cflag &= ~CSTOPB;  //1 stop byte
    tty.c_cflag |=  CS8;     //data bits = 8

	//print configurations
    printf(
        ((tcsetattr(*serial_handler, TCSANOW, &tty)) != 0)
            ? "\n  ERROR ! in Setting attributes"
            : "\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none"
    );

    tcflush(*serial_handler, TCIFLUSH); //Discards old data in the rx buffer
}


/** SERIAL_MONITORING_THREAD 
  * Function that read data from serial port 
  * and feed the packet buffer. Data is read
  * byte-to-byte (1x uint8_t per tick). Buffer
  * data is consumed accoding to the configured 
  * packet len.*/
static void* serial_monitoring_thread(void* dummy){

    uint8_t data;   // byte to be read
    uint8_t read_bytes; // amount of read bytes (must be 1)

	//thread runs indefinitely
    do{
		//read_bytes must be always 1, otherwise 
		//the reading failed 
        read_bytes = read(serial_handler, &data, 1);
        
        if(read_bytes != 1){
            //unable to get data, try again in next cycle
            
        }else {
        
        	//data read succefully, fill the buffer
            buffer->push(data);
        }
        
    }while(true);
}


//startup
int main(void)
{
    serial_init(&serial_handler);  /* Serial configuration */

    printf("\n +----------------------------------+");
    printf("\n |        Kobuki Driver             |");
    printf("\n +----------------------------------+\n");

	// buffer initialization
	buffer = new Buffer();

	// in a separated thread, bufferize the serial data
    pthread_t t;
    
    if(pthread_create(&t, NULL, &serial_monitoring_thread, NULL)){
        cout << "error creating thread" <<endl;
    }

	//keep parsing packets until the end of the program
    for (;;){   
       
        parse_packet(buffer);

        //sleep(CFG_DELAY);
    }

    serial_close(serial_handler); /* Close the serial port */
    return 0;

}


/**
void p_write(int* serial_handler){

    unsigned char* out_pkt = (unsigned char*) malloc(sizeof(unsigned char) * SOUND_PACKET_SIZE);
    MountPacket_Sound(out_pkt, SOUND_PACKET_SIZE);

    write(*serial_handler, out_pkt, SOUND_PACKET_SIZE);
    free(out_pkt);
}
**/