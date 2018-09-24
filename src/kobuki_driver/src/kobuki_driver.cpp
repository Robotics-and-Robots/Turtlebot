/*====================================================================================================*/
/* Serial Port Programming in C (Serial Port Read)                                                    */
/* Non Cannonical mode                                                                                */
/*----------------------------------------------------------------------------------------------------*/
/* Program reads a string from the serial port at 9600 bps 8N1 format                                 */
/* Baudrate - 9600                                                                                    */
/* Stop bits -1                                                                                       */
/* No Parity                                                                                          */
/*----------------------------------------------------------------------------------------------------*/
/* Compiler/IDE  : gcc 4.6.3                                                                          */
/* Library       :                                                                                    */
/* Commands      : gcc -o serialport_read serialport_read.c                                           */
/* OS            : Linux(x86) (Linux Mint 13 Maya)(Linux Kernel 3.x.x)                                */                              
/* Programmer    : Rahul.S                                                                            */
/* Date	         : 21-December-2014                                                                   */
/*====================================================================================================*/

/*====================================================================================================*/
/* www.xanthium.in										      */
/* Copyright (C) 2014 Rahul.S                                                                         */
/*====================================================================================================*/

/*====================================================================================================*/
/* Running the executable                                                                             */
/* ---------------------------------------------------------------------------------------------------*/ 
/* 1) Compile the  serialport_read.c  file using gcc on the terminal (without quotes)                 */
/*                                                                                                    */
/*	" gcc -o serialport_read serialport_read.c "                                                  */
/*                                                                                                    */
/* 2) Linux will not allow you to access the serial port from user space,you have to be root.So use   */
/*    "sudo" command to execute the compiled binary as super user.                                    */
/*                                                                                                    */
/*       "sudo ./serialport_read"                                                                     */
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
    #include <unistd.h>  /* UNIX Standard Definitions 	   */ 
    #include <errno.h>   /* ERROR Number Definitions           */
    #include <stdint.h>

using namespace std;

int main(void)
{
    int fd;

    printf("\n +----------------------------------+");
    printf("\n |        Serial Port Read          |");
    printf("\n +----------------------------------+");

    fd = open("/dev/ttyUSB0",O_WRONLY | O_NOCTTY | O_NONBLOCK | O_NDELAY);	
                            
	if (fd < 0)
		cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << endl;

    struct termios tty;	/* Create the structure                          */
    tcgetattr(fd, &tty);	/* Get the current attributes of the Serial port */

    /* Setting the Baud rate */
    cfsetispeed(&tty, B115200); /* Set Read  Speed as 9600                       */
    cfsetospeed(&tty, B115200); /* Set Write Speed as 9600                       */

    /* 8N1 Mode */
    tty.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    tty.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    tty.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

    if((tcsetattr(fd, TCSANOW, &tty)) != 0) /* Set the attributes to the termios structure*/
        printf("\n  ERROR ! in Setting attributes");
    else
        printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none");
        
    tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */
	unsigned int packet_size = 13;
	//unsigned int packet_size = 7;
	

	/* PACOTE QUE TOCA MUSIQUINHA */
    unsigned char packet[packet_size];
/*
    packet[0] = 0xAA;
    packet[1] = 0x55;
    packet[2] = 0x03;
    packet[3] = 0x04;
    packet[4] = 0x01;
    packet[5] = 0x04;



	 PACOTE QUE FAZ MOVER */

for (int i = 0; i < 100; i++){

	packet[0] = 0xAA; // Header 0
    packet[1] = 0x55; // Header 1
    packet[2] = 0x09; // Lenght
    
    //payload move
    packet[3] = 0x01; // Sub-Payload 0 - Header
    packet[4] = 0x04; // Sub-Payload 1 - Lenght
    
    uint16_t* vel = (uint16_t*)&packet[5];
    *vel = i * 50;
    
    //packet[5] = 0x90; // Sub-Payload 2 - Speed
    //packet[6] = 0xFF; // Sub-Payload 3 - Speed
    
    uint16_t* rad = (uint16_t*)&packet[7];
    *rad = i * 50; 
    //packet[7] = 0x70; // Sub-Payload 4 - Radius
    //packet[8] = 0x20; // Sub-Payload 5 - Radius

	//payload musiquinha
    packet[9] = 0x04;
    packet[10] = 0x01;
    packet[11] = 0x02;

    unsigned char cs = 0;
    
    for (unsigned int i = 2; i < packet_size; i++)
        cs ^= packet[i];

    packet[packet_size -1] = cs;

    for (int i = 0; i < packet_size; i ++)
        printf("\n Packet: 0x%02x", packet[i]);
    cout << endl;        

    int bytes_written;    
    bytes_written = write(fd, packet, packet_size);
    printf("written %d bytes\n", bytes_written);
    
    sleep(3);
}
    printf("\n +----------------------------------+\n\n\n");
    close(fd); /* Close the serial port */

	return 0;

}


