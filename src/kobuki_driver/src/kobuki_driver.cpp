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
	unsigned int packet_size = 7;

	/* PACOTE QUE TOCA MUSIQUINHA
    unsigned char packet[packet_size] = {
        0xAA, 0x55, // Header
        0x03,       // Lenght
        0x04,       // Sub-Payload 0 - Header
        0x01,       // Sub-Payload 1 - Lenght
        0x05,       // Sub-Payload 2 - CMD - Note
    };*/

	/* PACOTE QUE FAZ MOVER */
    unsigned char packet[packet_size] = {
        0xAA, 0x55, // Header
        0x06,       // Lenght
        0x01,       // Sub-Payload 0 - Header
        0x04,       // Sub-Payload 1 - Lenght
        0x80, 0x10, // Sub-Payload 2 e 3 - Speed
        0x70, 0x00, // Sub-Payload 4 e 5 - Radius
    };

    unsigned char cs = 0;
    
    for (unsigned int i = 2; i < packet_size; i++)
        cs ^= packet[i];

    packet[packet_size -1] = cs;

    for (int i = 0; i < packet_size; i ++)
        printf("\n Packet: 0x%02x", packet[i]);
    cout << endl;        

    int bytes_written;    
    for (int i = 0; i < 10; i++){
        bytes_written = write(fd, packet, packet_size);
        printf("written %d bytes\n", bytes_written);
    }
    
    printf("\n +----------------------------------+\n\n\n");
    close(fd); /* Close the serial port */

	return 0;

}


