#ifndef __KOBUKIPACKET_H
#define __KOBUKIPACKET_H

typedef struct{
    uint8_t header;
    uint8_t length;
    uint16_t timestamp;
    uint8_t bumper;
    uint8_t wheel_drop;
    uint8_t cliff;
    uint16_t encoder_left;
    uint16_t encoder_right;
    uint8_t pwm_left;
    uint8_t pwm_right;
    uint8_t button;
    uint8_t charger;
    uint8_t battery;
    uint8_t overcurrent_flags;
} BasicSensorDataPacket;

typedef struct{
    uint8_t id;
    uint8_t size;
    uint8_t signal_right;
    uint8_t signal_central;
    uint8_t signal_left;
} DockingIrPacket;

typedef struct{
    uint8_t id;
    uint8_t size;
    uint16_t angle;
    uint16_t angle_rate;
    uint8_t unused[3];
}  InertialSensorPacket;

typedef struct {
    uint8_t id;
    uint8_t size;
    uint16_t right_cliff_sensor;
    uint16_t central_cliff_sensor;
    uint16_t left_cliff_sensor;
} CliffSensorPacket;

typedef struct {
    uint8_t id;
    uint8_t size;
    uint16_t left_motor;
    uint16_t right_motor;
} CurrentPacket;


/**
	TODO:
		a) tem que fazer a estrutura do pacote de gyro
		b) tem que adicionar os packets abaixo na ordem certa,
		   pois somente o primeiro está correto até então
		c) tem que ler de X em X bytes. X é o tamanho total do pacotão

**/
typedef struct{

	uint8_t header_0;    //0xAA
	uint8_t header_1;    //0x55
	uint8_t payload_len; //0x??
	
	BasicSensorDataPacket basic_sensor_data;   //? bytes
	DockingIrPacket       docking_ir_data;     //? bytes
	InertialSensorPacket  inetial_sensor_data; //? bytes

	CliffSensorPacket     cliff_sensor_data;   //? bytes
	CurrentPacket         current_data;        //? bytes

<<<<<<< HEAD
    //RawDataGyro
    //GeneralPurpose

    uint8_t checksum;

=======
>>>>>>> 6519131129b8b6def4554cb708eeac33c12b78ea
} KobukiPacket;


#endif /* KOBUKIDRIVER */