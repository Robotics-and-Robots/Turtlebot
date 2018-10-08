#ifndef __KOBUKIPACKET_H
#define __KOBUKIPACKET_H

typedef struct{
    uint8_t id;
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

#endif /* KOBUKIDRIVER */