#ifndef SETPID_FUNC_H
#define SETPID_FUNC_H

#define GET_BAUDRATE   'b'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define RESET_ENCODERS 'r'
#define UPDATE_PID     'u'
#define READ_PIDOUT  'f'
#define READ_PIDIN  'i'

#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define ANALOG_READ    'a'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'

#define LEFT            0
#define RIGHT           1
#define FORWARDS true
#define BACKWARDS false

#define PID_RASIO 50.0

int runCommand();
void resetCommand();
void receive_setPID();

#endif
