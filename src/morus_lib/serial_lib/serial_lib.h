/* serial_lib.h */
/**
  *
  *  @file serial_lib.h
  *  @brief High level serial interface for communication with STM32F4 Discovery
  *         board.
  *  @author Matija Bedeković, <matija.bedekovic2@fer.hr>
  *
  */

#ifndef SERIAL_LIB_H
#define SERIAL_LIB_H

#include <fcntl.h>                          //File control
#include <termios.h>                        //POSIX terminal control
#include <pthread.h>                        //POSIX threads for paralel execution in
                                            //single system task (paralel Rx and Tx
#include <stdio.h>                          //Standard I/O from C
#include <unistd.h>                         //POSIX API - read, write, close etc.
#include <stdlib.h>                         //For memory allocation functions
#include <string.h>                         //memcpy
//Defines
#ifndef B115200
#define B115200 115200
#endif

#ifndef B57600
#define B57600  57600
#endif

#define SERIAL_PORT_OPEN        1
#define SERIAL_PORT_CLOSED      0
#define SERIAL_PORT_ERROR      -1

typedef struct
{
    int motor1;                  //4 byte
    int motor2;                  //4 byte
    int motor3;                  //4 byte
    int motor4;                  //4 byte
    uint8_t cmd;                 //1 byte + 3 bytes of padding (account for it in memcpy)
} motor_control_t;

class SerialPort
{
public:
    SerialPort();
    ~SerialPort();
    void initDefault();

    //public data data members
    const char *uart_name;
    int baudrate;
    int status;

    //public class interface
    int readMessage(char* message, size_t nbytes);      //reading from serial port
    int writeMessage(uint8_t *message, size_t len);     //writing to serial port

    void openSerial();
    void closeSerial();
    bool sendMotorControl(motor_control_t *msg);        //Sending serialized data

private:
    int fd;                                     //file descriptor
    pthread_mutex_t lock;

    uint8_t *outputBuffer;                   //output buffer memory pointer
    size_t outputBufferSize;                 //output buffer memory size in bytes

    //private setup functions, accses through public interface
    int _open_port(const char* port);
    bool _setup_port(int baud);                                 //port setup fnc
    int _read_port(char* cp, size_t nbytes);                    //read from port
    int _write_port(const uint8_t *buf, size_t len);            //write from buffer
    void _serialize_motor_control_t(motor_control_t *msg);      //serialize motor control data
};


#endif // SERIAL_LIB_H
