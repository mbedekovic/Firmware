/* serial_lib.cpp */
/**
  * @file serial_lib.cpp
  * @brief Serial interface function implementation
  *        Functions for opening, closing, reading and writing via serial ports
  * @author Matija Bedeković, <matija.bedekovic2@fer.hr>
  *
  */

#include "serial_lib.h"

//Constructor
SerialPort::SerialPort()
{
    initDefault();

    outputBuffer = (uint8_t*)malloc(sizeof(motor_control_t));     //Allocate initial minimal buffer size;
    if (outputBuffer == NULL)
    {
        fprintf(stderr,"\nERROR: Could not allocate memory for buffer\n");
        /**
          * @todo Should handle allocation problems
          */
        return;
    }
    outputBufferSize = sizeof(motor_control_t);                   //Allocated memory size
}

//Destructor
SerialPort::~SerialPort()
{
    pthread_mutex_destroy(&lock);           //Kill active mutex ITT
    free(outputBuffer);                     //Free allocated memory
    outputBufferSize = 0;
}

void SerialPort::initDefault()
{
    //Setup default member values
    fd         = -1;                        //File descriptor
    status     = SERIAL_PORT_CLOSED;        //Status of serial port closed
    uart_name  = (char*)"/dev/ttyS1";       //Default uart
    baudrate   = B115200;                   //Default baudrate

    outputBuffer     = NULL;                //Default value
    outputBufferSize = 0;                   //Default value

    //Start mutex (MUtual EXclusion)
    // -> for protection of shred memmory
    int result = pthread_mutex_init(&lock, NULL);   //no pthread_mutexattr (attribut)
    if (0 != result)
    {
        printf("\n mutex init failed\n");
    }
}

/**
 * @brief SerialPort::openSerial
 *      function for opening a serial port. Failure to open will result in
 *      error (TODO: Implement error handling in application code)
 */
void SerialPort::openSerial()
{
    //open serial port
    fd = _open_port(uart_name);
    if(-1 == fd)
    {
        printf("\nFaliure,could not open port!\n");
        printf("\nConnection attempt to port %s with %d baud failed,exiting!\n",
                uart_name,
                baudrate);
        return;
    }

    //setup port configuration
    if (!_setup_port(baudrate))
    {
        printf("\nFailure, could not configure port!\n");
    }


    //If everithing went well, we are now connceted
    printf("\nConnected to %s with %d baudrate, 8N1\n",
           uart_name,
           baudrate);
    status = SERIAL_PORT_OPEN;
    return;
}

void SerialPort::closeSerial()
{
    printf("\nClosing serial port %s\n",uart_name);
    int result = close(fd);                         //close returns 0 if OK, -1 if ERROR
    if (0 != result)
    {
        fprintf(stderr, "\nWARNING: Error on port close (%i)\n",result);
    }
    status = SERIAL_PORT_CLOSED;    //Error in closing isn't fatal so it's only Warning
    return;
}

/**
 * @brief SerialPort::_open_port
 *         Private function for opening a serial port with
 *         deffined setings for raw I/O without special
 *         characters. Decoding and encoding packets is
 *         done in higher lever functions.
 * @param port  uart port name i.e. /dev/ttySx
 * @return function returns file descriptor of given serial port
 */
int SerialPort::_open_port(const char* port)
{
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == fd)
    {
        //Couldn't open port
        return(-1);
    }
    else
    {
        fcntl(fd,F_SETFL, 0);
    }
    return fd;
}

/**
 * @brief SerialPort::_setup_port Function for serial port setup. It takes the
 * @param baud        - desired baudrate (115200 or 57600)
 * @return            - true if setup was successful
 *
 *  @todo: Zamijeni fprintf(stderr) funkcije sa Pixhawkovim error i warn
 *        metodama; uredi sucelje funkcije ili dodaj mogucnost postava
 *        pariteta, data_bits i stop_bits
 */

bool SerialPort::_setup_port(int baud)
{
    struct termios config;      //termios structure for port configuration
    //Get current serial port configuration
    if(tcgetattr(fd,&config)<0)
    {
        fprintf(stderr, "\nERROR: could not read config of fd %d\n",fd);
        return false;
    }

    //INPUT configuration for c_iflag (not used for setting raw input)
      //bit mask meaning for TRUE:
        //IGNBRK - ignore break condition
        //BRKINT - send SIGINT when beak condition detected
        //ICRNL  - Map CR to NL
        //INLCR  - Map NL to CR
        //PARMRK - Mark parity errors
        //INPCK  - Enable parity check
        //ISTRIP - Strip parity bits
        //IXON   - Enable software flow control outgoing

    //Setting RAW data input in local modes member c_lflag
      //bit mask meaning for TRUE:
        //ICANON  - Enable canonical input (else raw)
        //ECHO    - Enable echoing for input characters
        //ECHOE   - Echo erase character as BS-SP_BS
        //ISIG    - Enable SIGINTR, SIGSUSP, SIGDSUSP and
        //          SIGQUIT signals
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    //OUTPUT configuration for c_oflag
        //OPOST  - postproces output (not set = raw output)
    //When OPOST is reset all other flags in c_oflag are ignored
    config.c_oflag &= ~OPOST;

    //CONTROL configuration - set how read() behaves
        //VMIN = 0  - read will return wahewer is in the buffer (0 if empty); range [0,255]
        //VTIME = 0 - read will not block program execution, return immediately range [0, 2.25 sec]
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 0;

    //Set desired baudrate
    switch (baud)
    {
    case 115200:
        if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config,B115200)<0)
        {
            fprintf(stderr, "\nERROR: Could not set desired baud rate of %d baud\n",
                    baud);
            return false;
        }
        break;
    case 57600:
        if (cfsetispeed(&config, B57600)<0 || cfsetospeed(&config,B57600)<0)
        {
            fprintf(stderr, "\nERROR: Could not set desired baud rate of %d baud\n",
                    baud);
            return false;
        }
        break;
        //TODO: Replace fprintf with  warn function from pixhawks library
    }

    //Apply configuraton
    if (tcsetattr(fd,TCSANOW,&config)<0)
    {
        fprintf(stderr, "\nERROR: could not set configuration of fd %d\n",fd);
        return false;
    }

    //If nothing failed, setup has finished successfully
    return true;
}

/**
 * @brief SerialPort::_read_port function for safe reading from serial port
 *              without data courption. Data coruption is prevented using a
 *              pthread mutex
 * @param cp        - pointer to memory block to be filled with data from serial buffer
 * @param nbytes    - number of bytes to read. Function will read nbytes or less if
 *                    buffer contains less data of nbytes
 * @return          - number of bytes that were read
 */
int SerialPort::_read_port(char *cp, size_t nbytes)
{
    //Block uart ISR from filling the buffer while reading the data
    // with mutex lock
    pthread_mutex_lock(&lock);

    int result = read(fd, cp, nbytes);

    //Unlock mutex do buffer new data
    pthread_mutex_unlock(&lock);

    //Return the number of read bytes
    return result;
}

int SerialPort::_write_port(const uint8_t *buf, size_t len)
{
    //Lock the mutex
    pthread_mutex_lock(&lock);

    //Write packet via serial link
    const int bytesWritten = static_cast<int>(write(fd,buf,len));
    if(bytesWritten != len)
    {
        fprintf(stderr,"\nWARNING: Could not write %d bytes. Written only %d bytes\n",
                len, bytesWritten);
    }

    //Unlock mutex
    pthread_mutex_unlock(&lock);
    return bytesWritten;
}

int SerialPort::writeMessage(uint8_t *message, size_t len)
{
    int bytesWritten = _write_port(message,len);
    return bytesWritten;
}
int SerialPort::readMessage(char *message, size_t nbytes)
{
    return _read_port(message,nbytes);
}

void SerialPort::_serialize_motor_control_t(motor_control_t *msg)
{
    //This function is used to copy message to output buffer.
    union{
        motor_control_t msg_s;
        char data_s[20];
    } msg_u;

    msg_u.msg_s = *msg;

    memcpy(outputBuffer, msg_u.data_s, sizeof(motor_control_t));
}

bool SerialPort::sendMotorControl(motor_control_t *msg)
{
    _serialize_motor_control_t(msg);
    size_t bytesToWrite = sizeof(motor_control_t);
    size_t bytesWritten = writeMessage(outputBuffer,bytesToWrite);
    //NOTE: data is writen to serial line in inversed order
    //i.e. last byte comes first because of LITTLE ENDIAN. Take that in to account on the other side!!!
    if (bytesWritten == bytesToWrite)
    {
        //fprintf(stderr,"\nINFO: Successfully written %d bytes to serial port\n ",
        //            bytesWritten);
        return true;
    }
    else
    {
        fprintf(stderr, "\nWARNING: Couldn't write %d bytes, written only %d bytes\n",
                        bytesToWrite,bytesWritten);
        return false;
    }
}


