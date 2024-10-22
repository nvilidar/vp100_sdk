/*
 * @Version      : V1.0
 * @Date         : 2024-10-12 17:35:37
 * @Description  : serial api unix 
 */
#include "interface/serial/interface_serial.hpp"
#include <unistd.h>
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>
#include <err.h>
#include <linux/serial.h>
#include <cstring>
#include <sys/ioctl.h> //ioctl

namespace nvistar{
class InterfaceSerialImpl{
public:
  //var 
  int  _fd = -1;                    //File descriptor for the port
  bool open_flag = false;          //serial open flag

  // linux/include/uapi/asm-generic/termbits.h
  struct termios2 {
      tcflag_t c_iflag;       /* input mode flags */
      tcflag_t c_oflag;       /* output mode flags */
      tcflag_t c_cflag;       /* control mode flags */
      tcflag_t c_lflag;       /* local mode flags */
      cc_t c_line;            /* line discipline */
      cc_t c_cc[19];          /* control characters */
      speed_t c_ispeed;       /* input speed */
      speed_t c_ospeed;       /* output speed */
  };
  #ifndef TCGETS2
  #define TCGETS2     _IOR('T', 0x2A, struct termios2)
  #endif

  #ifndef TCSETS2
  #define TCSETS2     _IOW('T', 0x2B, struct termios2)
  #endif

  #ifndef BOTHER
  #define BOTHER      0010000
  #endif

  //function 

  /**
  * @Function: serial_set_databits
  * @Description: set databits 
  * @Return: bool
  * @param {termios} *tio
  * @param {serial_databits_t} databits
  */
  bool serial_set_databits(struct termios *tio,InterfaceSerial::serial_databits_t databits){
    tio->c_cflag &= (tcflag_t)~CSIZE;
    //set databits 
    switch (databits) {
      case InterfaceSerial::DataBits5:{
        tio->c_cflag |= CS5;
        break;
      }
      case InterfaceSerial::DataBits6:{
        tio->c_cflag |= CS6;
        break;
      }
      case InterfaceSerial::DataBits7:{
        tio->c_cflag |= CS7;
        break;
      }
      case InterfaceSerial::DataBits8:{
        tio->c_cflag |= CS8;
        break;
      }
      default:{
        tio->c_cflag |= CS8;
        break;
      }
    }
    return true;
  }
  /**
  * @Function: serial_set_parity
  * @Description: set parity 
  * @Return: bool 
  * @param {termios} *tio
  * @param {serial_parity_t} parity
  */
  bool serial_set_parity(struct termios *tio,InterfaceSerial::serial_parity_t parity){
    tio->c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);

    switch (parity) {
    #ifdef CMSPAR
      // Here Installation parity only for GNU/Linux where the macro CMSPAR.
      case InterfaceSerial::ParitySpace:{
        tio->c_cflag |=  (PARENB | CMSPAR);
        tio->c_cflag &= (tcflag_t) ~(PARODD);
        break;
      }
      case InterfaceSerial::ParityMark:{
        tio->c_cflag |= (PARENB | CMSPAR | PARODD);
        break;
      }
    #endif
      case InterfaceSerial::ParityNone:{
        tio->c_cflag &= (tcflag_t) ~(PARENB | PARODD);
        break;
      }
      case InterfaceSerial::ParityEven:{
        tio->c_cflag &= (tcflag_t) ~(PARODD);
        tio->c_cflag |= (PARENB);
        break;
      }
      case InterfaceSerial::ParityOdd:{
        tio->c_cflag |= (PARENB | PARODD);
        break;
      }
      default:{
        tio->c_cflag |=  (PARENB | CMSPAR);
        tio->c_cflag &= (tcflag_t) ~(PARODD);
        break;
      }
    }
    return true;
  }
  /**
  * @Function: serial_set_stopbits
  * @Description: set stopbit 
  * @Return: bool 
  * @param {termios} *tio
  * @param {serial_stopbits_t} stop_bits
  */
  bool serial_set_stopbits(struct termios *tio, InterfaceSerial::serial_stopbits_t stopbits){
    switch (stopbits) {
      case InterfaceSerial::StopOne:{
        tio->c_cflag &= (tcflag_t)~CSTOPB;
        break;
      }
      case InterfaceSerial::StopTwo:{
        tio->c_cflag |= (tcflag_t)CSTOPB;
        break;
      }
      default:{
        tio->c_cflag &= (tcflag_t)~CSTOPB;
        break;
      }
    }
    return true;
  }
  /**
  * @Function: serial_set_flowcontrol
  * @Description: set flowcontrol 
  * @Return: bool 
  * @param {termios} *tio
  * @param {serial_flowcontrol_t} flowcontrol
  */
  bool serial_set_flowcontrol(struct termios *tio, InterfaceSerial::serial_flowcontrol_t flowcontrol){
    switch (flowcontrol) {
      case InterfaceSerial::FlowNone:{
        tio->c_cflag &= ~CRTSCTS;
        tio->c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
      }
      case InterfaceSerial::FlowHardware:{
        tio->c_cflag |= CRTSCTS;
        tio->c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
      }
      case InterfaceSerial::FlowSoftware:{
        tio->c_cflag &= ~CRTSCTS;
        tio->c_iflag |= IXON | IXOFF | IXANY;
        break;
      }
      default:{
        tio->c_cflag &= ~CRTSCTS;
        tio->c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
      }
    } 
    return true;
  }
  /**
  * @Function: serial_baudrate_to_unix
  * @Description: baudrate to unix 
  * @Return: int
  * @param {int} baudrate
  */
  bool serial_baudrate_to_unix(int baudrate,int &unix_baud){
    bool status = true;
    switch (baudrate) {
      case 50:{
        unix_baud = B50;
        break;
      }
      case 75:{
        unix_baud = B75;
        break;
      }
      case 110:{
        unix_baud = B110;
        break;
      }
      case 134:{
        unix_baud = B134;
        break;
      }
      case 150:{
        unix_baud = B150;
        break;
      }
      case 200:{
        unix_baud = B200;
        break;
      }
      case 300:{
        unix_baud = B300;
        break;
      }
      case 600:{
        unix_baud = B600;
        break;
      }
      case 1200:{
        unix_baud = B1200;
        break;
      }
      case 1800:{
        unix_baud = B1800;
        break;
      }
      case 2400:{
        unix_baud = B2400;
        break;
      }
      case 4800:{
        unix_baud = B4800;
        break;
      }
      case 9600:{
        unix_baud = B9600;
        break;
      }
      case 19200:{
        unix_baud = B19200;
        break;
      }
      case 38400:{
        unix_baud = B38400;
        break;
      }
      case 57600:{
        unix_baud = B57600;
        break;
      }
      case 115200:{
        unix_baud = B115200;
        break;
      }
      case 230400:{
        unix_baud = B230400;
        break;
      }
      case 460800:{
        unix_baud = B460800;
        break;
      }
      case 500000:{
        unix_baud = B500000;
        break;
      }
      case 576000:{
        unix_baud = B576000;
        break;
      }
      case 921600:{
        unix_baud = B921600;
        break;
      }
      case 1000000:{
        unix_baud = B1000000;
        break;
      }
      case 1152000:{
        unix_baud = B1152000;
        break;
      }
      case 1500000:{
        unix_baud = B1500000;
        break;
      }
      case 2000000:{
        unix_baud = B2000000;
        break;
      }
      case 2500000:{
        unix_baud = B2500000;
        break;
      }
      case 3000000:{
        unix_baud = B3000000;
        break;
      }
      case 3500000:{
        unix_baud = B3500000;
        break;
      }
      case 4000000:{
        unix_baud = B4000000;
        break;
      }
      default:{
        status = false;
        break;
      }
    }
    return status;
  }
  /**
  * @Function: serial_set_baudrate
  * @Description: set serial baudrate 
  * @Return: bool 
  * @param {int} baudrate
  */
  bool serial_set_baudrate(int fd,struct termios *tio, int baudrate){
    int baud_unix;
    struct termios2 tio2;
    //standard baudrate
    if(serial_baudrate_to_unix(baudrate,baud_unix)){
      //get tio2 to clear custom baud
      if(ioctl(fd, TCGETS2, &tio2)){
        return false;
      }
      if(tio2.c_cflag & BOTHER){
        tio2.c_cflag |= CBAUD;
        tio2.c_cflag &= ~BOTHER;
        //set tio 2
        if(ioctl(fd, TCSETS2, &tio2)){
          return false;
        }
      }
      //set standard baudrate 
      if(cfsetispeed(tio, baud_unix) < 0){
        return false;
      }
      if(cfsetospeed(tio, baud_unix) < 0){
        return false;
      }
    }
    //custom baudrate 
    else{
      //get tio2 
      if(ioctl(fd, TCGETS2, &tio2)){
        return false;
      }
      tio2.c_cflag &= ~CBAUD;
      tio2.c_cflag |= BOTHER;
      tio2.c_ispeed = baudrate;
      tio2.c_ospeed = baudrate;
      //set & get tio 2
      if(ioctl(fd, TCSETS2, &tio2)){
        return false;
      }
      if(ioctl(fd, TCGETS2, &tio2)){
        return false;
      }
    }
    return true;
  }
};

/**
 * @Function: InterfaceSerial
 * @Description: 
 * @Return: None
 */
InterfaceSerial::InterfaceSerial() : _impl(new InterfaceSerialImpl){

}

/**
 * @Function: ~InterfaceSerial
 * @Description: 
 * @Return: None
 */
InterfaceSerial::~InterfaceSerial(){
  serial_close();
  delete _impl;
}

/**
 * @Function: serial_open
 * @Description: open serial
 * @Return: bool --- success or fail
 * @param {string} port_name 
 * @param {int} baud_rate
 * @param {int} party
 * @param {int} data_bits
 * @param {int} stop_bits
 * @param {int} flow_control
 */
bool InterfaceSerial::serial_open(std::string port_name, int baud_rate,
                  serial_parity_t parity,
                  serial_databits_t databits,
                  serial_stopbits_t stopbits,
                  serial_flowcontrol_t flow_control){
  //open the serial 
  _impl->_fd = open(port_name.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);  //no block 
  //open failed
  if(_impl->_fd == -1){
    return false;
  }
  struct termios tio;
  //termios get info
  if (tcgetattr(_impl->_fd, &tio) == -1) {
    return false;
  }
  //set up raw mode / no echo / binary
  tio.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
  tio.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                       ISIG | IEXTEN); //|ECHOPRT
  tio.c_oflag &= (tcflag_t) ~(OPOST);
  tio.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
  tio.c_iflag &= (tcflag_t) ~IUCLC;
#endif
#ifdef PARMRK
  tio.c_iflag &= (tcflag_t) ~PARMRK;
#endif
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  //set parameter 
  _impl->serial_set_parity(&tio, parity);            //set parity 
  _impl->serial_set_databits(&tio, databits);        //set databits 
  _impl->serial_set_stopbits(&tio, stopbits);        //set stopbits 
  _impl->serial_set_flowcontrol(&tio, flow_control); //set flowcontrol
  //set baudrate
  if(false == _impl->serial_set_baudrate(_impl->_fd, &tio, baud_rate)){         //set baudrate
    return false;
  }  
   //set flag 
  if (tcsetattr(_impl->_fd, TCSANOW, &tio) == -1) {
    return false;
  }             
  return true;
}

/**
 * @Function: serial_close
 * @Description: close serialport
 * @Return: void
 */
void InterfaceSerial::serial_close(){
  if(!serial_isopen()){
    return;
  }
  //close the serial 
  close(_impl->_fd);
  _impl->_fd = -1;
}

/**
 * @Function: serial_isopen
 * @Description: judge serial is open?
 * @Return: bool 
 */
bool InterfaceSerial::serial_isopen(){
  if(_impl->_fd != -1){
    return true;
  }
  return false;
}

/**
 * @Function: serial_read
 * @Description: serial read data 
 * @Return: int --- read true length
 * @param {uint8_t} *data
 * @param {int} max_length
 */
int InterfaceSerial::serial_read(uint8_t *data,int max_length){
  if(!serial_isopen()){
    return 0;
  }
  return read(_impl->_fd, data, max_length);
}

/**
 * @Function: serial_write
 * @Description: serial write data 
 * @Return: int --- write true length 
 * @param {uint8_t*} data
 * @param {int} length
 */
int InterfaceSerial::serial_write(const uint8_t* data,int length){
  if(!serial_isopen()){
    return 0;
  }
  return write(_impl->_fd, data, length);
}

/**
 * @Function: serial_flush
 * @Description: flush serial input and output
 * @Return: void
 */
void InterfaceSerial::serial_flush(){
  if(!serial_isopen()){
    return;
  }
  tcflush(_impl->_fd, TCIFLUSH);
  tcflush(_impl->_fd, TCOFLUSH);
}
}
