/*
 * nonoSerial is based on
 * wiringSerial.h:
 *	Handle a serial port
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */


#ifndef _GST_NONOSERIAL_H_
#define _GST_NONOSERIAL_H_

#include <gst/gst.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <math.h>

#include <errno.h>
#include <sys/ioctl.h>
#include <stdbool.h>

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif

#ifdef __linux__
#include <linux/serial.h>
#endif

		// public:
			// Serial();
			// int   nono_serial_openPort      (const char *device, const int baud) ;
			// void  nono_serial_closePort     ( int fd ) ;

			// int nono_serial_setBaudrate( speed_t baudrate );
			// int nono_serial_setCustomBaudrate( speed_t baudrate );
static int   	nono_serial_openPort (const char *device, const int baud);
static void  	nono_serial_closePort ( int fd );
static int 		nono_serial_setBaudrate( int fd, speed_t baudrate );
static int 		nono_serial_setCustomBaudrate( int fd, speed_t baudrate );
// static void 	nono_serial_writeByte ( int fd, const unsigned char c );
// static int 		nono_serial_writeString ( int fd, const char *s );
static int 		nono_serial_writeBytes ( int fd, const unsigned char* data, int len );
		

/*
 * serialOpen:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required - hopefully!
 *********************************************************************************
 */

static int nono_serial_openPort (const char *device, const int baud)
{

  struct termios oldoptions;
  struct termios options ;
  speed_t myBaud = -1;
  int     status;
  int fd = -1;

  switch (baud)
  {
    case     50:	myBaud =     B50 ; break ;
    case     75:	myBaud =     B75 ; break ;
    case    110:	myBaud =    B110 ; break ;
    case    134:	myBaud =    B134 ; break ;
    case    150:	myBaud =    B150 ; break ;
    case    200:	myBaud =    B200 ; break ;
    case    300:	myBaud =    B300 ; break ;
    case    600:	myBaud =    B600 ; break ;
    case   1200:	myBaud =   B1200 ; break ;
    case   1800:	myBaud =   B1800 ; break ;
    case   2400:	myBaud =   B2400 ; break ;
    case   4800:	myBaud =   B4800 ; break ;
    case   9600:	myBaud =   B9600 ; break ;
    case  19200:	myBaud =  B19200 ; break ;
    case  38400:	myBaud =  B38400 ; break ;
    case  57600:	myBaud =  B57600 ; break ;
    case 115200:	myBaud = B115200 ; break ;
    case 230400:	myBaud = B230400 ; break ;
#ifdef __linux__    
    default:      myBaud = baud ; break;
    // case 1000000:  myBaud = B1000000 ; break ;
#endif

  }
  g_print("all shit");
  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1){
    g_print("ERROR Opening SerialPort ---- %s at %i\n",device, baud);
    return -1 ;
  }
  // fcntl (fd, F_SETFL, O_RDWR) ;


    // Get the current options and save them so we can restore the default settings later.
  if (tcgetattr(fd, &oldoptions) == -1) {
        g_print("Error getting tty attributes %s(%d).\n", strerror(errno), errno);
  }

// Get and modify current options:
  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW | TCSAFLUSH, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  // int currentBaudbaudRate = (int) cfgetospeed(&options);
  // printf("Output baud rate changed to %d\n", (int) cfgetospeed(&options));
  
  // On OS X, starting in Tiger, we can set a custom baud rate, as follows:

  if ((int) cfgetospeed(&options) != baud) {

    int res = nono_serial_setCustomBaudrate( fd, baud );
    if( res == 0 ){
      myBaud = baud;
      g_print("SUCCESS: Altered baudrate to %i.\n",baud);
    }else{
      g_print("ERROR: Couldn't set baudrate to custom value %i.\n",baud);
    }

  }

#ifdef __APPLE__
  unsigned long mics = 1UL;
  int success = ioctl(fd, IOSSDATALAT, &mics);
  if( success < 0 ){
    g_print("Error Serial setting read latency %s - %s(%d).\n", device, strerror(errno), errno);
  }
#endif

  if( fd == -1 ) g_print("ERROR Connecting to Serial Device: %s.\n",device);
  else g_print("Connected to Serial Device: %s at %i baud\n",device,(int)myBaud);

  usleep (10000) ;	// 10mS

  return fd ;
}



/*
 * serialClose:
 *	Release the serial port
 *********************************************************************************
 */

static void nono_serial_closePort ( int fd )
{
  
    // Block until all written output has been sent from the device.
    // Note that this call is simply passed on to the serial device driver.
    // See tcsendbreak(3) <x-man-page://3/tcsendbreak> for details.

  if (tcdrain(fd) == -1) {
        g_print("Error waiting for serial drain.\n");
  }

    // Block until all written output has been sent from the device.
    // Note that this call is simply passed on to the serial device driver.
    // See tcsendbreak(3) <x-man-page://3/tcsendbreak> for details.

  // if (tcsetattr(fd, TCSANOW, &oldoptions) == -1) {
  //   printf("Error resetting tty attributes - %s(%d).\n",
  //   strerror(errno), errno);

  // }
  close (fd) ;
}




static int nono_serial_setBaudrate( int fd, speed_t baudrate ) {
  int rc;
  struct termios tio;

  rc = tcgetattr(fd, &tio);
  if (rc < 0) {
    g_print("ERROR: tcgetattr()\n");
    return -errno;
  }
  cfsetispeed(&tio, baudrate);
  cfsetospeed(&tio, baudrate);

  rc = tcsetattr(fd, TCSANOW, &tio);
  if (rc < 0) {
    g_print("ERROR: tcgetattr()\n");
    return -errno;
  }

  return 0;
}


// This part is taken from here: http://cgit.osmocom.org/osmocom-bb/plain/src/shared/libosmocore/src/serial.c

/*! \brief Change current baudrate to a custom one using OS specific method
 *  \param[in] fd File descriptor of the open device
 *  \param[in] baudrate Baudrate as integer
 *  \returns 0 for success or negative errno.
 *
 *  This function might not work on all OS or with all type of serial adapters
 */

static int nono_serial_setCustomBaudrate( int fd, speed_t baudrate ) {
#ifdef __linux__
  int rc;
  struct serial_struct ser_info;

  rc = ioctl(fd, TIOCGSERIAL, &ser_info);
  if (rc < 0) {
    g_print("ERROR: ioctl(TIOCGSERIAL)\n");
    return -errno;
  }

  ser_info.flags = ASYNC_SPD_CUST | ASYNC_LOW_LATENCY;
  ser_info.custom_divisor = ser_info.baud_base / baudrate;

  rc = ioctl(fd, TIOCSSERIAL, &ser_info);
  if (rc < 0) {
    g_print("ERROR: ioctl(TIOCSSERIAL)\n");
    return -errno;
  }

  return nono_serial_setBaudrate( fd, B38400 ); /* 38400 is a kind of magic ... */

#elif defined(__APPLE__)

#ifndef IOSSIOSPEED
#define IOSSIOSPEED    _IOW('T', 2, speed_t)
#endif

  int rc;
  unsigned int speed = baudrate;
  rc = ioctl(fd, IOSSIOSPEED, &speed);
  if (rc < 0) {
    printf("ERROR: ioctl(IOSSIOSPEED)\n");
    return -errno;
  }
  return 0;
#else

#warning unsupported platform
  return -1;
#endif
}



/*
 * serialPutchar:
 *	Send a single character to the serial port
 *********************************************************************************
 */

// static void nono_serial_writeByte ( int fd, const unsigned char c)
// {
//   if( fd == -1 ) return;
//   write (fd, &c, 1) ;
// }


// /*
//  * serialPuts:
//  *	Send a string to the serial port
//  *********************************************************************************
//  */

// static int nono_serial_writeString ( int fd, const char *s )
// {
//   // if( fd == -1 ) return -1;
//   return write (fd, s, strlen (s)) ;
// }

/*
 * writeBytes:
 *  Send an array of bytes to the serial port
 *********************************************************************************
 */

static int nono_serial_writeBytes ( int fd, const unsigned char* data, int len )
{
  return write (fd, data, len) ;
}



/*
static bool nono_serial_testSendToSerial( int fd, unsigned char* dataBuffer, int len, int val ){
  // Serial* serial = &serialDevices[0];
  // printf("-------------- TEST DATA FOR SERIAL (%s) -------------- ", serial->getDeviceName().c_str());
  int l = len;
  float ledVal;
  // int val;
  ledVal = (float)((sin(val/100.0f)+1)/2.0);
  val = (int)(ledVal * 230) & 0xff;
  memset(dataBuffer,val,l);
  dataBuffer[0] = 0xff;
  dataBuffer[1] = 0x00;
  // g_print("xFRAME #%i => %f\n",val, ledVal );
  // float divider = ((sin(val/10.0f)+1)/2.0) * 30.0 + 20.0;
  // divider = 60;//val/1000.0f;
  // float scale = 5000.0f;
  // float x,y;
  // float z;
  // z = divider;
  // z = val/100.0;
  // z = sin(val/10000.0f) * 100.0f;
  // printf("%i    %f",val, z);

  ledVal = (int)(val/5)%2;
  ledVal = (float)sin(val/10000.0f);

  // for( int i=2;i<l;i++ ){
  //   x = (i-2)%9;
  //   y = (int)((i-2)/9);
  //   // ledVal = noise->noise(scale * x, scale * y, z);
  //   // dataBuffer[i] = (int)(ledVal * 230) & 0xff;
  // }
 
  int lcdsPerPanel = 27;
  // int offset = 2;

  int panel = (val/30) % 20;
  int start = lcdsPerPanel*panel;
  int end = lcdsPerPanel*(panel+1);
  // end = start + 10;
  for( int i=2 + start;i<2+end;i++ ){
    // x = (i-2)%9;
    // y = (int)((i-2)/9);
    // ledVal = noise->noise(scale * x, scale * y, z);
    // dataBuffer[i] = (int)(ledVal * 230) & 0xff;
  }


  int lcdPos = start;//lcdsPerPanel*panel;

  // memset(dataBuffer,0,l);
  ledVal = (int)(val/10)%2;
  dataBuffer[lcdPos+2] = (int)(ledVal * 230) & 0xff;

  // printf(" LED VAL : %3i     Panel #%i   [ %i -> %i ]    LCD_Single #%i\n",dataBuffer[lcdPos+2], panel+1, start, end, lcdPos );

  bool res = true;

  if( !nono_serial_writeBytes( fd, dataBuffer, l  ) ){
    res = false;
  }

  return res;
}
*/
#endif