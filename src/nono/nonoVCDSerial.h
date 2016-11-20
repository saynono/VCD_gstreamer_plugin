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


#ifndef _GST_NONOVCDSERIAL_H_
#define _GST_NONOVCDSERIAL_H_



#include "nonoSerial.h"
#include "VCD_Defs.h"

static void nono_vcd_serial_init();

static void nono_vcd_serial_sendFrame( int fd, uint8* dataBuffer, int dataBufferSize, uint8* serialBuffer, int serialBufferSize );
static void nono_vcd_serial_sendDataToSection( int fd, int splitterID, uint8* data, int lenDataSection, uint8* serialBuffer, int serialBufferSize );
static bool nono_vcd_serial_sendData( int fd, uint8* serialBuffer, int serialBufferSize );

static void nono_vcd_serial_init(){
    
}


static void nono_vcd_serial_sendFrame( int fd, uint8* dataBuffer, int dataBufferSize, uint8* serialBuffer, int serialBufferSize ){

  int numSections = dataBufferSize/(NUM_ELEMENTS_PER_STRAND * NUM_STRANDS_PER_SECTIONS);
  int lenDataSection = NUM_ELEMENTS_PER_STRAND * NUM_STRANDS_PER_SECTIONS;
            
  for( int i=0;i<numSections;i++ ){
    nono_vcd_serial_sendDataToSection( fd, (i+1), dataBuffer+(lenDataSection*i), lenDataSection, serialBuffer, serialBufferSize);
  }

}
        
        
static void nono_vcd_serial_sendDataToSection( int fd, int splitterID, uint8* dataBuffer, int lenDataSection, uint8* serialBuffer, int serialBufferSize ){

//    printf("........\n");
//    printHex(data,20);
    //  HEADER
    //    0xC5 <lenlo> <lenhi> <id> <command> [data]
    int lenHeader = 5;
    int lenFrameHeader = 5;
    int bytesPerPort = NUM_ELEMENTS_PER_STRAND;//0x0B;
    int lenData = (bytesPerPort+1)*NUM_STRANDS_PER_SECTIONS + lenFrameHeader;
//    int lenTot = lenData + lenHeader + lenFrameHeader;
    
    serialBuffer[0] = 0xC5;                   // General Header
    serialBuffer[1] = 0xff & lenData;         // length low | starting after <command>
    serialBuffer[2] = 0x00;                   // length hi  | starting after <command>
    serialBuffer[3] = splitterID;             // ID of Splitter | 0 for all
    
    //  COMMAND SEND
    //    01 <nports> <nbyteslo> <nbyteshi> <mask> 00 [00 <d0..n>] [00 <d0..n>] .... [00 <d0..n>]
    
    serialBuffer[4] = 0x01;                     // Command      | 01 for sending data
    serialBuffer[5] = 0x0A;                     // Port of Splitter | 00 for sending same to all
    serialBuffer[6] = bytesPerPort+1;             // bytes low    | Data sending per port
    serialBuffer[7] = 0x00;                     // bytes hi     | Data sending per port
    serialBuffer[8] = 0x00;                     // mask         | ports mask
    serialBuffer[9] = 0x00;                     // format       | This should be 0 for the format used by the LCD

    
    if( NUM_ELEMENTS_PER_STRAND * NUM_STRANDS_PER_SECTIONS != lenDataSection ){
        printf(" Data Section Length wrong, is %i should be %i \n",lenDataSection,NUM_ELEMENTS_PER_STRAND * NUM_STRANDS_PER_SECTIONS);
    }

    memset( serialBuffer+(lenFrameHeader+lenHeader), 0, lenData);
    int pos = 10;
    for( int i=0;i<lenDataSection;i++ ){
      if(i%bytesPerPort==0) serialBuffer[pos++] = 0x00;
      serialBuffer[pos++] = dataBuffer[i];
    }
    nono_vcd_serial_sendData( fd, serialBuffer, pos );
    
//    if( splitterID == 1 ){
//        printHex(serialBuffer, pos);
//    }
}
        
static bool nono_vcd_serial_sendData( int fd, uint8* serialBuffer, int serialBufferSize ){
  bool res = true;
  if( !nono_serial_writeBytes( fd, serialBuffer, serialBufferSize  ) ){
    res = false;
  }
  return res;
}



#endif // _GST_NONOVCDSERIAL_H_