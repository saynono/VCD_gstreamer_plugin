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


#ifndef _GST_NONOVCDTESTFRAMES_H_
#define _GST_NONOVCDTESTFRAMES_H_

#include <gst/gst.h>
#include "VCD_Defs.h"
#include <time.h>

typedef enum {
  TEST_SECTION = 0,
  TEST_STRAND,
  TEST_LCD,
  TEST_ANIMATION1
} TEST_PATTERN;


//check for division by zero???
//--------------------------------------------------
static float ofMap(float value, float inputMin, float inputMax, float outputMin, float outputMax ) {

  if (fabs(inputMin - inputMax) < FLT_EPSILON){
    return outputMin;
  } else {
    float outVal = ((value - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin);
    return outVal;
  }

}

static int ofGetElapsedTimeMillis(){
  return ((float)clock() / CLOCKS_PER_SEC ) * 1000; 
}


static uint8 nono_vcd_getBlinkValue(){
    return (ofGetElapsedTimeMillis() % 1000) > 500 ? 0xff : 0x00;
}

static uint8 nono_vcd_getFadeValue(){
    return (uint8)( ofMap( (float)sin(ofGetElapsedTimeMillis()/300.0f),-1.f, 1.f , 10, 40) * 0xff);
}





static void nono_vcd_testSection( int sectionID, GstVideoFrame * frame ){
    // Test data

  guint8 *data;
  gint height;
  gint row_stride;
  gint len;

  int c = ofGetElapsedTimeMillis();
  g_print("Clock : %i\n", c);

  data = GST_VIDEO_FRAME_PLANE_DATA (frame, 0);

  // width = GST_VIDEO_FRAME_WIDTH (frame);
  height = GST_VIDEO_FRAME_HEIGHT (frame);

  row_stride = GST_VIDEO_FRAME_PLANE_STRIDE (frame, 0);
  // pixel_stride = GST_VIDEO_FRAME_COMP_PSTRIDE (frame, 0);
  len = row_stride * height;  

    memset(data,0,len);
    uint8 val = 0x00;
    uint8 blinkVal = nono_vcd_getBlinkValue();
    uint8 fadeVal = nono_vcd_getFadeValue();
    int lenDataSection = NUM_ELEMENTS_PER_STRAND * NUM_STRANDS_PER_SECTIONS;
    for( int i=0;i<NUM_SECTIONS;i++ ){
        val = (i == sectionID-1) ? blinkVal : fadeVal;
        memset(data+(i*lenDataSection), val, lenDataSection);
    }
//    sendFrame( dataBufferTest, NUM_ELEMENTS_TOTAL );
    
    //    memset(dataBufferTest, blinkVal, lenDataSection);
    //    sendDataToSection( sectionID, dataBufferTest, lenDataSection);
}

static void nono_vcd_createPattern( TEST_PATTERN pId, int someNum, GstVideoFrame * frame ){

    switch(pId){
        case TEST_SECTION:
            nono_vcd_testSection( someNum, frame );
            break;
        case TEST_STRAND:
            // nono_vcd_testStrand( someNum, data, len);
            break;
        case TEST_LCD:
            // nono_vcd_testLCD( someNum, data, len);
            break;
        case TEST_ANIMATION1:
            // nono_vcd_sendTestFrame(data, len);
            break;
    }
}


#endif // _GST_NONOVCDTESTFRAMES_H_