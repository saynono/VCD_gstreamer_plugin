//
//  VCD_Defs.h
//  VCD_TestController_test
//
//  Created by say nono on 20.07.16.
//  Copyright (c) 2016 __MyCompanyName__. All rights reserved.
//

#pragma once

#ifndef _UINT8_T
#define _UINT8_T
typedef unsigned char uint8_t;
#endif /* _UINT8_T */

#ifndef _UINT8
typedef uint8_t uint8;
#define _UINT8
#endif


#define NUM_SECTIONS                    43
#define NUM_STRANDS_PER_SECTIONS        10
#define NUM_ELEMENTS_PER_STRAND         5
#define NUM_ELEMENTS_TOTAL              NUM_SECTIONS * NUM_STRANDS_PER_SECTIONS * NUM_ELEMENTS_PER_STRAND


#define HEADER_NEW_FRAME                0xf0
#define HEADER_NEW_FRAME_PART1          0xf1
#define HEADER_NEW_FRAME_PART2          0xf2
#define HEADER_NEW_FRAME_PART3          0xf3
#define FRAME_PART_SIZE                 1000

#define HEADER_START_RECORDING          0xfA
#define HEADER_STOP_RECORDING           0xfB
#define UDP_PORT                        12993
#define UDP_BUFFER_SIZE                 4096

#define SERIAL_BAUDRATE                 2000000


#define DATA_BUFFER_SERIAL_IN_SIZE      1000
#define DATA_BUFFER_SERIAL_OUT_SIZE     1000
