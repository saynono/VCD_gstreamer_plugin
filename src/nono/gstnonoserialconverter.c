/* GStreamer
 * Copyright (C) 2016 Vincent Rebers <say.nono@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Suite 500,
 * Boston, MA 02110-1335, USA.
 */
/**
 * SECTION:element-gstnonoserialconverter
 *
 * The nonoserialconverter element does some stuff.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 videotestsrc ! video/x-raw,width=320,heigh=240 ! nonoserialconverter ! autovideosink
 * ]|
 * Takes video content and sends it to a serial port.
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>

#include "gstnonoserialconverter.h"
#include "nonoSerial.h"
#include "nonoVCDSerial.h"
// #include "nonoVCDTestFrames.h"
#include "VCD_Defs.h"


#ifndef max
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#endif

#ifndef min
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#endif



GST_DEBUG_CATEGORY_STATIC (gst_nonoserialconverter_debug_category);
#define GST_CAT_DEFAULT gst_nonoserialconverter_debug_category

/* prototypes */


static void gst_nonoserialconverter_set_property (GObject * object,
    guint property_id, const GValue * value, GParamSpec * pspec);
static void gst_nonoserialconverter_get_property (GObject * object,
    guint property_id, GValue * value, GParamSpec * pspec);
static void gst_nonoserialconverter_dispose (GObject * object);
static void gst_nonoserialconverter_finalize (GObject * object);

static gboolean gst_nonoserialconverter_start (GstBaseTransform * trans);
static gboolean gst_nonoserialconverter_stop (GstBaseTransform * trans);
static gboolean gst_nonoserialconverter_set_info (GstVideoFilter * filter, GstCaps * incaps,
    GstVideoInfo * in_info, GstCaps * outcaps, GstVideoInfo * out_info);
// static GstFlowReturn gst_nonoserialconverter_transform_frame (GstVideoFilter * filter,
//     GstVideoFrame * inframe, GstVideoFrame * outframe);
static void gst_nono_prepare_data (GstVideoFilter * filter, GstVideoFrame * frame, GstNonoSerialConverter *nonoserialconverter );
static void gst_nono_temp_transform (GstVideoFilter * filter, GstVideoFrame * frame, GstNonoSerialConverter *nonoserialconverter );
static GstFlowReturn gst_nonoserialconverter_transform_frame_ip (GstVideoFilter * filter,
    GstVideoFrame * frame);

enum
{
  PROP_0
};

/* pad templates */

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS_ANY);

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS_ANY);

/* FIXME: add/remove formats you can handle */
#define VIDEO_SRC_CAPS \
    GST_VIDEO_CAPS_MAKE("{ RGBA }")

/* FIXME: add/remove formats you can handle */
#define VIDEO_SINK_CAPS \
    GST_VIDEO_CAPS_MAKE("{ RGBA }")


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (GstNonoSerialConverter, gst_nonoserialconverter, GST_TYPE_VIDEO_FILTER,
  GST_DEBUG_CATEGORY_INIT (gst_nonoserialconverter_debug_category, "nonoserialconverter", 0,
  "debug category for nonoserialconverter element"));

static void
gst_nonoserialconverter_class_init (GstNonoSerialConverterClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstBaseTransformClass *base_transform_class = GST_BASE_TRANSFORM_CLASS (klass);
  GstVideoFilterClass *video_filter_class = GST_VIDEO_FILTER_CLASS (klass);
  GstElementClass *gstelement_class = (GstElementClass *) klass;

  video_filter_class->set_info = GST_DEBUG_FUNCPTR (gst_nonoserialconverter_set_info);
  video_filter_class->transform_frame_ip = GST_DEBUG_FUNCPTR (gst_nonoserialconverter_transform_frame_ip);

  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS(klass),
      "Nono Video To Serial Converter", "Generic", "Taking video data and outputing Serial data.",
      "Vincent <say.nono@gmail.com>");

  gobject_class->set_property = gst_nonoserialconverter_set_property;
  gobject_class->get_property = gst_nonoserialconverter_get_property;

  gobject_class->dispose = gst_nonoserialconverter_dispose;
  gobject_class->finalize = gst_nonoserialconverter_finalize;
  base_transform_class->start = GST_DEBUG_FUNCPTR (gst_nonoserialconverter_start);
  base_transform_class->stop = GST_DEBUG_FUNCPTR (gst_nonoserialconverter_stop);

  gst_element_class_add_pad_template (gstelement_class,
      gst_static_pad_template_get (&src_template));
  gst_element_class_add_pad_template (gstelement_class,
      gst_static_pad_template_get (&sink_template));

}

static void
gst_nonoserialconverter_init (GstNonoSerialConverter *nonoserialconverter)
{
  const char* device = "/dev/ttyUSB3";
  nonoserialconverter->frameNum = 0;
  nonoserialconverter->dataBufferSize = NUM_ELEMENTS_TOTAL;
  nonoserialconverter->serialBufferSize = DATA_BUFFER_SERIAL_OUT_SIZE;
  nonoserialconverter->fd = nono_serial_openPort( device, SERIAL_BAUDRATE );
  // nonoserialconverter->fd = nono_serial_openPort( device, 115200 );
  //allocate the memory for the array
  nonoserialconverter->dataBuffer = (guint8*)calloc( nonoserialconverter->dataBufferSize, sizeof(guint8));
  nonoserialconverter->serialBuffer = (guint8*)calloc( nonoserialconverter->serialBufferSize, sizeof(guint8));
  
  memset(nonoserialconverter->dataBuffer,0,nonoserialconverter->dataBufferSize);
  memset(nonoserialconverter->serialBuffer,0,nonoserialconverter->serialBufferSize);

  g_print("Opened Port %s with FD: %i\n", device, nonoserialconverter->fd );
  g_print("---- nothing ---- %p\n",nono_serial_openPort);

  nono_vcd_serial_init();
}

void
gst_nonoserialconverter_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  GstNonoSerialConverter *nonoserialconverter = GST_NONOSERIALCONVERTER (object);

  GST_DEBUG_OBJECT (nonoserialconverter, "set_property");

  switch (property_id) {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_nonoserialconverter_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  GstNonoSerialConverter *nonoserialconverter = GST_NONOSERIALCONVERTER (object);

  GST_DEBUG_OBJECT (nonoserialconverter, "get_property");

  switch (property_id) {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_nonoserialconverter_dispose (GObject * object)
{
  GstNonoSerialConverter *nonoserialconverter = GST_NONOSERIALCONVERTER (object);

  GST_DEBUG_OBJECT (nonoserialconverter, "dispose");

  /* clean up as possible.  may be called multiple times */
  if( nonoserialconverter->fd >= 0 ){
    nono_serial_closePort( nonoserialconverter->fd );    
  }

  //free the memory associated with the dynamic array
  free( nonoserialconverter->dataBuffer );

  g_print("Closed Serial Port with FD: %i\n", nonoserialconverter->fd );


  G_OBJECT_CLASS (gst_nonoserialconverter_parent_class)->dispose (object);
}

void
gst_nonoserialconverter_finalize (GObject * object)
{
  GstNonoSerialConverter *nonoserialconverter = GST_NONOSERIALCONVERTER (object);

  GST_DEBUG_OBJECT (nonoserialconverter, "finalize");

  /* clean up object here */

  G_OBJECT_CLASS (gst_nonoserialconverter_parent_class)->finalize (object);
}

static gboolean
gst_nonoserialconverter_start (GstBaseTransform * trans)
{
  GstNonoSerialConverter *nonoserialconverter = GST_NONOSERIALCONVERTER (trans);

  GST_DEBUG_OBJECT (nonoserialconverter, "start");

  return TRUE;
}

static gboolean
gst_nonoserialconverter_stop (GstBaseTransform * trans)
{
  GstNonoSerialConverter *nonoserialconverter = GST_NONOSERIALCONVERTER (trans);

  GST_DEBUG_OBJECT (nonoserialconverter, "stop");

  return TRUE;
}

static gboolean
gst_nonoserialconverter_set_info (GstVideoFilter * filter, GstCaps * incaps,
    GstVideoInfo * in_info, GstCaps * outcaps, GstVideoInfo * out_info)
{
  GstNonoSerialConverter *nonoserialconverter = GST_NONOSERIALCONVERTER (filter);

  GST_DEBUG_OBJECT (nonoserialconverter, "set_info");

  return TRUE;
}

/* transform */
// static GstFlowReturn
// gst_nonoserialconverter_transform_frame (GstVideoFilter * filter, GstVideoFrame * inframe,
//     GstVideoFrame * outframe)
// {
//   GstNonoSerialConverter *nonoserialconverter = GST_NONOSERIALCONVERTER (filter);

//   GST_DEBUG_OBJECT (nonoserialconverter, "transform_frame");

//   return GST_FLOW_OK;
// }


static void
gst_nono_temp_transform (GstVideoFilter * filter,
    GstVideoFrame * frame, GstNonoSerialConverter *nonoserialconverter ){

  guint8 clrVal = nonoserialconverter->frameNum*2;
  gint i, j;
  gint width, height;
  gint pixel_stride, row_stride, row_wrap;
  gint r, g, b;
  // gint y, u, v;
  gint offsets[3];
  guint8 *data;

  data = GST_VIDEO_FRAME_PLANE_DATA (frame, 0);
  offsets[0] = GST_VIDEO_FRAME_COMP_POFFSET (frame, 0);
  offsets[1] = GST_VIDEO_FRAME_COMP_POFFSET (frame, 1);
  offsets[2] = GST_VIDEO_FRAME_COMP_POFFSET (frame, 2);

  width = GST_VIDEO_FRAME_WIDTH (frame);
  height = GST_VIDEO_FRAME_HEIGHT (frame);

  row_stride = GST_VIDEO_FRAME_PLANE_STRIDE (frame, 0);
  pixel_stride = GST_VIDEO_FRAME_COMP_PSTRIDE (frame, 0);
  row_wrap = row_stride - pixel_stride * width;

  for (i = 0; i < height; i++) {
    for (j = 0; j < width; j++) {
      r = 0xff - data[offsets[0]];
      g = data[offsets[1]];
      b = data[offsets[2]];
      r = (clrVal+i) % 0xff;
      g = r;
      b = g;
      data[offsets[0]] = r;
      data[offsets[1]] = g;
      data[offsets[2]] = b;
      data += pixel_stride;
      // if( i == height -1 && j == width - 1 ){
      //   g_print (" RGB:[ %X , %X , %X ] \n",r,g,b);
      // }
    }
    data += row_wrap;
  }


  // int someNum = 3;
  // nono_vcd_createPattern( TEST_SECTION, someNum, frame );

  // g_print ("Received data of size w:%i h:%i row_stride:%i   pixel_stride:%i    row_wrap:%i !\n", width, height, row_stride, pixel_stride, row_wrap );
}


static void
gst_nono_prepare_data
 (GstVideoFilter * filter,
    GstVideoFrame * frame, GstNonoSerialConverter *nonoserialconverter ){

  gint width, height;
  gint pixel_stride, row_stride;
  guint8 *data;

  width = GST_VIDEO_FRAME_WIDTH (frame);
  height = GST_VIDEO_FRAME_HEIGHT (frame);

  row_stride = GST_VIDEO_FRAME_PLANE_STRIDE (frame, 0);
  pixel_stride = GST_VIDEO_FRAME_COMP_PSTRIDE (frame, 0);


  guint8 *dataBuffer = nonoserialconverter->dataBuffer;
  gint dataBufferSize = nonoserialconverter->dataBufferSize;
  memset( dataBuffer, 0, dataBufferSize );

  int px, py;
  int pos;
  data = GST_VIDEO_FRAME_PLANE_DATA (frame, 0);
  for( int i=0;i<dataBufferSize ;i++ ){
    px = min( width, i%NUM_ELEMENTS_PER_STRAND );
    py = min( height, (int) (i/NUM_ELEMENTS_PER_STRAND));
    pos = px*pixel_stride + py*row_stride;
    dataBuffer[i] = data[pos] & 0xff;
  }

}

static GstFlowReturn
gst_nonoserialconverter_transform_frame_ip (GstVideoFilter * filter, GstVideoFrame * frame)
{
  GstNonoSerialConverter *nonoserialconverter = GST_NONOSERIALCONVERTER (filter);

  GST_DEBUG_OBJECT (nonoserialconverter, "transform_frame_ip");
  // gst_nono_temp_transform( filter, frame, nonoserialconverter->dataBuffer[10] );

  gst_nono_temp_transform( filter, frame, nonoserialconverter );
  gst_nono_prepare_data( filter, frame, nonoserialconverter );
  // g_print(" FRAME #%i => %X\n",nonoserialconverter->frameNum, nonoserialconverter->dataBuffer[10] );
  if( nonoserialconverter->fd >= 0 ){
    nono_vcd_serial_sendFrame( nonoserialconverter->fd, nonoserialconverter->dataBuffer, nonoserialconverter->dataBufferSize, nonoserialconverter->serialBuffer, nonoserialconverter->serialBufferSize );
    // nono_serial_writeByte( nonoserialconverter->fd, 'c' );

    // nono_serial_testSendToSerial( nonoserialconverter->fd, nonoserialconverter->dataBuffer, nonoserialconverter->dataBufferSize, nonoserialconverter->frameNum );
  }
  nonoserialconverter->frameNum ++;

  return GST_FLOW_OK;
}

static gboolean
plugin_init (GstPlugin * plugin)
{

  /* FIXME Remember to set the rank if it's an element that is meant
     to be autoplugged by decodebin. */
  return gst_element_register (plugin, "nonoserialconverter", GST_RANK_NONE,
      GST_TYPE_NONOSERIALCONVERTER);
}

/* FIXME: these are normally defined by the GStreamer build system.
   If you are creating an element to be included in gst-plugins-*,
   remove these, as they're always defined.  Otherwise, edit as
   appropriate for your external plugin package. */
#ifndef VERSION
#define VERSION "0.0.1"
#endif
#ifndef PACKAGE
#define PACKAGE "nono"
#endif
#ifndef PACKAGE_NAME
#define PACKAGE_NAME "nono_SerialConverter"
#endif
#ifndef GST_PACKAGE_ORIGIN
#define GST_PACKAGE_ORIGIN "http://say-nono.com"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    nonoserialconverter,
    "This is a plugin that will convert Video to a Serial Data Stream. by nono",
    plugin_init, VERSION, "LGPL", PACKAGE_NAME, GST_PACKAGE_ORIGIN)

