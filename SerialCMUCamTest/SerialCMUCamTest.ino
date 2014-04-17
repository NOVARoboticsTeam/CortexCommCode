/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2013                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                                                           
/*                                                                             */
/*-----------------------------------------------------------------------------*/
                                              
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*    http://www.apache.org/licenses/LICENSE-2.0                               */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#include <Wire.h>

#include <AbsoluteIMU.h>

#include <AngleSensor.h>
#include <BaseI2CDevice.h>

#include <NXShield.h>

#include <NXShieldI2C.h>

#include <SoftwareSerial.h> 
#include <SHDefines.h>
#include <SoftI2cMaster.h>
#include <SumoEyes.h>

#include <CMUcam4.h>
#include <CMUcom4.h>

#define RED_MIN 187
#define RED_MAX 217
#define GREEN_MIN 118
#define GREEN_MAX 183
#define BLUE_MIN 106
#define BLUE_MAX 125
// Red Buckyball Tracking Values

#define RED_MIN1 78 
#define RED_MAX1 113
#define GREEN_MIN1 101
#define GREEN_MAX1 167
#define BLUE_MIN1 162
#define BLUE_MAX1 201
//Blue Buckyball Tracking Values

#define RED_MIN2 134
#define RED_MAX2 183
#define GREEN_MIN2 79
#define GREEN_MAX2 93
#define BLUE_MIN2 84
#define BLUE_MAX2 108
//Red Large ball Tracking Values


#define RED_MIN3 63
#define RED_MAX3 88
#define GREEN_MIN3 91
#define GREEN_MAX3 119
#define BLUE_MIN3 109
#define BLUE_MAX3 168
//Blue Large Ball Tracking Values



#define LED_BLINK 5 // 5 Hz
#define WAIT_TIME 5000 // 5 seconds
#define YUV_MODE true 

CMUcam4 cam(CMUCOM4_SERIAL);

struct InfraredResult
{
  byte Direction;
  byte Strength;
};

class InfraredSeeker
{
  public:
    static void Initialize();
    static boolean Test();
    static void ReadACRaw(byte* buffer);
    static void ReadDCRaw(byte* buffer);
    static InfraredResult ReadAC();
    static InfraredResult ReadDC();
    static int DirectionAngle(byte Direction);
  private:
    static InfraredResult PopulateValues(byte* buffer);
    static void ReadValues(byte OffsetAddress, byte* buffer);
    static const int Address = 0x10 / 2; //Divide by two as 8bit-I2C address is provided
};

void InfraredSeeker::Initialize()
{
  Wire.begin();
  Wire.beginTransmission(InfraredSeeker::Address);
  Wire.write(0x00);
  Wire.endTransmission();
  while(Wire.available() > 0)
    Wire.read();
}

boolean InfraredSeeker::Test()
{
  Wire.beginTransmission(InfraredSeeker::Address);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.requestFrom(InfraredSeeker::Address, 16);
  char Manufacturer_Model[16];
  while(Wire.available() < 16);
  for(byte i=0; i < 16; i++)
  {
    Manufacturer_Model[i] = Wire.read();
  }
  while(Wire.available() > 0)
    Wire.read();
  return strncmp(Manufacturer_Model, "HiTechncNewIRDir", 16)==0;
}

void InfraredSeeker::ReadValues(byte OffsetAddress, byte* buffer)
{
  Wire.beginTransmission(InfraredSeeker::Address);
  Wire.write(OffsetAddress);
  Wire.endTransmission();
  Wire.requestFrom(InfraredSeeker::Address, 6);
  while(Wire.available() < 6);
  for(byte i = 0; i < 6; i++)
  {
    buffer[i] = Wire.read();
  }
  while(Wire.available() > 0)
    Wire.read();
}

void InfraredSeeker::ReadACRaw(byte* buffer)
{
  ReadValues(0x49, buffer);
}

void InfraredSeeker::ReadDCRaw(byte* buffer)
{
  ReadValues(0x42, buffer);
}

InfraredResult InfraredSeeker::PopulateValues(byte* buffer)
{
  InfraredResult Data;
  Data.Direction = buffer[0];
  if(buffer[0] != 0)
  {
    if(buffer[0] % 2 == 0)
    {
      Data.Strength = (buffer[buffer[0] / 2] + buffer[buffer[0] / 2 + 1]) / 2;
    }
    else
    {
      Data.Strength = buffer[buffer[0] / 2 + 1];
    }
  }
  else
  {
    Data.Strength = 0;
  }
  return Data;
}

InfraredResult InfraredSeeker::ReadAC()
{
  byte buffer[6];
  ReadACRaw(buffer);
  return PopulateValues(buffer);
}

InfraredResult InfraredSeeker::ReadDC()
{
  byte buffer[6];
  ReadDCRaw(buffer);
  return PopulateValues(buffer);
}

int DirectionAngle(byte Direction)
{
  return Direction * 30 - 150;
}

typedef struct _irdata { 
  unsigned char header_aa; 
  unsigned char header_55; 
  unsigned char message_type; 
  unsigned char datalen; 
  unsigned short irDirection; 
  unsigned short irDirectionAngle; 
  unsigned short irStrength; 
  unsigned char id; 
  unsigned char checksum; 
} irData; 

typedef struct _imudata { 
  unsigned char header_aa; 
  unsigned char header_55; 
  unsigned char message_type; 
  unsigned char datalen; 
  unsigned short gyroX; 
  unsigned short gyroY;
  unsigned short gyroZ; 
  unsigned short acclX; 
  unsigned short acclY; 
  unsigned short acclZ; 
  unsigned short tiltX; 
  unsigned short tiltY; 
  unsigned short tiltZ;
  unsigned char  id;
  unsigned char  checksum;
} imudata; 
typedef struct _camdata { 
      unsigned char  header_aa;
      unsigned char  header_55;
      unsigned char  message_type;
      unsigned char  datalen;
      unsigned short mx;
      unsigned short my;
      unsigned short x1;
      unsigned short y1;
      unsigned short x2;
      unsigned short y2;
      unsigned short pixels;
      unsigned short confidence;
      unsigned char  id;
      unsigned char  checksum;
      } camdata;

#define  VEXDATAOFFSET         4
#define  VEX_DATA_BUFFER_SIZE  sizeof(imudata)

     
typedef union _vexdata { 
      camdata cdata;  
      irData idata; 
      imudata mdata; 
      unsigned char buffer[VEX_DATA_BUFFER_SIZE];  
      } vexdata;

vexdata  MyVexDataTx;

/*-----------------------------------------------------------------------------*/
/*  Initialize the software serial port                                        */
/*-----------------------------------------------------------------------------*/

SoftwareSerial mySerial(9, 8); // RX, TX

void
serialInit( long baud )
{
    mySerial.begin( baud );
}

/*-----------------------------------------------------------------------------*/
/*  Transmit one character                                                     */
/*-----------------------------------------------------------------------------*/

void
serialTxChar(unsigned char c)
{
    // software send
     mySerial.write( c );
    
    // hardware send
    //Serial.write( c );  
}

NXShield    nxshield;

//
// declare the i2c devices used on NXShield(s).
//

AbsoluteIMU imu (0x22);

void setup()
{
  InfraredSeeker::Initialize();
  cam.begin();
   VexDataInit( &MyVexDataTx );
  // Wait for auto gain and auto white balance to run.
  nxshield.init( SH_SoftwareI2C );
  cam.LEDOn(LED_BLINK);
  delay(WAIT_TIME); 
  serialInit(115200); 
  imu.init( &nxshield, SH_BAS2 );

      serialInit(115200); 

  // Turn auto gain and auto white balance off.

  cam.autoGainControl(false);
  cam.autoWhiteBalance(false);

  cam.LEDOn(CMUCAM4_LED_ON);
}

void loop()
{
   static  uint32_t  timer = 0;
    InfraredResult InfraredBall = InfraredSeeker::ReadAC();
    // Run every 20mS (50x per second)
    if(millis() - timer < 20)
      return;     
    timer =  millis();
   
  CMUcam4_tracking_data_t t_data;

  cam.pollMode(true);
  gyro mygyro;
  accl myaccl;
  for(;;)
  {
    cam.trackColor(RED_MIN, RED_MAX, GREEN_MIN, GREEN_MAX, BLUE_MIN, BLUE_MAX);
    cam.getTypeTDataPacket(&t_data); // Get a tracking packet.
    MyVexDataTx.cdata.mx =  (short)&t_data.mx; 
    MyVexDataTx.cdata.my = (short)&t_data.my; 
    MyVexDataTx.cdata.x1 = (short)&t_data.x1; 
    MyVexDataTx.cdata.y1 = (short)&t_data.y1; 
    MyVexDataTx.cdata.x2 = (short)&t_data.x2; 
    MyVexDataTx.cdata.y2 = (short)&t_data.y2; 
    MyVexDataTx.cdata.pixels = (short)&t_data.pixels;
    MyVexDataTx.cdata.confidence = (short)&t_data.confidence;
   
     delay(100); 
    
    MyVexDataTx.idata.irDirection = InfraredBall.Direction; 
    MyVexDataTx.idata.irDirectionAngle = DirectionAngle(InfraredBall.Direction); 
    MyVexDataTx.idata.irStrength = InfraredBall.Strength;
   
     delay(100); 
    
    MyVexDataTx.mdata.gyroX = mygyro.gx; 
    MyVexDataTx.mdata.gyroY = mygyro.gy; 
    MyVexDataTx.mdata.gyroZ = mygyro.gz; 
    MyVexDataTx.mdata.acclX = myaccl.ax; 
    MyVexDataTx.mdata.acclY = myaccl.ay; 
    MyVexDataTx.mdata.acclZ = myaccl.az; 
    MyVexDataTx.mdata.tiltX = myaccl.tx; 
    MyVexDataTx.mdata.tiltY = myaccl.ty; 
    MyVexDataTx.mdata.tiltZ = myaccl.tz;  

    // Process the packet data safely here.
 

    
    delay(50); 
    MyVexDataTx.cdata.id++;
    
     MyVexDataTx.idata.id++;
     
     MyVexDataTx.mdata.id++; 
    
    // Calculate checksum   
    VexDataChecksum( &MyVexDataTx );

    // Transmit data
    delay(200); 
    VexDataTransmit( &MyVexDataTx );

    
    
  }


  }


void
VexDataInit( union _vexdata *v )
{
    int  i;
    
    // clear all
    for(i=0;i<VEX_DATA_BUFFER_SIZE;i++)
      v->buffer[i] = 0;
    
    // Initialize packet  
    v->cdata.header_aa    = 0xAA;
    v->cdata.header_55    = 0x55;
    v->cdata.message_type = 0x01; // Only one type of message
    v->cdata.datalen      = 0x10; // 16 bytes of data
    v->cdata.id           = 0x00;
    
    v->idata.header_aa    = 0xAA;
    v->idata.header_55    = 0x55;
    v->idata.message_type = 0x01; // Only one type of message
    v->idata.datalen      = 0x10; // 16 bytes of data
    v->idata.id           = 0x00;
    
    v->mdata.header_aa    = 0xAA;
    v->mdata.header_55    = 0x55;
    v->mdata.message_type = 0x01; // Only one type of message
    v->mdata.datalen      = 0x10; // 16 bytes of data
    v->mdata.id           = 0x00;
}

/*-----------------------------------------------------------------------------*/
/*  Calculate the checksum for the VEX data                                    */
/*-----------------------------------------------------------------------------*/

void
VexDataChecksum( union _vexdata  *v )
{
    int  i;
    int  cs = 0;
   
    for(i=0;i<(VEX_DATA_BUFFER_SIZE-1);i++)
      cs += v->buffer[i];
    
    v->cdata.checksum = 0x100 - (cs & 0xFF);
    v->idata.checksum = 0x100 - (cs & 0xFF);
    v->mdata.checksum = 0x100 - (cs & 0xFF);
}

/*-----------------------------------------------------------------------------*/
/*  send the VEX data to the serial port                                       */
/*-----------------------------------------------------------------------------*/

void
VexDataTransmit( union _vexdata  *v )
{
    int  i;
    
    for(i=0;i<VEX_DATA_BUFFER_SIZE;i++)
      serialTxChar( v->buffer[i] );
}


/***************************************************************************//**
* @file
* @par MIT License - TERMS OF USE:
* @n Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* @n
* @n The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
* @n
* @n THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*******************************************************************************/
