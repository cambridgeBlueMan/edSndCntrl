/******************************************************************************
SparkFun_9DOF_Edison_Block_Example.cpp
Example code for the 9DOF Edison Block
14 Jul 2015 by Mike Hord
https://github.com/sparkfun/SparkFun_9DOF_Block_for_Edison_CPP_Library

Demonstrates the major functionality of the SparkFun 9DOF block for Edison.

** Supports only I2C connection! **

Development environment specifics:
  Code developed in Intel's Eclipse IOT-DK
  This code requires the Intel mraa library to function; for more
  information see https://github.com/intel-iot-devkit/mraa

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "mraa.hpp"

#include <iostream>
#include <unistd.h>
#include "SFE_LSM9DS0.h"

// now include the osc packet stuff
#include "oscpkt.h"
#include "udp.h"

// include timer for tests
#include <time.h>
using namespace std;

int main()
{
clock_t starttime;
  LSM9DS0 *imu;
  imu = new LSM9DS0(0x6B, 0x1D);
  // The begin() function sets up some basic parameters and turns the device
  //  on; you may not need to do more than call it. It also returns the "whoami"
  //  registers from the chip. If all is good, the return value here should be
  //  0x49d4. Here are the initial settings from this function:
  //  Gyro scale:        245 deg/sec max
  //  Xl scale:          4g max
  //  Mag scale:         2 Gauss max
  //  Gyro sample rate:  95Hz
  //  Xl sample rate:    100Hz
  //  Mag sample rate:   100Hz
  // These can be changed either by calling appropriate functions or by
  //  pasing parameters to the begin() function. There are named constants in
  //  the .h file for all scales and data rates; I won't reproduce them here.
  //  Here's the list of fuctions to set the rates/scale:
  //  setMagScale(mag_scale mScl)      setMagODR(mag_odr mRate)
  //  setGyroScale(gyro_scale gScl)    setGyroODR(gyro_odr gRate)
  //  setAccelScale(accel_scale aScl)  setGyroODR(accel_odr aRate)
  // If you want to make these changes at the point of calling begin, here's
  //  the prototype for that function showing the order to pass things:
  //  begin(gyro_scale gScl, accel_scale aScl, mag_scale mScl,
	//				gyro_odr gODR, accel_odr aODR, mag_odr mODR)
  // ******************************************************************************************
  uint16_t imuResult = imu->begin();
  // ******************************************************************************************
  // either run individual changes here or add a call to begin()
// SET ODR ETC HERE
 imu->setAccelODR(imu->A_ODR_1600);
 imu->setAccelScale(imu->A_SCALE_2G);
//imu->setGyroScale(imu->G_SCALE_245DPS);

  cout<<hex<<"Chip ID: 0x"<<imuResult<<dec<<" (should be 0x49d4)"<<endl;

  bool newAccelData = false;
  bool newMagData = false;
  bool newGyroData = false;
  bool overflow = false;
  const int PORT_NUM = 57121;
 // ************************************************************************************
	// make a socket
oscpkt::UdpSocket sock;
// connect it
sock.connectTo("192.168.1.8", PORT_NUM);
// if you fail to make a socket then report the fact
if (!sock.isOk()) {
  cerr << "Error connection to port " << PORT_NUM << ": " << sock.errorMessage() << "\n";
}
// otherwise, if you succeeded in making a socket
else
{
	  // report that a socket was made succesfully
  cout << "Client started, will send packets to port " << PORT_NUM << std::endl;
}
// WE NOW HAVE A SOCKET
  // Loop and report data
  while (1)
  {
	  starttime = clock();
    // First, let's make sure we're collecting up-to-date information. The
    //  sensors are sampling at 100Hz (for the accelerometer, magnetometer, and
    //  temp) and 95Hz (for the gyro), and we could easily do a bunch of
    //  crap within that ~10ms sampling period.
    while ((newGyroData & newAccelData & newMagData) != true)
    {
      if (newAccelData != true)
      {
        newAccelData = imu->newXData();
      }
      if (newGyroData != true)
      {
        newGyroData = imu->newGData();
      }
      if (newMagData != true)
      {
        newMagData = imu->newMData(); // Temp data is collected at the same
                                      //  rate as magnetometer data.
      }
    }

    newAccelData = false;
    newMagData = false;
    newGyroData = false;

    // Of course, we may care if an overflow occurred; we can check that
    //  easily enough from an internal register on the part. There are functions
    //  to check for overflow per device.
    overflow = imu->xDataOverflow()
    		/* |
               imu->gDataOverflow() |
               imu->mDataOverflow() */

            		   ;

    if (overflow)
    {
      // cout<<"WARNING: DATA OVERFLOW!!!"<<endl;
    }

    // Calling these functions causes the data to be read from the IMU into
    //  10 16-bit signed integer public variables, as seen below. There is no
    //  automated check on whether the data is new; you need to do that
    //  manually as above. Also, there's no check on overflow, so you may miss
    //  a sample and not know it.
    imu->readAccel();
    imu->readMag();
    imu->readGyro();
    imu->readTemp();

    // *********************************************************************************

    // ************************************************************************************
    // now lets do some stuff to test max and min and speed

    int16_t minAccelX, maxAccelX;
    	if (imu->ax > maxAccelX)
    	{
    		maxAccelX = imu->ax;
      	  cout<<" Max Accel x: "<<imu->calcAccel(maxAccelX)<<endl;
    	}
    	if (imu->ax < minAccelX)
    	{
    	    minAccelX = imu->ax;
      	  cout<<"Min Accel x: "<<imu->calcAccel(minAccelX)<<endl;
    	}

// NOW SPEED
//
 //  	cout<<"Accel x:" << imu->calcAccel(imu->ax) <<endl;
//    	    cout<<"elapsed time: "<< (clock() - starttime)/ (double) CLOCKS_PER_SEC<<endl;
//







    // ********************************************************


    // Print the unscaled 16-bit signed values.
    //cout<<"-------------------------------------"<<endl;
   // oscpkt::Message msg("Gyrox"); msg.pushInt32(imu->gx);
   // oscpkt::Message msg("Gyroy"); msg.pushInt32(imu->gy);
   // oscpkt::Message msg("Gyroz"); msg.pushInt32(imu->gz);

   // X AXIS
    oscpkt::Message msg("/Accelx"); msg.pushInt32(imu->ax);
    oscpkt::PacketWriter pw;
          // add messages to an osc bundle
          //pw.startBundle().startBundle().addMessage(msg).endBundle().endBundle();
    // simpler
    pw.init().addMessage(msg);

    // send it
    bool ok = sock.sendPacket(pw.packetData(), pw.packetSize());

    // CONSOLE DIAGNOSTIC



    //
          // Y AXIS
//    oscpkt::Message msg("Accely"); msg.pushInt32(imu->ay);
//    pw.startBundle().startBundle().addMessage(msg).endBundle().endBundle();
//              ok = sock.sendPacket(pw.packetData(), pw.packetSize());
//              // Z AXIS
//    oscpkt::Message msg("Accelz"); msg.pushInt32(imu->az);
//    pw.startBundle().startBundle().addMessage(msg).endBundle().endBundle();
//              ok = sock.sendPacket(pw.packetData(), pw.packetSize());
  //  oscpkt::Message msg("Magx"); msg.pushInt32(imu->mx);
  //  oscpkt::Message msg("Magy"); msg.pushInt32(imu->my);
  //  oscpkt::Message msg("Magz"); msg.pushInt32(imu->mz);
  //  oscpkt::Message msg("Temp"); msg.pushInt32(imu->temperature);
  //  oscpkt::Message msg("-------------------------------------");

    // Print the "real" values in more human comprehensible units.
   // oscpkt::Message msg("-------------------------------------");
//    oscpkt::Message msg("Gyrox"); msg.pushInt32(imu->calcGyro(imu->gx));
//    oscpkt::Message msg("Gyroy"); msg.pushInt32(imu->calcGyro(imu->gy));
//    oscpkt::Message msg("Gyroz"); msg.pushInt32(imu->calcGyro(imu->gz));
//    oscpkt::Message msg("Accelx"); msg.pushInt32(imu->calcAccel(imu->ax));
//    oscpkt::Message msg("Accely"); msg.pushInt32(imu->calcAccel(imu->ay));
//    oscpkt::Message msg("Accelz"); msg.pushInt32(imu->calcAccel(imu->az));
//    oscpkt::Message msg("Magx"); msg.pushInt32(imu->calcMag(imu->mx));
//    oscpkt::Message msg("Magy"); msg.pushInt32(imu->calcMag(imu->my));
//    oscpkt::Message msg("Magz"); msg.pushInt32(imu->calcMag(imu->mz));

    // Temp conversion is left as an example to the reader, as it requires a
    //  good deal of device- and system-specific calibration. The on-board
    //  temp sensor is probably best not used if local temp data is required!
  //  oscpkt::Message msg("-------------------------------------"<<endl;
    // *********************************************************************************

    sleep(0.000645161); // 1/1550
  }

	return MRAA_SUCCESS;
}



/*
 *
 * // Print the unscaled 16-bit signed values.
    cout<<"-------------------------------------"<<endl;
    cout<<"Gyro x: "<<imu->gx<<endl;
    cout<<"Gyro y: "<<imu->gy<<endl;
    cout<<"Gyro z: "<<imu->gz<<endl;
    cout<<"Accel x: "<<imu->ax<<endl;
    cout<<"Accel y: "<<imu->ay<<endl;
    cout<<"Accel z: "<<imu->az<<endl;
    cout<<"Mag x: "<<imu->mx<<endl;
    cout<<"Mag y: "<<imu->my<<endl;
    cout<<"Mag z: "<<imu->mz<<endl;
    cout<<"Temp: "<<imu->temperature<<endl;
    cout<<"-------------------------------------"<<endl;

    // Print the "real" values in more human comprehensible units.
    cout<<"-------------------------------------"<<endl;
    cout<<"Gyro x: "<<imu->calcGyro(imu->gx)<<" deg/s"<<endl;
    cout<<"Gyro y: "<<imu->calcGyro(imu->gy)<<" deg/s"<<endl;
    cout<<"Gyro z: "<<imu->calcGyro(imu->gz)<<" deg/s"<<endl;
    cout<<"Accel x: "<<imu->calcAccel(imu->ax)<<" g"<<endl;
    cout<<"Accel y: "<<imu->calcAccel(imu->ay)<<" g"<<endl;
    cout<<"Accel z: "<<imu->calcAccel(imu->az)<<" g"<<endl;
    cout<<"Mag x: "<<imu->calcMag(imu->mx)<<" Gauss"<<endl;
    cout<<"Mag y: "<<imu->calcMag(imu->my)<<" Gauss"<<endl;
    cout<<"Mag z: "<<imu->calcMag(imu->mz)<<" Gauss"<<endl;
 *
 *
 *
 */

