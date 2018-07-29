
/*
 * 
 */

#ifndef AUTOQUAD_H
#define AUTOQUAD_H

#include "MyLib.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>


#include <Adafruit_GPS.h>
#define gpsSerial Serial1

#include <Servo.h>

/*****************
   ###############
   ##### Imu #####
   ###############
 *****************
*/
class Imu {
  private:
    Adafruit_LSM303_Accel_Unified accelRaw;
    Adafruit_LSM303_Mag_Unified   magRaw;
    Adafruit_BMP085_Unified       bmpRaw;
    Adafruit_L3GD20_Unified       gyroRaw;

    Vector accelCalib, magCalib, gyroCalib;

  public:
    float seaLevelPressure;
    Vector accel, mag, gyro;
    Imu();
    void initialize();
    void update();
    void calibrate();
    void printCalibration();
    void setCalibrationValues( Vector a, Vector g, Vector m );
};





/*****************
   ###############
   ##### Gps #####
   ###############
 *****************
*/
class Gps {
  private:
    Adafruit_GPS *adafruitGps;
    int pingTime;                           // ms
    long updateTimeLast;

    double degreesLatitude, degreesLongitude;
    double altitude;

  public:
    Gps( Adafruit_GPS *ag , int pt);
    void initialize();
    void update();

};




/****************************
   ##########################
   ##### PID Controller #####
   ##########################
 ****************************
*/
class PidController {
  private:
    double pGain, iGain, dGain, errorLast;
    double i, maxIOutput, maxOutput;

  public:
    double *measured, *input, *output;

    PidController( double *in, double *out, double *meas, double pg, double ig, double dg );
    void setMaxIOutput( double mio );
    void setMaxOutput( double mo );
    void setGainValues( double pg, double ig, double dg );
    void calculate();
};




/*************************************
   ###################################
   ##### Quadcopter Motor Object #####
   ###################################
 *************************************
*/
class QuadMotors {
  private:
    int pinFL, pinFR, pinBL, pinBR;
    Servo escFL; // FL CW
    Servo escFR; // FR CCW
    Servo escBL; // BL CCW
    Servo escBR; // BR CW
  public:
    QuadMotors( int fl, int fr, int bl, int br );
    void setThrust( int fl, int fr, int bl, int br );
    void initialize();
    void off();
};




/*****************************
   ###########################
   ##### Battery Monitor #####
   ###########################
 *****************************
*/
class BatteryMonitor {
  private:
    int batteryPin;
    double resistor1, resistor2;
    double fullVoltage, criticalBatteryVoltage;
    double batteryVoltage;
    int batteryState;
  public:
    BatteryMonitor( int pin, double r1, double re, double fullV, double critV );
    void update();
    int getBatteryState();
    double getBatteryVoltage();
    double getCriticalBatteryVoltage();
};





/***********************************
   #################################
   ##### Quadcopter Controller #####
   #################################
 ***********************************
*/
class QuadController {
  private:
    PidController *rollPid, *pitchPid, *yawPid;
    Imu *imu;
    QuadMotors *quadMotors;
    BatteryMonitor *batteryMonitor;

    double *rollInput, *pitchInput, *yawInput;
    double *rollOutput, *pitchOutput, *yawOutput;

    double totalRoll, totalPitch, totalYaw;
    double targetRoll, targetPitch, targetYaw;

    double maxRotationRateRoll, maxRotationRatePitch, maxRotationRateYaw;
    double rollRotationRateScalar, pitchRotationRateScalar, yawRotationRateScalar;

    void calculateOrientation();
    //double batteryThrustCompensation();
    
  public:

    double thrust, hoverThrust, batteryAddedThrust;
    
    QuadController( Imu *im, QuadMotors *qm, BatteryMonitor *bm, PidController *r, PidController *p, PidController *y );
    void calculate( double dt );
    void updateMotorThrust();
    void setTarget( double r, double p, double y );
    void setMaxRotationRates( double mrrr, double mrrp, double mrry );
    void setRotationRateScalars( double rrrs, double prrs, double yrrs );
};

#endif
