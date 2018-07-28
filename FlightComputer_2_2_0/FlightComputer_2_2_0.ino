/*
 * Flight Computer for the UHCC ARLISS 2018 quadcopter
 * 
 * @author Damien Apilando
 * 07/27/2018
 * 
 * email: damienakap@gamil.com
 * 
 * In colaboration with campuses of the University of Hawaii Community Collage system
 *  - Honolulu Community Collage
 *  - Windward Community Collage
 * 
 */
#include "AutoQuad.h"

// create gps object
Adafruit_GPS adaGps(&gpsSerial);
Gps gps( &adaGps, 100 );          // update every 100 ms (10Hz)

// create imu object
Imu imu;

// create PID variable of input and output
static double rollInput    = 0.0d, rollOutput   = 0.0d;
static double pitchInput   = 0.0d, pitchOutput  = 0.0d;
static double yawInput     = 0.0d, yawOutput    = 0.0d;

// create PID controllers
//                                              Gain values ->   [p]   [i]   [d]
PidController roll(   &rollInput,   &rollOutput,  &(imu.gyro.x), 0.0d, 0.0d, 0.0d );
PidController pitch(  &pitchInput,  &pitchOutput, &(imu.gyro.y), 0.0d, 0.0d, 0.0d );
PidController yaw(    &yawInput,    &yawOutput,   &(imu.gyro.z), 0.0d, 0.0d, 0.0d );

// set Motors
//  Motor Direction -> CW   CCW   CCW   CW
//       Motor pins -> FL   FR    BL    BR
QuadMotors quadMotors( 10,  11,   12,   13 );

// create Battery Monitor
// Resistor1 and Resistor2 are in the voltage divider for reading battery voltage
//                              BatteryPin   Resistor1   Resistor2   FullVoltage   CriticalVoltage
BatteryMonitor batteryMonitor(      A0,        14.7d,      4.7d,       12.8d,          11.1d );

// creater quadcopter controller
QuadController quadController( &imu, &quadMotors, &batteryMonitor, &roll, &pitch, &yaw);\

void setup(void){
  Serial.begin(9600);
  //quadMotors.initialize();
  //quadMotors.off();
  
  // set pid gains      [p]     [i]     [d]
  roll.setGainValues(   0.0d,   0.0d,   0.0d  );
  pitch.setGainValues(  0.0d,   0.0d,   0.0d  );
  yaw.setGainValues(    0.0d,   0.0d,   0.0d  );
  
  //set user simulated input
  quadController.setMaxRotationRates( PI, PI, PI );         // max input rotation rate in radians per second
  quadController.serRotationRateScalars( 2.0, 2.0, 2.0 );   // input rotation rate = angleDelta * scalar
  
  imu.initialize();
  gps.initialize();
  
  imu.update();
  gps.update();
  batteryMonitor.update();
  
}

static long timer = millis();
void loop(void){
  
  // Update at 100Hz
  if(timer>millis()){ timer = millis(); }
  int dt = millis()-timer;
  if( dt >= 10 ){ 
    timer = millis();
    
    
    
    quadController.calculate((double)dt);
    
  }
}
