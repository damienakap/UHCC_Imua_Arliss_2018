/*
 * Flight Computer for the UHCC ARLISS 2018 quadcopter
 * 
 * @author Damien Apilando
 * last Updated: 07/28/2018
 *
 * email: damienakap@gmail.com
 * 
 * In collaboration with campuses of the University of Hawaii Community College system
 *  - Honolulu Community College
 *  - Windward Community College
 * 
 * required arduino Libraries:
 *  Adafruit_Sensor.h
 *  Adafruit_LSM303_U.h
 *  Adafruit_BMP085_U.h
 *  Adafruit_L3GD20_U.h
 *  Adafruit_10DOF.h
 *  Adafruit_GPS.h
 *
 * It assumes you are using the UHCC ARLISS quadcopter parts
 * - see at: https://github.com/damienakap/UHCC_Imua_Arliss_2018
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

static boolean runLoop = true;
static long timer = millis();
static long missionTimer = millis();

void setup(void){
  Serial.begin(9600);
  delay(500);
  quadMotors.initialize();
  quadMotors.off();
  
  // set pid gains      [p]     [i]     [d]
  roll.setGainValues(   15.0d,   0.2d,   0.2d  );
  pitch.setGainValues(  40.0d,   0.3d,   0.2d  );
  yaw.setGainValues(    50.0d,   0.4d,   0.0d  );
  
  //set simulated pilot input
  quadController.setMaxRotationRates( PI, PI, PI );         // max input rotation rate in radians per second
  quadController.setRotationRateScalars( 0.8, 1.0, 0.0 );   // input rotation rate = angleDelta * scalar

  quadController.thrust = 0.0d;
  quadController.hoverThrust = 528.0d;
  quadController.batteryAddedThrust = 100.0d;
  
  imu.initialize();
  gps.initialize();
  
  imu.update();
  gps.update();
  batteryMonitor.update();

  missionTimer = millis();
  timer = millis();
  Serial.println("running code");
  
}

void loop(void){
  
  // Update at 100Hz
  if(timer>millis()){ timer = millis(); }
  int dt = millis()-timer;
  if( millis()-missionTimer > 10000 ){ runLoop = false; }

  if(runLoop){
    if( dt >= 10){ 
      timer = millis();
      quadController.thrust = quadController.hoverThrust;
      quadController.calculate((double)dt);
    }
  }else{
    Serial.println("Done running code");
    quadMotors.off();
  }
}
