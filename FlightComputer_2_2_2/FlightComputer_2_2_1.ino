/*
  Flight Computer for the UHCC ARLISS 2018 quadcopter

  @author Damien Apilando
  last Updated: 09/02/2018

  email: damienakap@gmail.com

  In collaboration with campuses of the University of Hawaii Community College system
   - Honolulu Community College
   - Windward Community College

  required arduino Libraries:
   Adafruit_Sensor.h
   Adafruit_LSM303_U.h
   Adafruit_BMP085_U.h
   Adafruit_L3GD20_U.h
   Adafruit_10DOF.h
   Adafruit_GPS.h

  It assumes you are using the UHCC ARLISS quadcopter parts
  - see at: https://github.com/damienakap/UHCC_Imua_Arliss_2018

  Update:
   fixed pid running when stationary on the ground
   pid gains and angle offsets set for stabbility.

*/







#include "MyLib.h"
#include "AutoQuad.h"
#include "AutoQuadStates.h"
#include "MadgwickAHRS.h"



#define DEAD_TRIGGER_PIN 6



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
QuadController quadController( &imu, &quadMotors, &batteryMonitor, &roll, &pitch, &yaw); \

StateManager stateManager( &quadController, 0 );





// create updats timers
static long updateGpsTimer  = millis();
static long updateLoopTimer = millis();
static long missionTimer    = millis();






/*
   Setup initial values
*/
void setup(void) {
  Serial.begin(9600);
  delay(500);
  quadMotors.initialize();
  quadMotors.off();

  // set MadgwickAHRS settings
  sampleFreq = 100.0f;
  beta = 0.5f;

  // set pid gains      [p]      [i]      [d]
  roll.setGainValues(   23.0d,   0.9d,   16.0d  );
  pitch.setGainValues(  26.0d,   1.0d,   16.0d  );
  yaw.setGainValues(    130.0d,   2.0d,    0.0d  );

  // initialize pid I values
  //quadController.setPidIValues( -6.0d, 8.0d, -0.0d );
  quadController.setPidIValues( 0.0d, 0.0d, -0.0d );

  // set max pid output values
  roll.setMaxOutput(400);   roll.setMaxIOutput(60);
  pitch.setMaxOutput(400);  pitch.setMaxIOutput(80);
  yaw.setMaxOutput(450 );    yaw.setMaxIOutput(300);

  //set simulated pilot input  ORDER: ROLL  PITCH  YAW
  quadController.setMaxRotationRates( PI * 0.1d, PI * 0.1d, PI * 0.1d );   // max input rotation rate in radians per second
  quadController.setRotationRateScalars( 2.5d, 2.5d, 3.0d );   // input rotationRate = angleDelta * scalar
  //quadController.setTotalRotationOffset(-13.0d*DEG_TO_RAD, -19.0d*DEG_TO_RAD, 0.0d );    // set input rotation offsets
  quadController.setTotalRotationOffset( -3.0d * DEG_TO_RAD, -6.9s * DEG_TO_RAD, 0.0d * DEG_TO_RAD ); // set input rotation offsets

  quadController.thrust = 0.0d;                   // the current thrust of the quadcopter motors. Thrust is from 0 to 1000 ( 0% to 100%)
  quadController.hoverThrust = 198.0d;            // (52.8) the amount of thrust to hover ~53% max thrust
  quadController.batteryAddedThrust = 396.0d;     // thrust += batteryAddedThrust* (12.8 volts / batteryVoltage)
  // * a fully charged 3s battery has a stored voltage of about 12.65 volts

  // set State to idle
  stateManager.setState( 0 );

  imu.initialize();
  gps.initialize( true );
  pinMode( DEAD_TRIGGER_PIN, INPUT_PULLUP );
  delay(100);
  imu.update();
  gps.update();
  batteryMonitor.update();

  missionTimer = millis();
  updateLoopTimer = millis();
  updateGpsTimer = millis();
  Serial.println("running code");

}








/*
   Main Loop
*/
static bool missionDone = false;
void loop(void) {
  double dt = 0.0d;

  // Update Gps and mission at 10Hz
  if ( updateTimer( &updateGpsTimer, &dt, 100 ) ) {

    // update gps
    if ( gps.isAvailable() ) {
      gps.update();
    }

    // update mission progress
    if ( digitalRead( DEAD_TRIGGER_PIN ) && stateManager.getState() != 0 ) {
      Serial.println("DeadTrigger Pulled");
      missionDone = true;
      stateManager.setState(0);
    }
    if ( !missionDone ) {
      updateMission();
    }

  }



  // Update at current state 100Hz
  if ( updateTimer( &updateLoopTimer, &dt, 10 ) ) {
    stateManager.update( dt );
  }

}









/*
   Update mission from bottom to top
*/
int updateMission() {
  int missionTime = millis() - missionTimer;                                          // STATE    RUN_TIME

  if ( missionTime >= 19000 ) {
    stateManager.setState(0);  // idle     done
    missionDone = true;
    return 0;
  }
  if ( missionTime >= 18000 ) {
    stateManager.setState(3);  // land     1s
    return 0;
  }
  if ( missionTime >= 8000 ) {
    stateManager.setState(2);  // Nav    10s
    Serial.println(yawOutput);
    //Serial.println("Hover");
    return 0;
  }
  if ( missionTime >= 6000 ) {
    stateManager.setState(1);  // launch   2s
    Serial.println(yawOutput);
    //Serial.println("Launch");
    return 0;
  }
  if ( missionTime >= 0    ) {
    stateManager.setState(0);  // idle     5s
    Serial.println(yawOutput);
    //Serial.println("Idle");
    return 0;
  }

}





