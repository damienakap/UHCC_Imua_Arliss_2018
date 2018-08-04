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
 * Update:
 *  added quadcopter states for timed events
 * 
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
QuadController quadController( &imu, &quadMotors, &batteryMonitor, &roll, &pitch, &yaw);\

StateManager stateManager( &quadController, 0 );





// create updats timers
static long updateGpsTimer  = millis();
static long updateLoopTimer = millis();
static long missionTimer    = millis();






/*
 * Setup initial values
 */
void setup(void){
  Serial.begin(9600);
  delay(500);
  quadMotors.initialize();
  quadMotors.off();

  // set MadgwickAHRS settings
  sampleFreq = 100.0f;
  beta = 0.5f;
  
  // set pid gains      [p]      [i]      [d]
  roll.setGainValues(   12.0d,   0.03d,   0.01d  );
  pitch.setGainValues(  18.0d,   0.07d,   0.01d  );
  yaw.setGainValues(    18.0d,   0.03d,    0.00d  ); 

  // initialize pid I values
  quadController.setPidIValues( -6.0d, -20.0d, -5.0d );
  
  // set max pid output values
  roll.setMaxOutput(400);   roll.setMaxIOutput(50);
  pitch.setMaxOutput(400);  pitch.setMaxIOutput(50);
  yaw.setMaxOutput(400);    yaw.setMaxIOutput(50);
  
  //set simulated pilot input
  quadController.setMaxRotationRates( PI, PI, PI );         // max input rotation rate in radians per second
  quadController.setRotationRateScalars( 3.0d, 3.0d, 1.0d );   // input rotationRate = angleDelta * scalar
  //quadController.setTotalRotationOffset(-13.0d*DEG_TO_RAD, -19.0d*DEG_TO_RAD, 0.0d );    // set input rotation offsets
  quadController.setTotalRotationOffset( 0.0d, -5.0d*DEG_TO_RAD, 0.0d );    // set input rotation offsets
  
  quadController.thrust = 0.0d;                   // the current thrust of the quadcopter motors. Thrust is from 0 to 1000 ( 0% to 100%)
  quadController.hoverThrust = 380.0d;            // (52.8) the amount of thrust to hover ~53% max thrust
  quadController.batteryAddedThrust = 300.0d;     // thrust += batteryAddedThrust* (12.8 volts / batteryVoltage)
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
 * Main Loop
 */
static bool missionDone = false;
void loop(void){
  double dt = 0.0d;
  
  // Update Gps and mission at 10Hz
  if( updateTimer( &updateGpsTimer, &dt, 100 ) ){

    // update gps
    if( gps.isAvailable() ){
      gps.update();
    }

    // update mission progress
    if( digitalRead( DEAD_TRIGGER_PIN ) && stateManager.getState() != 0 ){
      missionDone = true;
      stateManager.setState(0);
    }
    if( !missionDone){
      updateMission();
    }
    
  }

  
  
  // Update at current state 100Hz
  if( updateTimer( &updateLoopTimer, &dt, 10 ) ){ 
    stateManager.update( dt );
  }
  
}









/*
 * Update mission from bottom to top
 */
void updateMission(){
  int missionTime = millis() - missionTimer;                                          // STATE    RUN_TIME
      
  if( missionTime >= 5000 ){ stateManager.setState(0); missionDone = true; return; }   // idle     done
  if( missionTime >= 3000 ){ stateManager.setState(3); return; }                       // land     3s
  if( missionTime >= 1000 ){ stateManager.setState(1); return; }                       // hover    5s
  if( missionTime >= 0    ){ stateManager.setState(0); return; }                       // idle     1s
  
}





