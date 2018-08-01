
#include "AutoQuad.h"
#include "MyLib.h"
#include "MadgwickAHRS.h"

/*****************
   ###############
   ##### Imu #####
   ###############
 *****************
*/
Imu::Imu() {
  this->accelRaw = Adafruit_LSM303_Accel_Unified(30301);
  this->magRaw   = Adafruit_LSM303_Mag_Unified(30302);
  this->bmpRaw   = Adafruit_BMP085_Unified(18001);
  this->gyroRaw  = Adafruit_L3GD20_Unified(20);

  this->seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

}
/************************
   Initialize Imu sensors
 ************************
*/
void Imu::initialize() {
  if (!(this->accelRaw).begin()) {
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
  if (!(this->gyroRaw).begin()) {
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  if (!(this->magRaw).begin()) {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  if (!(this->bmpRaw).begin()) {
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while (1);
  }
}
/***********************************************
   Update accelerometer, gyroscope, magnetometer
 ***********************************************
*/
void Imu::update() {
  sensors_event_t event;

  this->accelRaw.getEvent(&event);
  this->accel.set( event.acceleration.x, event.acceleration.y, event.acceleration.z );

  this->gyroRaw.getEvent(&event);
  this->gyro.set( event.gyro.x, event.gyro.y, event.gyro.z);

  this->magRaw.getEvent(&event);
  this->mag.set( event.magnetic.x, event.magnetic.y, event.magnetic.z );
}
/***************************
   Calibrate Imu for offsets
 ***************************
*/
void Imu::calibrate() {
  long timer = millis();
  double kg = 0.01;
  int sampleRate = 100;

  this->accelCalib.set(0, 0, 0);
  this->gyroCalib.set(0, 0, 0);
  this->magCalib.set(0, 0, 0);

  while ( (millis() - timer) < 10000 ) {
    this->update();

    this->accelCalib  = (this->accelCalib) * (1 - kg) + (this->accel) * kg;
    this->gyroCalib   = (this->gyroCalib) * (1 - kg)  + (this->gyro) * kg;
    this->magCalib    = (this->magCalib) * (1 - kg)   + (this->mag) * kg;

    delay((int)(1000 / sampleRate));
  }
  this->accelCalib * 0.5;
  this->gyroCalib * 0.5;
  this->magCalib * 0.5;

}
/**************************
   Print Calibration Values
 **************************
*/
void Imu::printCalibration() {
  Serial.println("##### IMU Calibration Values #####");
  Serial.print("AccelCalib: "); (this->accelCalib).print(); Serial.println();
  Serial.print("GyroCalib: "); (this->gyroCalib).print(); Serial.println();
  Serial.print("MagCalib: "); (this->magCalib).print(); Serial.println();
  Serial.println("##################################");
}
/************************
   Set Calibration Values
 ************************
*/
void Imu::setCalibrationValues( Vector ac, Vector gc, Vector mc ) {
  this->accelCalib  = ac;
  this->gyroCalib   = gc;
  this->magCalib    = mc;
}




/*****************
   ###############
   ##### Gps #####
   ###############
 *****************
*/
Gps::Gps( Adafruit_GPS *ag, int pt ) {
  this->adafruitGps = ag;
  this->pingTime    = pt;
}
void Gps::initialize( bool hz10 ) {

  Serial.println("Initializing GPS...");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  (*this->adafruitGps).begin(9600);

  (*this->adafruitGps).sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //(*this->adafruitGps).sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  
  if( hz10 ){ (*this->adafruitGps).sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);  }
  else{ (*this->adafruitGps).sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); }
  
  (*this->adafruitGps).sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  //gpsSerial.println(PMTK_Q_RELEASE);
  Serial.println("Initialized GPS");
}
bool Gps::isAvailable(){
  char c = (*this->adafruitGps).read();

  if( (*this->adafruitGps).newNMEAreceived() ) {
    if( !(*this->adafruitGps).parse((*this->adafruitGps).lastNMEA()) ) {
      return false;
    }
  }else{
      return false;
  }

  return true;
}
void Gps::update() {

  if ( (*this->adafruitGps).fix ) {
    
  }
  
}






/****************************
   ##########################
   ##### PID Controller #####
   ##########################
 ****************************
*/
PidController::PidController( double *in, double* out, double *meas, double pg, double ig, double dg ) {
  this->input = in;
  this->output = out;
  this->measured = meas;

  this->pGain = pg;
  this->iGain = ig;
  this->dGain = dg;

  this->maxOutput = 400;
  this->maxIOutput = 50;

  this->i = 0.0d;
  this->errorLast = 0.0d;
}
/*****************
   Set gain values
 *****************
*/
void PidController::setGainValues( double pg, double ig, double dg ) {
  this->pGain = pg;
  this->iGain = ig;
  this->dGain = dg;
}
/**********************
   Calculate PID Output
 **********************
*/
void PidController::calculate() {
  double error = (*this->measured) - (*this->input);

  double p  = error * this->pGain;
  this->i  += error * this->iGain;
  double d  = ( error - this->errorLast ) * this->dGain;

  if( this->i > this->maxIOutput ){ this->i = this->maxIOutput; }
  
  *this->output = p + i + d;

  if( (*this->output) > this->maxOutput ){ *this->output = this->maxOutput; }
  
  this->errorLast = error;
}
void PidController::setMaxIOutput( double mio ){
  this->maxIOutput = mio;
}
void PidController::setMaxOutput( double mo ){
  this->maxOutput = mo;
}




/*************************************
   ###################################
   ##### Quadcopter Motor Object #####
   ###################################
 *************************************
*/
QuadMotors::QuadMotors( int fl, int fr, int bl, int br ) {
  pinMode( fl, OUTPUT );
  pinMode( fr, OUTPUT );
  pinMode( bl, OUTPUT );
  pinMode( br, OUTPUT );

  this->pinFL = fl;
  this->pinFR = fr;
  this->pinBL = bl;
  this->pinBR = br;
}
/******************************
   Initialize Quadcopter Motors
 ******************************
*/
void QuadMotors::initialize() {
  (this->escFL).attach(this->pinFL);
  (this->escFR).attach(this->pinFR);
  (this->escBL).attach(this->pinBL);
  (this->escBR).attach(this->pinBR);

  Serial.println("Initializing ESCs...");

  long t = millis();
  while ( millis() - t < 10000 ) {
    (this->escFL).writeMicroseconds(2000);
    (this->escFR).writeMicroseconds(2000);
    (this->escBL).writeMicroseconds(2000);
    (this->escBR).writeMicroseconds(2000);
  }
  t = millis();
  while ( millis() - t < 10000 ) {
    (this->escFL).writeMicroseconds(1000);
    (this->escFR).writeMicroseconds(1000);
    (this->escBL).writeMicroseconds(1000);
    (this->escBR).writeMicroseconds(1000);
  }
  Serial.println("ESCs Initialized.");
}
/***************************
   Set thrust for each motor
 ***************************
*/
void QuadMotors::setThrust( int fl, int fr, int bl, int br ) {
  (this->escFL).writeMicroseconds(fl);
  (this->escFR).writeMicroseconds(fr);
  (this->escBL).writeMicroseconds(bl);
  (this->escBR).writeMicroseconds(br);
}
/*******************************
   Set all motors to zero thrust
 *******************************
*/
void QuadMotors::off() {
  (this->escFL).writeMicroseconds(1000);
  (this->escFR).writeMicroseconds(1000);
  (this->escBL).writeMicroseconds(1000);
  (this->escBR).writeMicroseconds(1000);
}





/*****************************
   ###########################
   ##### Battery Monitor #####
   ###########################
 *****************************
*/
BatteryMonitor::BatteryMonitor( int pin, double r1, double r2, double fullV, double critV ) {
  this->batteryPin      = pin;
  this->resistor1       = r1;
  this->resistor2       = r2;
  this->fullVoltage     = fullV;
  this->criticalBatteryVoltage = critV;
  pinMode( this->batteryPin , INPUT );
}
/**********************
   Get battery state
 **********************
   0: normal
   1: low (below 25%)
   2: critical
*/
int BatteryMonitor::getBatteryState() {
  return this->batteryState;
}
/**********************
   Update Battery State
 **********************
*/
void BatteryMonitor::update() {

  /* update bettery voltage

     analogRead() returns 0 to 1023 ( 0 = 0volts & 1023 = 5volts ).
     Any voltage higher than 5 volts also reads as 1023.
  */
  double vOut = (double)analogRead(this->batteryPin) / 1023.0d ;                                         // read from pin. Gets values 0-1023. Map 0-1023 to 0.0-1.0 (a normalized scale)
  vOut *= 3.3d;                                                                               // map from 0.0-1.0 to 0.0-5.0 (this is the output Voltage) (0 to 5 volts)
  this->batteryVoltage =  vOut * ( this->resistor1 + this->resistor2 ) / this->resistor2 ;    // get the source voltage and store the value

  // functional voltage range
  double range = ( ( (this->fullVoltage) - (this->criticalBatteryVoltage) ) * 0.25d ) + (this->criticalBatteryVoltage);

  // update battery state
  if ( (this->batteryVoltage) <= (this->criticalBatteryVoltage) ) {  // is voltage critical
    this->batteryState = 2;
  } else if ( (this->batteryVoltage) <= range ) {              // is voltage low (less than 25% of functional voltage range)
    this->batteryState = 1;
  } else {                                                     // else voltage is in functional range
    this->batteryState = 0;
  }

}
/*********************
   Get Battery Voltage
 *********************
*/
double BatteryMonitor::getBatteryVoltage() {
  return this->batteryVoltage;
}

double BatteryMonitor::getCriticalBatteryVoltage() {
  return this->criticalBatteryVoltage;
}




/***********************************
   #################################
   ##### Quadcopter Controller #####
   #################################
 ***********************************
*/
QuadController::QuadController(  Imu *im, QuadMotors *qm, BatteryMonitor *bm, PidController *r, PidController *p, PidController *y ) {
  this->imu = im;
  this->quadMotors = qm;
  this->batteryMonitor = bm;

  this->rollPid   = r;
  this->pitchPid  = p;
  this->yawPid    = y;

  this->rollInput  = (*r).input;
  this->pitchInput = (*p).input;
  this->yawInput   = (*y).input;

  this->rollOutput  = (*r).output;
  this->pitchOutput = (*p).output;
  this->yawOutput   = (*y).output;

  this->maxRotationRateRoll   = PI;
  this->maxRotationRatePitch  = PI;
  this->maxRotationRateYaw    = PI;

  this->rollRotationRateScalar  = 2.0d;
  this->pitchRotationRateScalar = 2.0d;
  this->yawRotationRateScalar   = 2.0d;

  this->thrust = 0;
  this->hoverThrust = 500;
  this->batteryAddedThrust = 100;

  this->pidOn = true;

}
/************************************************
   Calculater the quadcopters current orientation
 ************************************************
*/
void QuadController::calculateOrientation() {
  // reset quadcopter x-, y-, z-axis
  Vector quadZAxis(0, 0, 1); //up
  Vector quadXAxis(1, 0, 0); //forward
  Vector quadYAxis(0, 1, 0); //left

  // generate orientation quaternion using Madgwick's Attitude and Heading Reference System algorithm
  MadgwickAHRSupdate(
    (*this->imu).gyro.x, (*this->imu).gyro.y, (*this->imu).gyro.z,
    (*this->imu).accel.x, (*this->imu).accel.y, (*this->imu).accel.z,
    (*this->imu).mag.x, (*this->imu).mag.y, (*this->imu).mag.z
  );
  Quaternion qRot( q0, q1, q2, q3 );

  // use quaternion to rotate vectors to quadcopters current orientation
  qRot.rotatePoint( &quadZAxis );
  qRot.rotatePoint( &quadXAxis );
  qRot.rotatePoint( &quadYAxis );

  // calculate roll pitch yaw relative to the horizontal world plane (xy-plane)
  this->totalYaw = atan2( quadXAxis.y, quadXAxis.x );

  Vector quadXOnXYplane( quadXAxis.x, quadXAxis.y, 0 );     // get projection of quadcopter x axis on the world xy-plane
  this->totalPitch = -sign(quadXAxis.z) * quadXAxis.angleTo(quadXOnXYplane);

  Vector quadYOnXYplane( cos(this->totalYaw + HALF_PI), sin(this->totalYaw + HALF_PI), 0 ); // get projection of quadcopter y axis on the world xy-plane
  this->totalRoll = sign(quadYAxis.z) * quadYAxis.angleTo(quadYOnXYplane);
}
/***********************
 * Calculate PID outputs
 ***********************
 */
void QuadController::calculatePid(){
  
  double deltaRoll  = (this->targetRoll)  - (this->totalRoll);
  double deltaPitch = (this->targetPitch) - (this->totalPitch);
  double deltaYaw   = (this->targetYaw)   - (this->totalYaw);

  double desiredRoll  = deltaRoll;
  double desiredPitch = deltaPitch * cos(this->totalRoll)   +   deltaYaw * sin(this->totalRoll);
  double desiredYaw   = deltaYaw * cos(this->totalRoll)     -   deltaPitch * sin(this->totalRoll);

  double desiredRollRate  =   desiredRoll     *   (this->rollRotationRateScalar);
  if ( desiredRollRate   >   this->maxRotationRateRoll   ) {
    desiredRollRate   =   this->maxRotationRateRoll;
  }
  if ( desiredRollRate   <   -this->maxRotationRateRoll   ) {
    desiredRollRate   =   -this->maxRotationRateRoll;
  }

  double desiredPitchRate =   desiredPitch    *   (this->pitchRotationRateScalar);
  if ( desiredPitchRate  >   this->maxRotationRatePitch  ) {
    desiredPitchRate  =   this->maxRotationRatePitch;
  }
  if ( desiredPitchRate  <   -this->maxRotationRatePitch  ) {
    desiredPitchRate  =   -this->maxRotationRatePitch;
  }

  double desiredYawRate   =   desiredYaw      *   (this->yawRotationRateScalar);
  if ( desiredYawRate    >   this->maxRotationRateYaw    ) {
    desiredYawRate    =   this->maxRotationRateYaw;
  }
  if ( desiredYawRate    <   -this->maxRotationRateYaw    ) {
    desiredYawRate    =   -this->maxRotationRateYaw;
  }

  (*this->rollInput)  = desiredRollRate;
  (*this->pitchInput) = desiredPitchRate;
  (*this->yawInput)   = desiredYawRate;

  (*this->rollPid).calculate();
  (*this->pitchPid).calculate();
  (*this->yawPid).calculate();
  
}
/*************************
   Calculate Thrust Values
 *************************
*/
void QuadController::calculate( double dt ) {
  (*this->imu).update();
  (*this->batteryMonitor).update();

  this->calculateOrientation();
  //Serial.print(dt);Serial.print(" , ");Serial.print((this->totalRoll)*RAD_TO_DEG);Serial.print(" , ");Serial.print((this->totalPitch)*RAD_TO_DEG);Serial.print(" , ");Serial.println((this->totalYaw)*RAD_TO_DEG);
  
  this->calculatePid();
  
  this->updateMotorThrust();
  
  //Serial.println( (*this->batteryMonitor).getBatteryVoltage() );
  Serial.println(*this->rollOutput);

}
/*********************
   Update Motor Thrust
 *********************
*/
void QuadController::updateMotorThrust() {

  double fl = 1000.0d + (this->thrust);
  double fr = 1000.0d + (this->thrust);
  double bl = 1000.0d + (this->thrust);
  double br = 1000.0d + (this->thrust);

  if(this->pidOn){

    double batteryVoltage = (*this->batteryMonitor).getBatteryVoltage();
    //Serial.println(batteryVoltage);
    double batteryScalar = 0.0d;
    
    if ( (*this->batteryMonitor).getBatteryState() == 0 ) {
      batteryScalar = 12.8d / (*this->batteryMonitor).getBatteryVoltage();
    }
    
    fl += -(*this->rollOutput) + (*this->pitchOutput) - (*this->yawOutput) + batteryScalar*(this->batteryAddedThrust);
    fr +=  (*this->rollOutput) + (*this->pitchOutput) + (*this->yawOutput) + batteryScalar*(this->batteryAddedThrust);
    bl += -(*this->rollOutput) - (*this->pitchOutput) + (*this->yawOutput) + batteryScalar*(this->batteryAddedThrust);
    br +=  (*this->rollOutput) - (*this->pitchOutput) - (*this->yawOutput) + batteryScalar*(this->batteryAddedThrust);
    
  }

  if (fl > 1900) {  fl = 1900; }
  if (fr > 1900) {  fr = 1900; }
  if (bl > 1900) {  bl = 1900; }
  if (br > 1900) {  br = 1900; }

  if (fl < 1000) { fl = 1000; }
  if (fr < 1000) { fr = 1000; }
  if (bl < 1000) { bl = 1000; }
  if (br < 1000) { br = 1000; }

  (*this->quadMotors).setThrust( fl, fr, bl, br );

}
/***********************************************
   Set target roll, pitch, yaw angles in radians
 ***********************************************
*/
void QuadController::setTarget( double r, double p, double y ) {
  this->targetRoll  = r;
  this->targetPitch = p;
  this->targetYaw   = y;
}
/*******************************************************************************
   Set the maximum rotation rates for the roll, pitch, yaw PID controller inputs
 *******************************************************************************
*/
void QuadController::setMaxRotationRates( double mrrr, double mrrp, double mrry ) {
  this->maxRotationRateRoll   = mrrr;
  this->maxRotationRatePitch  = mrrp;
  this->maxRotationRateYaw    = mrry;
}
void QuadController::setRotationRateScalars( double rrrs, double prrs, double yrrs ) {
  this->rollRotationRateScalar  = rrrs;
  this->pitchRotationRateScalar = prrs;
  this->yawRotationRateScalar   = yrrs;
}

