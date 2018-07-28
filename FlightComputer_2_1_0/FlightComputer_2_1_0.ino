
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

//double pi = 3.1415926535897932;
//static double pi2 = pi*2;
//static double degToRad = pi/180;
//static double radToDeg = 180/pi;

static double gpsTargLatitude = 21.395293;
static double gpsTargLongitude = -157.738333;
static double gpsTargAltitude = 30;

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

#include <Servo.h>

Servo ESC0; // FL CW
Servo ESC1; // FR CCW
Servo ESC2; // BL CCW
Servo ESC3; // BR CW

double batR1 = 14.7;  //kOhm
double batR2 = 4.7;   //kOhm 

double magnitude( double x, double y ){ return pow( x*x + y*y, 0.5); }
double magnitude( double x, double y, double z ){ return pow( x*x + y*y + z*z , 0.5); }

#include <Adafruit_GPS.h>

#define gpsSerial Serial1

Adafruit_GPS GPS(&gpsSerial);

#define GPSECHO  true

static boolean usingInterrupt = false;
void useInterrupt(boolean);

static int gps_ping_time = 500; // ms

static int gps_hour;
static int gps_minute;
static int gps_second;
static int gps_millisecond;
static int gps_day;
static int gps_month;
static int gps_year;

static bool gps_fix;
static int gps_fix_quality;
static float gps_latitude;
static float gps_longitude;
static float gps_lat;
static float gps_lon;
static float gps_latitude_deg;
static float gps_longitude_deg;
static float gps_angle;
static float gps_speed; //knots
static float gps_altitude;
static float gps_satellites;

void initIMU(){
  if(!accel.begin()){
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!gyro.begin()){
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!mag.begin()){
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin()){
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}

class Vector{
  private:
    double X;
    double Y;
    double Z;
  public:
    Vector( double x, double y, double z){
      this->X = x;
      this->Y = y;
      this->Z = z;
    }

    // functions
    void multiply( double s ){
      this->X *= s;
      this->Y *= s;
      this->Z *= s;
    }
    void add( double n0, double n1, double n2){
      this->X += n0;
      this->Y += n1;
      this->Z += n2;
    }
    void add( Vector v ){
      this->X += v.X;
      this->Y += v.Y;
      this->Z += v.Z;
    }
    void subtract( double n0, double n1, double n2 ){
      this->X -= n0;
      this->Y -= n1;
      this->Z -= n2;
    }
    void subtract( Vector v ){
      this->X -= v.X;
      this->Y -= v.Y;
      this->Z -= v.Z;
    }
    double dot( Vector v ){
      return (this->X * v.X) + (this->Y * v.Y) + (this->Z * v.Z);
    }
    double dotXY( Vector v ){
      return (this->X * v.X) + (this->Y * v.Y);
    }
    double dotYZ( Vector v ){
      return (this->Y * v.Y) + (this->Z * v.Z);
    }
    double dotZX( Vector v ){
      return (this->X * v.X) + (this->Z * v.Z);
    }
    
    void normalize(){
      double l = this->getLength();
      this->X /= l;
      this->Y /= l;
      this->Z /= l;
    }
    void copy( Vector v ){
      this->set( v.X, v.Y, v.Z );
    }
    void rotateX( double theta ){
      this->Y = (this->Y * cos(theta)) - (this->Z * sin(theta));
      this->Z = (this->Y * sin(theta)) + (this->Z * cos(theta));
    }
    void rotateY( double theta ){
      this->X = (this->X * cos(theta)) + (this->Z * sin(theta));
      this->Z = -(this->X * sin(theta)) + (this->Z * cos(theta));
    }
    void rotateZ( double theta ){
      this->X = (this->X * cos(theta)) - (this->Y * sin(theta));
      this->Y = (this->X * sin(theta)) + (this->Y * cos(theta));
    }
    
    // get
    double getThetaDelta( Vector v ){
      return acos( this->dot(v) / ( this->getLength() * v.getLength() ) );
    }
    double getThetaDeltaXY( Vector v ){
      return acos( this->dotXY(v) / ( this->getXYLength() * v.getXYLength() ) );
    }
    double getThetaDeltaYZ( Vector v ){
      return acos( this->dotYZ(v) / ( this->getYZLength() * v.getYZLength() ) );
    }
    double getThetaDeltaZX( Vector v ){
      return acos( this->dotZX(v) / ( this->getZXLength() * v.getZXLength() ) );
    }
    double getX(){ return this->X; }
    double getY(){ return this->Y; }
    double getZ(){ return this->Z; }
    double getXYLength(){
      return  magnitude( this->X, this->Y );
    }
    double getYZLength(){
      return pow( 
        (this->Y)*(this->Y) +
        (this->Z)*(this->Z)
        , 0.5
      );
    }
    double getZXLength(){
      return pow( 
        (this->X)*(this->X) +
        (this->Z)*(this->Z)
        , 0.5
      );
    }
    double getLength(){ 
      return pow( 
        (this->X)*(this->X) +
        (this->Y)*(this->Y) +
        (this->Z)*(this->Z)  
        , 0.5
      );
    }

    // set
    void setX( double x ){ this->X = x; }
    void setY( double y ){ this->Y = y; }
    void setZ( double z ){ this->Z = z; }
    void set( double x, double y, double z ){
      this->X = x;
      this->Y = y;
      this->Z = z;
    }
};

class PID{
  private:
    double* input;
    double* output;
    double* target;
    double pGain, iGain, dGain;

    double total;
    double inputLast;
    double deltaLast;

    double maxIValue = 400;
    double maxOutputValue = 400;
    
    boolean kEnable;
    double estimate;
    double* measurement;
    double uMeasurement;
    double uEstimate;
    double kG;

    double rangeMin = 0;
    double rangeMax = 0;
    int rangeClipType = 0;
    
  public:
    PID( double* in, double* out, double* targ, double p, double i, double d){
      this->input = in;
      this->output = out;
      this->target = targ;
      this->pGain = p;
      this->iGain = i;
      this->dGain = d;

      this->deltaLast = 0;
      this->total = 0;
      this->inputLast = *input;

      
      
      this->kEnable = false;
      this->estimate = 0;
      this->uEstimate = 1;
      this->uMeasurement = 0;
      this->kG = 1;
      
    }
    void setRange( double rMin, double rMax, boolean clipType){
      this->rangeMin = rMin;
      this->rangeMax = rMax;
      this->rangeClipType = clipType;
    }
    void enableKalmanFilter( boolean b ){ this->kEnable = b; }
    void setKalmanFilter( double* meas, double inEst, double uEst, double uMeas ){
      this->estimate = inEst;
      this->uEstimate = uEst;
      this->measurement = meas;
      this->uMeasurement = uMeas;
      this->kG = 1;
    }
    
    void calculate(double dt){
      
      // PID
        double delta = (*this->input - this->inputLast);
        //this->total += (*this->input)*dt;

        // clip value between -pi and pi
        while( this->total > TWO_PI ){ this->total -= TWO_PI; }
        while( this->total < -TWO_PI ){ this->total += TWO_PI; }
        if( this->total > PI ){ this->total -= TWO_PI; }
        if( this->total < -PI ){ this->total += TWO_PI; }

        //Serial.println(delta);
        double p = *this->input * this->pGain;
        double i = ( total - (*this->target) ) * this->iGain;
        double d = ( delta ) * this->dGain;

        if( i < -this->maxIValue){ i = -this->maxIValue; }
        if( i > this->maxIValue){ i = this->maxIValue; }
        
        *this->output = p + i + d;

        if( *this->output < -this->maxOutputValue ){ *this->output = -this->maxOutputValue; }
        if( *this->output > this->maxOutputValue ){ *this->output = this->maxOutputValue; }
        
        this->deltaLast = delta;
        this->inputLast = *this->input;
      
      
    }
    
    double getTotal(){ return this->total;}
    void setTotal( double v ) { this->total = v; }
    void setMaxIValue( double m ){ this->maxIValue = m; }
    void setMaxOutputValue( double m ){ this->maxOutputValue = m; }
    
};

static double thrust = 0.0d;
static double hoverThrust = 515.0d;

double getBatteryVoltage(){
  double vOut = (analogRead(A0)*5/1024);
  return ( vOut * ( batR1 + batR2 ) / batR2 );
}

// yaw +:left -:right
static double yawInput = 0;
static double yawOutput = 0;
static double yawTarget = 0;
static double yawMeasured = 0;  
static double yawAngle = 0;
//static PID yaw( &yawInput, &yawOutput, &yawTarget, 20.0f, 80.0f, 0.02f);
static PID yaw( &yawInput, &yawOutput, &yawTarget, 15.0d, 35.0d, 0.0d);

// pitch +:down -:up
static double pitchInput = 0;
static double pitchOutput = 0;
static double pitchTarget = 10.5*DEG_TO_RAD;
static double pitchMeasured = 0;
static double pitchAngle = 0;
static double pitchOffset = 15*DEG_TO_RAD;
static PID pitch( &pitchInput, &pitchOutput, &pitchTarget, 20.0d, 90.0d, 0.0d);

// roll +:right -:left
static double rollInput = 0;
static double rollOutput = 0;
static double rollTarget = -2.8d*DEG_TO_RAD;
static double rollMeasured = 0;
static double rollAngle = 0;
static PID roll( &rollInput, &rollOutput, &rollTarget, 12.0d, 70.0d, 0.002d);

static Vector imuAccel(0,0,0);
static Vector imuGyro(0,0,0);
static Vector imuMag(0,0,0);

/*
static Vector imuGyroCalib(0,0,0);
static Vector imuAccelAv(0,0,0);
static double imuAccelAvSampleSize = 20;
*/
static Vector magneticHeading(0,0,0);
static double magneticMagnitude = 1.0;
static Vector gravity(0,0,0);
static double gravityMagnitude = 9.8;
static double thetaDeltaGravityMagnetic = 0;

//static Vector gpsPosAv(0,0,0);
static double gpsLatRadAv = 0;
static double gpsLonRadAv = 0;
static double gpsDeltaX = 0;
static double gpsDeltaY = 0;
static double gpsPlanarDistance = 0;

double getThrustAcceloration(){
  return gravityMagnitude/(hoverThrust - 1000.0d) ;
}

void updateIMU(){
  sensors_event_t event;
  
  accel.getEvent(&event);
  imuAccel.set( event.acceleration.x, event.acceleration.y, event.acceleration.z );
  //imuAccelAv.multiply( (imuAccelAvSampleSize-1)/imuAccelAvSampleSize );
  //imuAccelAv.add( event.acceleration.x/imuAccelAvSampleSize, event.acceleration.y/imuAccelAvSampleSize, event.acceleration.z/imuAccelAvSampleSize );

  gyro.getEvent(&event);
  //imuGyro.set( event.gyro.x - imuGyroCalib.getX(), event.gyro.y - imuGyroCalib.getY(), event.gyro.z - imuGyroCalib.getZ() );
  imuGyro.set( event.gyro.x, event.gyro.y, event.gyro.z);
  
  mag.getEvent(&event);
  imuMag.set( event.magnetic.x, event.magnetic.y, event.magnetic.z );
  
}

void setup() {
  Serial.begin(115200);

  ESC0.attach(10);
  ESC1.attach(11);
  ESC2.attach(12);
  ESC3.attach(13);  
  
  Serial.println("Initializing ESCs...");
  
  long t = millis();
  while( millis()-t < 8000 ){
    ESC0.writeMicroseconds(2000);
    ESC1.writeMicroseconds(2000);
    ESC2.writeMicroseconds(2000);
    ESC3.writeMicroseconds(2000);
  }
  t = millis();
  while( millis()-t < 10000 ){
    ESC0.writeMicroseconds(1000);
    ESC1.writeMicroseconds(1000);
    ESC2.writeMicroseconds(1000);
    ESC3.writeMicroseconds(1000);
  }
  Serial.println("ESCs Initialized.");
  Serial.println("Calibrating IMU...");

  initIMU();
  
  t = millis();
  double calibVal = 0.05;
  while( millis()-t < 10000 ){
    
    updateIMU();
    
    gravity.multiply( 1-calibVal );
    gravity.setX( gravity.getX() + calibVal*imuAccel.getX() );
    gravity.setY( gravity.getY() + calibVal*imuAccel.getY() );
    gravity.setZ( gravity.getZ() + calibVal*imuAccel.getZ() );

    magneticHeading.multiply( 1-calibVal );
    magneticHeading.setX( magneticHeading.getX() + calibVal*imuMag.getX() );
    magneticHeading.setY( magneticHeading.getY() + calibVal*imuMag.getY() );
    magneticHeading.setZ( magneticHeading.getZ() + calibVal*imuMag.getZ() );

    /*
    imuGyroCalib.multiply( 1-calibVal );
    imuGyroCalib.setX( imuGyroCalib.getX() + calibVal*imuGyro.getX() );
    imuGyroCalib.setY( imuGyroCalib.getY() + calibVal*imuGyro.getY() );
    imuGyroCalib.setZ( imuGyroCalib.getZ() + calibVal*imuGyro.getZ() );
    */
  }
  //imuGyroCalib.multiply(2);
  gravityMagnitude = gravity.getLength();
  magneticMagnitude = magneticHeading.getLength();
  thetaDeltaGravityMagnetic = magneticHeading.getThetaDelta( gravity );
  Serial.println("IMU Calibrated.");


  //  set pid limits
  yaw.setMaxIValue(200);
  yaw.setMaxOutputValue(200);
  pitch.setMaxIValue(400);
  pitch.setMaxOutputValue(400);
  roll.setMaxIValue(400);
  roll.setMaxOutputValue(400);

  // initialize GPS
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  Serial.println("Initializing GPS...");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  gpsSerial.println(PMTK_Q_RELEASE);

  gps_hour = GPS.hour;
  gps_minute = GPS.minute;
  gps_second = GPS.seconds;
  gps_millisecond = GPS.milliseconds;
  gps_day = GPS.day;
  gps_month = GPS.month;
  gps_year = GPS.year;
  Serial.println("Initialized GPS");
  
}

static long loopTimeLast = millis();
double loopDelay = 0.01d;   // 10 milliseconds

static long gpsTimeLast = millis();
double gpsDelay = 0.1d;


void loop() {

  // update gps
  boolean runGPS = true;
  char c = GPS.read();
  //if (GPSECHO) { if(c) Serial.print(c); }
  if (GPS.newNMEAreceived()){ if (!GPS.parse(GPS.lastNMEA())) { runGPS = false; } }
  
  if (gpsTimeLast > millis()){ gpsTimeLast = millis(); }
  double gpsTd = (double)(millis() - gpsTimeLast)/1000.0d;
  if( gpsTd >= gpsDelay && runGPS){

    // update timer
    gpsTd = gpsDelay;
    gpsTimeLast = millis();

    gps_hour =        GPS.hour;
    gps_minute =      GPS.minute;
    gps_second =      GPS.seconds;
    gps_millisecond = GPS.milliseconds;  
    gps_day =        GPS.day;
  
    gps_fix =         GPS.fix;
    gps_fix_quality = GPS.fixquality;
    //Serial.print("Fix: ");Serial.print(gps_fix);Serial.print(", ");Serial.println(gps_fix_quality);
    if (gps_fix){
      gps_latitude =      GPS.latitude;
      gps_longitude =     GPS.longitude;
      gps_lat =           GPS.lat;
      gps_lon =           GPS.lon;
      gps_latitude_deg =  GPS.latitudeDegrees;
      gps_longitude_deg = GPS.longitudeDegrees;
      gps_angle =         GPS.angle;
      gps_speed =         GPS.speed; //knots
      gps_altitude =      GPS.altitude;
      gps_satellites =    (int)GPS.satellites;

      double gain = 0.1;
      double latRad = gps_latitude_deg*DEG_TO_RAD;
      double lonRad = gps_longitude_deg*DEG_TO_RAD;
      if( abs(latRad-gpsLatRadAv) > 0.1 ){ gpsLatRadAv =  latRad;}
      if( abs(lonRad-gpsLonRadAv) > 0.1 ){ gpsLonRadAv =  lonRad;}
      gpsLatRadAv = (1-gain)*gpsLatRadAv + gain*latRad;
      gpsLonRadAv = (1-gain)*gpsLonRadAv + gain*lonRad;
      //R = √ [ (r1² * cos(B))² + (r2² * sin(B))² ] / [ (r1 * cos(B))² + (r2 * sin(B))² ] 
      double r1 = 6378.1370;  // equatorial radius
      double r2 = 6356.7523;  // polar radius
      double r = pow(  ( pow(r1*r1*cos(gpsLatRadAv), 2)+pow( r2*r2*sin(gpsLatRadAv), 2) )/( pow(r1*cos(gpsLatRadAv), 2)+pow( r2*sin(gpsLatRadAv), 2) ) , 0.5);
      double rMinor = r*cos(gpsLatRadAv);
      
      //gpsPosAv.set( (1-gain)*gpsPosAv.getX() + gain*rMinor*cos(lon_rad), (1-gain)*gpsPosAv.getY() + gain*rMinor*sin(lon_rad), (1-gain)*gpsPosAv.getZ() + gain*r*sin(lat_rad) );

      gpsDeltaY = (gpsTargLatitude*DEG_TO_RAD - gpsLatRadAv)*r;
      gpsDeltaX = (gpsTargLongitude*DEG_TO_RAD - gpsLonRadAv)*rMinor;
      gpsPlanarDistance = magnitude( gpsDeltaX, gpsDeltaY );
      //Serial.print(gpsPlanarDistance);Serial.print(" , ");Serial.print(gpsDeltaX);Serial.print(" , ");Serial.println(gpsDeltaY);
      
      //Serial.print(" GPS Pos: ");Serial.print(r);Serial.print(", ");Serial.print(gpsPosAv.getX());Serial.print(", ");Serial.print(gpsPosAv.getY());Serial.print(", ");Serial.println(gpsPosAv.getZ());
      //Serial.print("GPS position: ");Serial.print(gps_latitude,DEC);Serial.print(" ,");Serial.println(gps_longitude);
      //Serial.print("GPS position deg: ");Serial.print(gps_latitude_deg,DEC);Serial.print(" ,");Serial.println(gps_longitude_deg,DEC);
      //Serial.print("GPS angle: ");Serial.println(gps_angle);
      //Serial.print("GPS speed: ");Serial.println(gps_speed);
      //Serial.print("GPS altitude: ");Serial.println(gps_altitude);
    }
    
  }

  // update
  if (loopTimeLast > millis()){ loopTimeLast = millis(); }
  double td = (double)(millis() - loopTimeLast)/1000.0d;
  if( td >= loopDelay ){
    
    //update timer
    td = loopDelay;
    loopTimeLast = millis();
    
    updateIMU();
    
    // yaw translate pitch-roll
    double temp = rollAngle;
    rollAngle += - pitchAngle*sin( imuGyro.getZ()*td );
    pitchAngle += rollAngle*sin( imuGyro.getZ()*td );

    // pitch to yaw due to roll

    rollAngle   += imuGyro.getX()*td;
    pitchAngle  += imuGyro.getY()*td*cos( rollAngle ) - imuGyro.getZ()*td*sin( rollAngle );
    yawAngle    += imuGyro.getY()*td*sin( rollAngle ) + imuGyro.getZ()*td*cos( rollAngle );
    
    // pitch to roll if pitch magnitude is greater than pi/2, flip 
    /*
    if( pitch.getTotal() < -pi/2 ){
      pitch.setTotal( -pi - pitch.getTotal() );
      if( roll.getTotal() < 0 ){ roll.setTotal( roll.getTotal() + pi ); }
      if( roll.getTotal() > 0 ){ roll.setTotal( roll.getTotal() - pi ); }
      
    } else if( pitch.getTotal() > pi/2 ){
      pitch.setTotal( pi - pitch.getTotal() );
      if( roll.getTotal() < 0 ){ roll.setTotal( roll.getTotal() + pi ); }
      if( roll.getTotal() > 0 ){ roll.setTotal( roll.getTotal() - pi ); }
    }
    */
    
    // imu filters
      // yaw filter
    double magXYLength = magnitude( imuMag.getY() , imuMag.getX() );
    if( magXYLength > 0.01 ){
      yawMeasured = -atan2( imuMag.getY()*cos( rollAngle ) - imuMag.getZ()*sin( rollAngle ) , imuMag.getX()*cos( pitchAngle ) + imuMag.getZ()*sin( pitchAngle ) );
      yawAngle = 0.7*yawAngle + 0.3*yawMeasured ;
      //Serial.print(" yaw Measured: ");Serial.print( yawMeasured );
    }

        // roll filter
    double accelMeasLength = magnitude( imuAccel.getY(), imuAccel.getZ() );
    double calibGain = 0.99;
    if( accelMeasLength > 0.1 ){
      rollMeasured = atan2( imuAccel.getY(), imuAccel.getZ() )  ;
      rollAngle = calibGain*rollAngle + (1-calibGain)*rollMeasured ;
      //Serial.print(" roll Measured: ");Serial.print( rollMeasured );
    }
      // pitch filter
    accelMeasLength = magnitude( imuAccel.getX(), imuAccel.getZ()*cos( rollAngle) + imuAccel.getY()*sin(rollAngle) );
    if( accelMeasLength > 0.1 ){

      pitchMeasured = -atan2( imuAccel.getX(), imuAccel.getZ()*cos( rollAngle ) + imuAccel.getY()*sin( rollAngle ));
      pitchAngle = calibGain*pitchAngle + (1-calibGain)*pitchMeasured ;
      //Serial.print(" pitch Measured: ");Serial.print( pitchMeasured );

    }

    pitch.setTotal( pitchAngle*cos(rollAngle) );//*cos(rollAngle) - yawAngle*sin(rollAngle) );
    roll.setTotal( rollAngle );
    yaw.setTotal( yawAngle );//*cos(rollAngle) - pitchAngle*sin(rollAngle) );
    //Serial.println(roll.getTotal()-rollTarget);
    //update pid inputs
    pitchInput = imuGyro.getY();
    rollInput = imuGyro.getX();
    yawInput = imuGyro.getZ();

    //pid calculate
    pitch.calculate(td);
    roll.calculate(td);
    yaw.calculate(td);

    Serial.println(pitchOutput);
    
    missionRun( td );

    /*
    double calibVal = 0.05;
    
    // predict gravity vector
    gravity.rotateZ( imuGyro.getZ()*td );
    gravity.rotateY( imuGyro.getY()*td );
    gravity.rotateX( imuGyro.getX()*td );

    calibVal = abs( gravity.getThetaDelta( imuAccel ) )/0.2 ;
    if( calibVal>0.8 ){ calibVal = 0.8; }
    //calibVal = 1 - calibVal;
    
    gravity.multiply( 1-calibVal );
    gravity.setX( gravity.getX() + calibVal*imuAccel.getX() );
    gravity.setY( gravity.getY() + calibVal*imuAccel.getY() );
    gravity.setZ( gravity.getZ() + calibVal*imuAccel.getZ() );    
    
    magneticHeading.multiply( 1-calibVal );
    magneticHeading.setX( magneticHeading.getX() + calibVal*imuMag.getX() );
    magneticHeading.setY( magneticHeading.getY() + calibVal*imuMag.getY() );
    magneticHeading.setZ( magneticHeading.getZ() + calibVal*imuMag.getZ() );
    */
    
    //Serial.println(abs( imuAccel.getLength() - gravityMagnitude ),DEC);
    //Serial.println(calibVal, DEC);
    
    //Serial.print(" pitch: ");Serial.print(pitch.getTotal()*180/pi, DEC);
    //Serial.print(" roll: ");Serial.print(roll.getTotal()*180/pi, DEC);
    //Serial.print(" yaw: ");Serial.print(yaw.getTotal()*180/pi, DEC);

    //Serial.print(" pid Roll: ");Serial.print( rollOutput );
    //Serial.print(" pid Pitch: ");Serial.print( pitchOutput );
    //Serial.print(" pid Yaw: ");Serial.print( yawOutput );
    //Serial.println("");
    //Serial.println( gravity.getZ() , DEC );
  }
  
}
