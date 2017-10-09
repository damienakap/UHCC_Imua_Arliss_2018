
#include <SPI.h>
#include <SD.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

#include <Servo.h>
#include <math.h>

#include <Adafruit_GPS.h>
#define gpsSerial Serial1
Adafruit_GPS GPS(&gpsSerial);
#define GPSECHO  true

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

const double pi = 3.1415926535897932384626433832795;
const double r_earth = 6371; // km

double toRadians(double deg){
  return (deg*pi/(double)180.0);
}
int getSign(double f){
  if(f >= 0){ return 1;
  }else{    return -1;}
}
double getDist2D(double x, double y){
  return pow(pow(x,2)+pow(y,2),0.5);
}
double getDist3D(double x, double y, double z){
  return pow(pow(x,2)+pow(y,2)+pow(z,2),0.5);
}

class SDCARD{
  private:
    String log_file_name;
    int chipSelect;
  public:
  SDCARD(int chip, String lfn){
    chipSelect = chip;
    log_file_name = lfn;
  }

  void initial(){
    Serial.print("Initializing SD card...");
    if(!SD.begin(chipSelect))Serial.println("Card failed, or not present");
    Serial.println("card initialized.");
  }

  void setLogFile(String lfn){ log_file_name = lfn; }
  
  void logData(String dataString){
    File dataFile = SD.open( log_file_name, FILE_WRITE);
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.flush();
      dataFile.close();
      Serial.println(dataString);
    }else{
      Serial.println("error opening "+log_file_name+".");
    }
    return;
  }
  
};

SDCARD sdcard = SDCARD( 4, "log.txt");

class IMU{
  private:
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
    Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
    Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
    Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
    sensor_t sensor;
    sensors_event_t event;

    //imu info vars
    int accel_max;
    int accel_min;
    int accel_res;
    int gyro_max;
    int gyro_min;
    int gyro_res;
    int mag_max;
    int mag_min;
    int mag_res;
    int bmp_max;
    int bmp_min;
    int bmp_res;
    //imu values
    float accel_x;
    float accel_y;
    float accel_z;
    float accel_calib_x = 0.0;
    float accel_calib_y = 0.0;
    float accel_calib_z = 0.0;
    
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float gyro_calib_x = 0.0;
    float gyro_calib_y = 0.0;
    float gyro_calib_z = 0.0;
    
    float mag_x;
    float mag_y;
    float mag_z;
    float bmp_pressure;
    float bmp_temp;
    float seaLevelPressure;
    float altitude;
    
  public:
    void initial(){
      accel.getSensor(&sensor);
      accel_max = sensor.max_value;
      accel_min = sensor.min_value;
      accel_res = sensor.resolution;
      
      gyro.getSensor(&sensor);
      gyro_max = sensor.max_value;
      gyro_min = sensor.min_value;
      gyro_res = sensor.resolution;
      
      mag.getSensor(&sensor);
      mag_max = sensor.max_value;
      mag_min = sensor.min_value;
      mag_res = sensor.resolution;
    
      bmp.getSensor(&sensor);
      bmp_max = sensor.max_value;
      bmp_min = sensor.min_value;
      bmp_res = sensor.resolution;

      updateAccel();
      updateMag();
      updateGyro();
      updateBpm();
    }
    void updateAccel(){
      accel.getEvent(&event);
      accel_x = event.acceleration.x + accel_calib_x;
      accel_y = event.acceleration.y + accel_calib_y;
      accel_z = event.acceleration.z + accel_calib_z;
      return;
    }
    void updateMag(){
      mag.getEvent(&event);
      mag_x = event.magnetic.x;
      mag_y = event.magnetic.y;
      mag_z = event.magnetic.z;
      return;
    }
    void updateGyro(){
      gyro.getEvent(&event);
      gyro_x = event.gyro.x + gyro_calib_x;
      gyro_y = event.gyro.y + gyro_calib_y;
      gyro_z = event.gyro.z + gyro_calib_z;
      return;
    }
    void updateBpm(){
      bmp.getEvent(&event);
      bmp_pressure = event.pressure;
      bmp_temp; bmp.getTemperature(&bmp_temp);
      seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
      altitude = bmp.pressureToAltitude(seaLevelPressure,bmp_pressure,bmp_temp);
      return;
    }
    float getMagX(){return mag_x;}
    float getMagY(){return mag_y;}
    float getMagZ(){return mag_z;}
    float getAccelX(){return accel_x;}
    float getAccelY(){return accel_y;}
    float getAccelZ(){return accel_z;}
    float getGyroX(){return gyro_x;}
    float getGyroY(){return gyro_y;}
    float getGyroZ(){return gyro_z;}
    float getTemp(){return bmp_temp;}
    float getPress(){return bmp_pressure;}
    float getSeaPress(){return seaLevelPressure;}
    float getCurrentAltitude(){return altitude;}
  
};

IMU imu = IMU();

class NAVIGATION{
  private:
  IMU* myIMU;
  int ping_time = 1200; // ms
  bool fix;
  double latitude_deg;
  double longitude_deg;
  double latitude_deg_last;
  double longitude_deg_last;
  float angle;
  float speed_now; //knots
  double altitude; // cm
  double altitude_last;
  uint32_t timer;

  double target_latitude_deg = 40.8735733032;
  double target_longitude_deg = -119.1088027954;
  double target_delta_pos_x = 0.0;
  double target_delta_pos_y = 0.0;
  float target_angle = 0.0;
  double target_distance = 0.0;
  
  public:
  NAVIGATION(IMU* im, double targ_long, double targ_lat){
    myIMU = im;
    target_longitude_deg = targ_long;  target_latitude_deg = targ_lat;
  }
  void initial(){
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
    gpsSerial.println(PMTK_Q_RELEASE);
    timer = millis();
  }
  
  bool getGpsFix(){return fix;}
  double getGpsLat(){return latitude_deg;}
  double getGpsLong(){return longitude_deg;}
  float getGpsSpeed(){return speed_now;}
  float getGpsAngle(){return angle;}
  double getGpsAltitude(){return altitude;}
  
  double getTargDX(){return target_delta_pos_x;}
  double getTargDY(){return target_delta_pos_y;}
  double getTargDist(){return target_distance;}
  float getTargAngle(){return target_angle;}
  
  void updateGps(){
    if (timer > millis())  timer = millis();
    if (millis() - timer > ping_time) {
      timer = millis();
      char c = GPS.read();
      if (GPS.newNMEAreceived()){ if(!GPS.parse(GPS.lastNMEA()))return; }
      
      fix =         GPS.fix;
      if (fix){
        latitude_deg_last = latitude_deg;
        longitude_deg_last = longitude_deg;
        altitude_last = altitude;
        
        latitude_deg =  GPS.latitudeDegrees;
        longitude_deg = GPS.longitudeDegrees;
        angle =         GPS.angle;
        speed_now =     GPS.speed; //knots
        altitude =      GPS.altitude;

        target_delta_pos_x = r_earth*toRadians(target_latitude_deg-latitude_deg);
        target_delta_pos_y = r_earth*toRadians(target_longitude_deg-longitude_deg);
        target_distance = getDist2D( target_delta_pos_x, target_delta_pos_y );
        target_angle = atan2( target_delta_pos_x, target_delta_pos_y );
        if(target_angle>pi){ target_angle = -pi-pi+target_angle; }
        if(target_angle<-pi){ target_angle = pi+pi+target_angle; }
        
      }
       
    }
    return;
  }
};

NAVIGATION nav = NAVIGATION( &imu, -119.1088027954, 40.8735733032);

class PID{
  private:
    IMU* pidIMU;
    int axis = 0;
    float p_gain = 0.0;  //Gain setting for the P-controller.
    float i_gain = 0.0;  //Gain setting for the I-controller.
    float d_gain = 0.0;  //Gain setting for the D-controller.
    float pid_max = 200; //Maximum output of the PID-controller (+/-)

    float pid_out = 0;                 // pid output for axis
    float delta_angle_last = 0;        // last angle rate difference between gyro and input rate
    float set_angle = 0;               // in rads (set angle from center)
    float delta_rate_last = 0;
    float gyro_total_angle = 0;        // in rads (total angle from center)
    float accel_angle = 0;             // in rads (total angle from accelerometer)

    float correction_factor = 0.002;   // uses the accelorometer/magnetometer to correct gyro
    
  public:
  PID( IMU* sens, int a, float p, float i, float d, float maxi){
    pidIMU = sens;  axis = a;   pid_max = maxi;
    p_gain = p;   i_gain = i;   d_gain = d;
  }

  void updateAngle(float time_delta){
    if(axis==0){ // x-axis (roll)
      gyro_total_angle += (pidIMU->getGyroX()*time_delta);
      gyro_total_angle *= ((float)1.0-correction_factor);
      gyro_total_angle += ((float)atan2(pidIMU->getAccelY(),pidIMU->getAccelZ())*correction_factor);
    }
    if(axis==1){ // y-axis (pitch)
      gyro_total_angle += (pidIMU->getGyroY()*time_delta);
      gyro_total_angle *= ((float)1.0-correction_factor);
      gyro_total_angle += ((float)atan2(-pidIMU->getAccelX(),pidIMU->getAccelZ())*correction_factor);
    }
    if(axis==2){ // z-axis (yaw)
      gyro_total_angle += (pidIMU->getGyroZ()*time_delta);
      gyro_total_angle *= ((float)1.0-correction_factor);
      gyro_total_angle += ((float)atan2(-pidIMU->getMagY(),pidIMU->getMagX())*correction_factor);
    }
  }

  void setAngle(float angle){ set_angle = angle; }
  float getGyroAngle(){return gyro_total_angle;}
  
  void calculate(float time_delta, float t_axis){
    if(axis==0){ gyro_total_angle += t_axis * sin(pidIMU->getGyroZ()*time_delta);}
    if(axis==1){ gyro_total_angle -= t_axis * sin(pidIMU->getGyroZ()*time_delta);}
    float delta_angle = (gyro_total_angle - set_angle);
    float delta_rate = (delta_angle - delta_angle_last )/time_delta;

    float p_out = delta_rate * p_gain;
    float i_out = delta_angle * i_gain;
    float d_out = (delta_rate - delta_rate_last) * d_gain/time_delta;
    
    if(i_out>pid_max){i_out=pid_max;}
    if(i_out<-pid_max){i_out=-pid_max;}
    
    pid_out = (p_out + i_out + d_out);
    
    // clip yaw between +/- pid_max_yaw
    if(pid_out>pid_max){pid_out=pid_max;}
    if(pid_out<-pid_max){pid_out=-pid_max;}

    delta_rate_last = delta_rate;
    delta_angle_last = delta_angle;
    
    return;
  }

  float getOutput(){return pid_out;}
  
};

class QUADCOPTER_MOTOR_CONTROLlER{
  private:

    IMU* myIMU;
    PID PID_roll =  PID(  myIMU, 0,    12.0, 90.0, 0.14, 400);
    PID PID_pitch = PID(  myIMU, 1,    12.0, 90.0, 0.14, 400);
    PID PID_yaw =   PID(  myIMU, 2,    32.0, 0, 0, 200);

    bool pid_on = false;
    int pid_skip = 0;
    uint32_t pid_time_last = micros();
    float pid_update_delay = 4000;
    
    Servo ESC0;
    Servo ESC1;
    Servo ESC2;
    Servo ESC3;

    float thrust = 1000;
    float target_thrust = 1000;
    bool smoothThrust = false;
    int smoothThrustType = 0;
    float smoothThrustFactor = 0.0;

    int batVoltagePin = A4;
    float r1 = 2.55;
    float r2 = 1;
    float diodeComp = 0.6;
    float max_voltage = 12.6;
    float batVoltage = 0.0;
    
    void updateBat(){
      batVoltage = ((r1+r2)*analogRead(batVoltagePin)*3.3/1023/r2)+diodeComp;
      if(batVoltage <= diodeComp+0.1) batVoltage = 0;
    }
    float getBatComp(){
      if(batVoltage <= diodeComp+0.1){ return 0.0;
      }else{ return ( 1000-(1000*batVoltage/max_voltage) ); }
    }
    int thrustClip(float tin){
      int t = (int)tin;
      if(t>2000){t=2000;} if(t<1000){t=1000;}
      return t;
    }
  public:
    QUADCOPTER_MOTOR_CONTROLlER(IMU* im, int fl_pin, int fr_pin, int bl_pin, int br_pin){
      myIMU = im;
      ESC0.attach(fl_pin); ESC1.attach(fr_pin); ESC2.attach(bl_pin); ESC3.attach(br_pin);
    }
    void setRoll(float roll){   PID_roll.setAngle(roll);    pid_skip=0; return;}
    void setPitch(float pitch){ PID_pitch.setAngle(pitch);  pid_skip=0; return;}
    void setYaw(float yaw){     PID_yaw.setAngle(yaw);      pid_skip=0; return;}
    
    void setThrust(float tin){
      target_thrust = thrustClip(tin);
      return;
    }
    void setPercentThrust(float per){ 
      target_thrust = thrustClip( 10*(100.0+per)) ;
      return;
    }
    void setSmoothThrust(bool st, int stt,  float stf){
      smoothThrust = st; smoothThrustType = stt; smoothThrustFactor = stf;
      return;
    }
    void setPidOn(bool on){ pid_skip=0; pid_on = on; return;}
    
    void updateMotors(){
      float pid_time_delta = (micros() - pid_time_last);
      if(pid_time_delta >= 4000){

        pid_time_delta /= 1000000;
        pid_time_last = micros();
        
        myIMU->updateAccel();
        myIMU->updateGyro();
        myIMU->updateMag();
        
        PID_roll.updateAngle(pid_time_delta);
        PID_pitch.updateAngle(pid_time_delta);
        PID_yaw.updateAngle(pid_time_delta);
        
        float roll = PID_roll.getGyroAngle();
        float pitch = PID_pitch.getGyroAngle();
        PID_roll.calculate( pid_time_delta, pitch );
        PID_pitch.calculate( pid_time_delta, roll );      
        PID_yaw.calculate( pid_time_delta, 0.0 );
        
        updateBat();
        if(smoothThrust){
          if(smoothThrustType){
            thrust += (target_thrust-thrust)*(1-smoothThrustFactor)/pid_time_delta;
          }else{
            int sign = getSign(target_thrust-thrust);
            thrust += sign*1000*(smoothThrustFactor)/pid_time_delta;
            if(sign<0 && thrust < target_thrust){thrust = target_thrust;}
            if(sign>0 && thrust > target_thrust){thrust = target_thrust;}
          }
        }else{ thrust=target_thrust; }
        
        if(pid_on){
          if(pid_skip < 2){
            pid_skip += 1;
            int t = thrustClip( thrust + getBatComp() );
            ESC0.writeMicroseconds( t );
            ESC1.writeMicroseconds( t );
            ESC2.writeMicroseconds( t );
            ESC3.writeMicroseconds( t );
          }else{
            int t = thrustClip( thrust + PID_pitch.getOutput() - PID_roll.getOutput() - PID_yaw.getOutput() + getBatComp() );
            ESC0.writeMicroseconds( t );
            t =     thrustClip( thrust + PID_pitch.getOutput() + PID_roll.getOutput() + PID_yaw.getOutput() + getBatComp() );
            ESC1.writeMicroseconds( t );
            t =     thrustClip( thrust - PID_pitch.getOutput() - PID_roll.getOutput() + PID_yaw.getOutput() + getBatComp() );
            ESC2.writeMicroseconds( t );
            t =     thrustClip( thrust - PID_pitch.getOutput() + PID_roll.getOutput() - PID_yaw.getOutput() + getBatComp() );
            ESC3.writeMicroseconds( t );
          }
        }else{
          ESC0.writeMicroseconds( thrust );
          ESC1.writeMicroseconds( thrust );
          ESC2.writeMicroseconds( thrust );
          ESC3.writeMicroseconds( thrust );
        }
      }
      return;
    }

    void initial(){
      setPidOn(false);
      
      double time_last = millis();
      setThrust(2000);
      while(millis()-time_last < 6000){ updateMotors(); }
      
      time_last = millis();
      setThrust(1000);
      while(millis()-time_last < 10000){ updateMotors(); }
      
      time_last = millis();
      setThrust(1100);
      while(millis()-time_last < 100){ updateMotors(); }
      
      setThrust(1000);
      return;
    }
    
};

QUADCOPTER_MOTOR_CONTROLlER motor_controller = QUADCOPTER_MOTOR_CONTROLlER( &imu, 10, 11, 12, 13);

void setup(){
  Serial.begin(112500);
  while (!Serial) {}
  imu.initial();
  nav.initial();
  motor_controller.initial();
  
  on_start();
  
}

void loop() {
  nav.updateGps();
  motor_controller.updateMotors();
  
  mission_code();
  
}



