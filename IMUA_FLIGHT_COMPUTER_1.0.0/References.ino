
/*

basic vars:
  pi : 
  r_earth : (in km)
###########################

Basic Functions:
  double toRadians( double deg ) : Converts degrees to radians.
  int getSign( double val ) : returns the sign of the input value as a 1 or -1.
  double getDist2D(double x, double y) : returns the length of a vecor of components x and y.
  double getDist3D(double x, double y, double z) : returns the length of a vecor of components x, y and z.
###########################

Class Objects:
  Notes:
    calling object function :
      object.function( v1, v2, ... , vn);
  ###########################
  
  SDCARD:
    object : motor_controller
    
    void setLogFile(String log_file_name) : set the log file of the SDCARD object.
    void logData(String dataString) : saves the data string to the log file.
  ###########################
  
  IMU:
    object : imu
    
    void updateAccel() : updates the accelerometer values.
    void updateMag() : updates the magnetometer values.
    void updateGyro() : updates the gyrometer values.
    void updateBpm() : updates the barrometer values.

    float getMagX() : returns the x component of Earth's magnetic field relative to the IMU in uT.
    float getMagY() : returns the y component of Earth's magnetic field relative to the IMU in uT.
    float getMagZ() : returns the z component of Earth's magnetic field relative to the IMU in uT.
    float getAccelX() : returns the x component of Earth's gravity relative to the IMU in m/s^2.
    float getAccelY() : returns the y component of Earth's gravity relative to the IMU in m/s^2.
    float getAccelZ() : returns the z component of Earth's gravity relative to the IMU in m/s^2.
    float getGyroX() : returns the x component of the IMU's instantaneous rotation rate in rads/s.
    float getGyroY() : returns the y component of the IMU's instantaneous rotation rate in rads/s.
    float getGyroZ() : returns the z component of the IMU's instantaneous rotation rate in rads/s.
    float getTemp() : returns the temperature of the atmosphere degC.
    float getPress() : returns the pressure of the atmosphere hPa.
    float getSeaPress() : returns the pressure at sealevel hPa.
    float getCurrentAltitude() : returns the altitude in meters.
  ###########################
  
  NAVIGATION:
    object : nav
    
    bool getGpsFix() : returns true when the gps is recieving accurate satillite data.
    double getGpsLat()  : return the modules' current latitude position in degrees.
    double getGpsLong()  : return the modules' current longitude position in degrees.
    float getGpsSpeed() : get the modules' speed relative to the last gps position.
    float getGpsAngle() : get the modules' direction of motion based on the last gps position.
    # double getGpsAltitude() : returns the current altitude in cm?
    
    double getTargDX() : returns the modules x position difference from the target position.
    double getTargDY() : returns the modules y position difference from the target position.
    double getTargDist() : returns the modules distance from the target.
    float getTargAngle() : returns the angular direction of the target relative to the IMU's orientation.
  ###########################
  
  QUADCOPTER_MOTOR_CONTROLlER:
    object : motor_controller
    
    void setRoll(float roll)  : sets the roll angle in radians of the quadcopter.
    void setPitch(float pitch)  : sets the pitch angle in radians of the quadcopter.
    void setYaw(float yaw)  : sets the yaw angle in radians of the quadcopter.
    
    void setThrust(float trust) : sets the target thrust of the motors (1000 - 2000).
    void setPercentThrust : sets the target percent thrust of the motors (0 - 100)%.
    void setSmoothThrust(bool on, int type,  float factor) :
      sets the thrust smoothing to slowly increase/decrease the motor thrust to the target thrust.
      inputs:
        on : true/false
        type : 0-constant increase or 1-linear increase
        factor : sets the smooth thrust increase per second
          for type=0 : value 0-1.0 corresponding to raw thrust 0-1000
          for type=1 : value 0-1.0 corresponding to a percent of the difference between the
            current thrust to the target thrust. ( factor * (targ_thrust - cur_thrust) )
    void setPidOn(bool on) : turn on/of the PID controller correction (stabilization).
  ###########################
  
 */


 

