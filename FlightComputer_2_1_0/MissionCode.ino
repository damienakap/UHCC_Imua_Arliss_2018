
int deadTriggerPin = 9;

static int state = 0;
static long stateTimer = millis();
static boolean stateInit = false;

static double batVoltageAv = 12.6;

void missionInit(){
  pinMode( deadTriggerPin, INPUT_PULLUP );
}

void missionRun( double td ){
  switch( state ){
    case 1:         // launch
      launchState();
      break;
    case 2:         // hover
      hoverState();
      break;
    case 3:         // navigation state
      navState();
      break;
    case 4:         //  done state
      doneState();
      break;
    default:        // wait state
      waitState();
  }
}

void launchState(){
  if( !stateInit ){
    Serial.println("Launch State");
    stateTimer = millis();
    stateInit = true;
  }
  
  thrust = 50+hoverThrust;

  setPidThrust(thrust); 

  if( digitalRead(deadTriggerPin) ){
    setState(4);
    return;
  }
  if(stateTimer > millis()){ stateTimer = millis(); }
  if( millis()-stateTimer >= 1000 ){
    setState(2);
    return;
  }
}

void hoverState(){
  if( !stateInit ){
    Serial.println("Hover State");
    stateTimer = millis();
    stateInit = true;
  }

  thrust = hoverThrust;
  
  setPidThrust(thrust);
  //Serial.println(rollOutput);

  if( digitalRead(deadTriggerPin) ){
    setState(4);
    return;
  }
  if(stateTimer > millis()){ stateTimer = millis(); }
  if( millis()-stateTimer >= 50000 ){
    setState(3);
    return;
  }
}

void navState(){
  if( !stateInit ){
    Serial.println("Navigation State");
    stateTimer = millis();
    stateInit = true;
  }
  
  thrust = hoverThrust-100;
  
  setPidThrust(thrust);
  //Serial.println(rollOutput);

  if( digitalRead(deadTriggerPin) ){
    setState(4);
    return;
  }
  if(stateTimer > millis()){ stateTimer = millis(); }
  if( millis()-stateTimer >= 5000 ){
    setState(4);
    return;
  }
}

void doneState(){
  if(!stateInit){
    stateTimer = millis();
    stateInit = true;
  }
  if(stateTimer > millis()){ stateTimer = millis(); }
  if(millis()-stateTimer > 1000){
    Serial.println("Done State");
    stateTimer = millis();
  }
  setThrust( 1000, 1000, 1000, 1000 );
}

void waitState(){
  if( !digitalRead(deadTriggerPin) ){
    setState(1);
  }else{
    Serial.println("Wait State");
    setThrust( 1000, 1000, 1000, 1000 );
  }
}

void setState( int s ){
  state = s;
  thrust = 1000;
  stateInit = false;
}

double getVoltageScalar(){
  //Serial.println(getBatteryVoltage());
  batVoltageAv = 0.9*batVoltageAv + 0.1*getBatteryVoltage();
  double s = ( 2-batVoltageAv/12.7 );
  if(s<1){s = 1;}
  if( s>1.3846 ){ s>1.3846; }
  return s;
}

void setPidThrust( double t ){
  
  double batVoltScalar = getVoltageScalar();
  t += 100*magnitude(  1-abs(cos(rollAngle)), 1-abs(cos(pitchAngle))  );
  
  double fl = (t + pitchOutput - rollOutput - yawOutput)*batVoltScalar;
  double fr = (t + pitchOutput + rollOutput + yawOutput)*batVoltScalar;
  double bl = (t - pitchOutput - rollOutput + yawOutput)*batVoltScalar;
  double br = (t - pitchOutput + rollOutput - yawOutput)*batVoltScalar;

  if(fl>900){ fl = 900; }
  if(fr>900){ fr = 900; }
  if(bl>900){ bl = 900; }
  if(br>900){ br = 900; }
  
  setThrust( fl + 1000, fr + 1000, bl + 1000, br + 1000);
}

void setThrust( int fl, int fr, int bl, int br ){
  ESC0.writeMicroseconds( fl );
  ESC1.writeMicroseconds( fr );
  ESC2.writeMicroseconds( bl );
  ESC3.writeMicroseconds( br );
}

