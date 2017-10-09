

static uint32_t test_timer = millis();
const uint32_t test_run_time = 360*1000; // test time in milliseconds
static bool test_done = false;

const int DeadSwitchPin = 6;

void idle_code(){
  motor_controller.setPidOn(false);
  motor_controller.setThrust(1000);
  motor_controller.updateMotors();
  
  return;
}

void running_code(){
  if(!test_done){
    
    if(millis()-test_timer<test_run_time){
      motor_controller.setPidOn(true);
      motor_controller.setThrust(1380);
    }else{ test_done = true; }
    
  }else{
    Serial.println("Done");
    motor_controller.setPidOn(false);
    motor_controller.setThrust(1000);
    test_done= true;
  }
  
  return;
}

void on_start(){
  pinMode(DeadSwitchPin, INPUT_PULLUP);

  delay(1000);
  Serial.println("Go");
  delay(1000);
  test_timer = millis();

  motor_controller.setSmoothThrust( true, 0,  0.01);
  
  return;
}

void mission_code(){
  if (!digitalRead(DeadSwitchPin)) {
    idle_code();
  }else{
    running_code();
  }
  
}



