void motor_setup()
{
  motA.attach(PE0, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motB.attach(PE1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motC.attach(PE2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  myservoX.attach(PE3);
  myservoY.attach(PE4);
  myservoA.attach(PE15);
  myservoB.attach(PE14);
}

void update_motor()
{ 
  if(ch5==0)
  {
    error_roll=error_pitch=error_yaw=integral_error_roll=integral_error_pitch=integral_error_yaw=derivative_error_roll=derivative_error_pitch=derivative_error_yaw=last_error_roll=last_error_pitch=last_error_yaw=0;
    error_roll1=error_pitch1=error_yaw1=integral_error_roll1=integral_error_pitch1=integral_error_yaw1=derivative_error_roll1=derivative_error_pitch1=derivative_error_yaw1=last_error_roll1=last_error_pitch1=last_error_yaw1=0;
    error_alt=error_alt1=integral_error_alt=integral_error_alt1=derivative_error_alt=derivative_error_alt1=last_error_alt=last_error_alt1=0;
    error_pos=error_vel=integral_error_pos=integral_error_vel=derivative_error_pos=derivative_error_vel=last_error_pos=last_error_vel=0;
    motA.writeMicroseconds(1000);
    motB.writeMicroseconds(1000);
    motC.writeMicroseconds(1000);
    myservoX.write(servoAngleInit1);
    myservoY.write(servoAngleInit2);
    myservoA.write(servoAngleInitA);
    myservoB.write(servoAngleInitB);
    armStatus = 0;
    pitch_transisi = 0;
  }

  if(ch5==1 && throttle_channel<1000){armStatus=1;}

  if(armStatus==1 && ch6 ==0){
    controlDrone();
    pulse_escx = 0;
    pulse_escy = 0;
    transisi_servo = 0;
    fwhead1 = 0;
    statusAktif = 0;
    pitch_transisi = 0;
    motA.writeMicroseconds(pulse_length_esc1);
    motB.writeMicroseconds(pulse_length_esc2);
    motC.writeMicroseconds(pulse_length_esc3);
    // myservoX.write(pulse_length_servo1);
    // myservoY.write(pulse_length_servo2);
    myservoA.write(96);
    myservoB.write(81);
    if((pitch_deg<-40 && pitch_deg>40)||(roll_deg<-40 && roll_deg>40)){
      myservoX.write(servoAngleInit1);
      myservoY.write(servoAngleInit2);
    }
    else{
      myservoX.write(pulse_length_servo1);
      myservoY.write(pulse_length_servo2);
    }
  }

  if(armStatus==1 && ch6==1){
      controlPlane();
      controlDrone();
      // yawtrans = ((yaw_control)*2) + (gz*(0.8));
      // yawtrans=constrain(yawtrans,-50,50);
      // pulse_length_esc1 += yawtrans;
      // pulse_length_esc2 -= yawtrans;

      transisiTime_current = millis();
      if((transisiTime_current - transisiTime_previous >= 50)){pitch_transisi += 1; transisiTime_previous = transisiTime_current;}
      if(pitch_transisi > 25){pitch_transisi = 25;} 
      if(pitch_transisi == 25){pulse_escx = 200; pulse_escy = 200;}
      pitch_transisi2 = (100-pitch_transisi)-(yawControl*1); 
      pitch_transisi1 = (pitch_transisi+78)-(yawControl*1);
      pitch_transisi1 = constrain(pitch_transisi1, 78, 113);
      pitch_transisi2 = constrain(pitch_transisi2, 65, 100);
      // if(pitch_transisi > 40){pitch_transisi2 = 135;}
      // if(pitch_transisi==40){fwhead1 =1;}
      motA.writeMicroseconds(pulse_length_esc1);
      motB.writeMicroseconds(pulse_length_esc2);
      motC.writeMicroseconds(pulse_length_esc3);
      myservoX.write(pitch_transisi1);
      myservoY.write(pitch_transisi2);
      myservoA.write(Servo1);
      myservoB.write(Servo2);
    //  if(fwhead1 == 0){ 
    //   if(statusAktif == 0){
    //   transisiTime_current = millis();
    //   statusAktif = 1;
    //   }
    //  if(statusAktif == 1 && (millis()-transisiTime_current > 4000)){
    //   fwhead1 = 1;
    //   statusAktif = 0;
    //  }
    //   roll_input=0;
    //   pitch_input=0;
    //   controlPlane();
    //   controlDrone();
    //   // yawControl   = ((yaw_control*-1)*6) + (gyroZ_filt*(2.8*-1));
    //   // pulse_length_esc1 += yawControl;
    //   // pulse_length_esc2 -= yawControl;
      
    //   motA.writeMicroseconds(pulse_length_esc1); 
    //   motB.writeMicroseconds(pulse_length_esc2);
    //   motC.writeMicroseconds(pulse_length_esc3);
    //   myservoX.write(105);
    //   myservoY.write(105);
    //   myservoA.write(Servo1);
    //   myservoB.write(Servo2);
      
    //  }

  // if(fwhead1 == 1){
  //   if(statusAktif == 0){
  //   transisiTime_current1 = millis();
  //   transisi_servo = 1800;
  //   statusAktif = 1;
  //   motA.writeMicroseconds(transisi_servo);
  //   motB.writeMicroseconds(transisi_servo);
  //   }
  //   if(statusAktif == 1 && (millis()-transisiTime_current1 > 2000)){
  //    motA.writeMicroseconds(throttle_channel);
  //    motB.writeMicroseconds(throttle_channel);
  //    statusAktif = 1;}
  //    controlPlane();
  //    motC.writeMicroseconds(1000);
  //    myservoX.write(59);
  //    myservoY.write(61);
  //    myservoA.write(Servo1);
  //    myservoB.write(Servo2);
  //   }
  }
  if(armStatus==1 && ch6==2){
    controlPlane();
    transisiTime_current = millis();
    // if((transisiTime_current - transisiTime_previous >= 50)){pitch_transisi += 1; transisiTime_previous = transisiTime_current;}
    // if(pitch_transisi > 90){pitch_transisi = 90;}
    
    // pitch_transisi1 = 106-pitch_transisi; 
    // pitch_transisi2 = pitch_transisi+80;
    // if(pitch_transisi > 80){pitch_transisi2 = 170;}
    motA.writeMicroseconds(throttle_channel);
    motB.writeMicroseconds(throttle_channel);
    motC.writeMicroseconds(1000);
    myservoX.write(180);
    myservoY.write(0);
    myservoA.write(Servo1);
    myservoB.write(Servo2);
  }
}