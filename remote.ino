// void remote_init()
// {
//   pinMode(PE7, INPUT_PULLUP);
//   pinMode(PE8, INPUT_PULLUP);
//   pinMode(PE9, INPUT_PULLUP);
//   pinMode(PE10, INPUT_PULLUP);
//   pinMode(PE11, INPUT_PULLUP);
//   pinMode(PE12, INPUT_PULLUP);
//   pinMode(PE13, INPUT_PULLUP);
//   pinMode(PE14, INPUT_PULLUP);
// //  pinMode(41, INPUT_PULLUP);
  
//   attachInterrupt(PE7, rollris,  RISING);
//   attachInterrupt(PE8, pitchris, RISING);
//   attachInterrupt(PE9, throtris, RISING);
//   attachInterrupt(PE10, yawris, RISING);
//   attachInterrupt(PE11, ch5ris, RISING);
//   attachInterrupt(PE12, ch6ris, RISING);
//   attachInterrupt(PE13, ch7ris, RISING); 
//   attachInterrupt(PE14, ch8ris, RISING);
// //  attachInterrupt(41, ch9ris, RISING);  
// }
// //
// //========== roll channel ==========
// void rollris() 
// {
//   attachInterrupt(PE7, rollfall, FALLING);
//   channel1 = micros();
// }
 
// void rollfall() 
// {
//   attachInterrupt(PE7, rollris, RISING);
//   roll_channel = micros()-channel1 - 20;
// }

// //========== pitch channel ==========
// void pitchris() 
// {
//   attachInterrupt(PE8, pitchfall, FALLING);
//   channel2 = micros();
// }
 
// void pitchfall() 
// {
//   attachInterrupt(PE8, pitchris, RISING);
//   pitch_channel = micros()-channel2 + 25;
// }

// //========== throttle channel ==========
// void throtris() 
// {
//   attachInterrupt(PE9, throtfall, FALLING);
//   channel3 = micros();
// }
 
// void throtfall() 
// {
//   attachInterrupt(PE9, throtris, RISING);
//   throttle_channel = micros()-channel3;
// }

// //========== yaw channel ==========
// void yawris() 
// {
//   attachInterrupt(PE10, yawfall, FALLING);
//   channel4 = micros();
// }
 
// void yawfall() 
// {
//   attachInterrupt(PE10, yawris, RISING);
//   yaw_channel = micros()-channel4 + 15;
// } 

// //========== ch5 channel ==========
// void ch5ris() 
// {
//   attachInterrupt(PE11, ch5fall, FALLING);
//   channel5 = micros();
// }
 
// void ch5fall() 
// {
//   attachInterrupt(PE11, ch5ris, RISING);
//   ch5_channel = micros()-channel5;
// }

// //========== ch6 channel ==========
// void ch6ris() 
// {
//   attachInterrupt(PE12, ch6fall, FALLING);
//   channel6 = micros();
// }
 
// void ch6fall() 
// {
//   attachInterrupt(PE12, ch6ris, RISING);
//   ch6_channel = micros()-channel6;
// }

// //========== ch7 channel ==========
// void ch7ris() 
// {
//   attachInterrupt(PE13, ch7fall, FALLING);
//   channel7 = micros();
// }
 
// void ch7fall() 
// {
//   attachInterrupt(PE13, ch7ris, RISING);
//   ch7_channel = micros()-channel7;
// }

// //========== ch8 channel ==========
// void ch8ris() 
// {
//  attachInterrupt(PE14, ch8fall, FALLING);
//  channel8 = micros();
// }

// void ch8fall() 
// {
//  attachInterrupt(PE14, ch8ris, RISING);
//  ch8_channel = micros()-channel8;
// }

void mapremote()
{
  elrsupdate();
  roll_input = 0;
  if (roll_channel > 1510){roll_input=roll_channel-1510;}else if(roll_channel < 1490){roll_input=roll_channel-1490;} roll_input =constrain(roll_input,-511,510); roll_input/=23;roll_input1=roll_input;
  pitch_input = 0;
  if (pitch_channel > 1511){pitch_input=pitch_channel-1511;}else if(pitch_channel < 1490){pitch_input=pitch_channel-1490;} pitch_input =constrain(pitch_input,-512,511); pitch_input/=25;pitch_input1=pitch_input*1.3;
  yaw_input = 0;
  if (yaw_channel > 1518){yaw_input=yaw_channel-1518;}else if(yaw_channel < 1497){yaw_input=yaw_channel-1497;} yaw_input =constrain(yaw_input,-510,500); yaw_input/=8;
  if(ch5_channel<1600){ch5=0;}else if(ch5_channel>1600){ch5=1;}
  if(ch6_channel<1400){ch6=0;}else if(ch6_channel>1400 && ch6_channel<1600){ch6=1;}else if(ch6_channel>1600){ch6=2;}
//  if(throttle_channel <= 1450 || throttle_channel >= 1530){alt_mode=0;} else if(throttle_channel >= 1450 && throttle_channel <= 1530){alt_mode=1;}
  // if(roll_channel <= 1400 || roll_channel >= 1600 || pitch_channel <= 1400 || pitch_channel >= 1600){pst_mode=0;}else if(roll_channel > 1400 && roll_channel < 1600 && pitch_channel > 1400 && pitch_channel < 1600){pst_mode=1;}
  if(ch7_channel<1400){ch7=0;}if(ch7_channel>1400){ch7=1;}
  if(ch7_channel<1600){head_mode=0;}if(ch7_channel>1600){head_mode=1;}
  if(ch8_channel<1400){ch8=0;}if(ch8_channel>1400){ch8=1;}
}

void SerialEvent() 
{
  while (Serial1.available()){ inChar = Serial1.read();
  
    //========== trimming ==========
//    initial trimm ===============
  //  if (inChar == 'z'){ servoAngleInitA -= 1;}
  //  if (inChar == 'x'){ servoAngleInitA += 1;}
  //  if (inChar == 'c'){ servoAngleInitB -= 1;}
  //  if (inChar == 'v'){ servoAngleInitB += 1;}

//    //auto trimm ==========
  //  if (inChar == 'u'){ servo1_up += 1;servo1_down += 1;servo2_up += 1;servo2_down += 1;}
  //  if (inChar == 'y'){ servo1_up -= 1;servo1_down -= 1;servo2_up -= 1;servo2_down -= 1;}
  //  if (inChar == 'i'){ servo1_up += 1;servo1_down += 1;servo2_up -= 1;servo2_down -= 1;}
  //  if (inChar == 'o'){ servo1_up -= 1;servo1_down -= 1;servo2_up += 1;servo2_down += 1;}
   if (inChar == 'u'){ servo1_up1 += 1;servo1_down1 += 1;}
   if (inChar == 'y'){ servo1_up1 -= 1;servo1_down1 -= 1;}
   if (inChar == 'i'){ servo2_up1 += 1;servo2_down1 += 1;}
   if (inChar == 'o'){ servo2_up1 -= 1;servo2_down1 -= 1;}

    //========== gain tunning ==========
//    //roll gain feedback ==========
    //  if (inChar == 'q'){ Kp_roll += 0.1;}
    //  if (inChar == 'w'){ Kp_roll -= 0.1;}
    //  if (inChar == 'e'){ Ki_roll += 0.001;}
    //  if (inChar == 'r'){ Ki_roll -= 0.001;}
    //  if (inChar == 't'){ Kd_roll += 0.01;}
    //  if (inChar == 'y'){ Kd_roll -= 0.01;}
    //  if (inChar == 'u'){ Kp_roll2 += 0.01;}
    //  if (inChar == 'i'){ Kp_roll2 -= 0.01;}
   
//    //pitch gain feedback ==========
    //  if (inChar == 'a'){ Kp_pitch += 0.1;}
    //  if (inChar == 's'){ Kp_pitch -= 0.1;}
    //  if (inChar == 'd'){ Ki_pitch += 0.01;}
    //  if (inChar == 'f'){ Ki_pitch -= 0.01;}
    //  if (inChar == 'g'){ Kd_pitch += 0.01;}
    //  if (inChar == 'h'){ Kd_pitch -= 0.01;}
    //  if (inChar == 'j'){ Kp_pitch2 += 0.01;}
    //  if (inChar == 'k'){ Kp_pitch2 -= 0.01;}

    //  if (inChar == 'z'){ Kp_pos += 0.01;}
    //  if (inChar == 'x'){ Kp_pos -= 0.01;}
    //  if (inChar == 'c'){ Kp_vel += 0.01;}
    //  if (inChar == 'v'){ Kp_vel -= 0.01;}
////    
////    //yaw gain feedback ==========
    // if (inChar == 'z'){ gain_yaw += 0.01;}
    // if (inChar == 'x'){ gain_yaw -= 0.01;}
    // if (inChar == 'c'){ gain_vel_yaw += 0.001;}
    // if (inChar == 'v'){ gain_vel_yaw -= 0.001;}
//    if (inChar == 'b'){ Kd[YAW] += 0.001;}
//    if (inChar == 'n'){ Kd[YAW] -= 0.001;}

// latitude
  //  if (inChar == 'q'){ gain_latitude += 0.1;}
  //  if (inChar == 'w'){ gain_latitude -= 0.1;}
  //  if (inChar == 'e'){ gain_vel_lat += 0.01;}
  //  if (inChar == 'r'){ gain_vel_lat -= 0.01;}

  //   // longitude
  //  if (inChar == 'a'){ gain_longitude += 0.1;}
  //  if (inChar == 's'){ gain_longitude -= 0.1;}
  //  if (inChar == 'd'){ gain_vel_long += 0.01;}
  //  if (inChar == 'f'){ gain_vel_long -= 0.01;}

  //   //altitude
  //  if (inChar == 'z'){ gain_altitude += 0.1;}
  //  if (inChar == 'x'){ gain_altitude -= 0.1;}
  //  if (inChar == 'c'){ gain_vel_z += 0.01;}
  //  if (inChar == 'v'){ gain_vel_z -= 0.01;}
////
//  if (inChar == 'u'){ pulse_esc11 += 10;}
//  if (inChar == 'i'){ pulse_esc11 -= 10;}
//   if (inChar == 'o'){ pulse_esc44 += 10;}
//   if (inChar == 'p'){ pulse_esc44 -= 10;}
//    
  //  if (inChar == 'l'){ danger = 1;}
  //  if (inChar == 'k'){ darurat = 1;}
////    
//  ===============IMU on===============================
    //  if (inChar == 'u'){ last_pitch1 += 0.1;}
    //  if (inChar == 'i'){ last_pitch1 -= 0.1;}
    //  if (inChar == 'o'){ last_roll += 0.1;}
    //  if (inChar == 'p'){ last_roll -= 0.1;}
////    
////    if (inChar == 'z'){ servo1_up += 1;servo1_down += 1;}
////    if (inChar == 'x'){ servo1_up -= 1;servo1_down -= 1;}
////    if (inChar == 'm'){ servo2_up += 1;servo2_down += 1;}
////    if (inChar == 'n'){ servo2_up -= 1;servo2_down -= 1;}
   }
}
