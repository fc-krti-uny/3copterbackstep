void controlDrone(){
  currentTime_pid=millis();
  if(currentTime_pid-previousTime_pid>9){
  dt=(currentTime_pid-previousTime_pid);
//==============================++++++++++++++++++++++++++++++++++++++++++++======================Virtual PID==============================================================
  if(ch6!=0){roll_input=pitch_input=yaw_input=0;}
  error_roll1=(roll_input+target_roll)-roll_deg;
  error_pitch1=(pitch_input*-1+target_pitch*-1)-pitch_deg;
  error_yaw1=(yaw_input*-1)-yaw_control;
  error_alt1=alt_input-altitude_cm;
  error_pos=0-distance_cm;

  integral_error_roll1+=error_roll1*0.01;
  integral_error_pitch1+=error_pitch1*0.01;
  integral_error_yaw1+=error_yaw1*0.01;
  integral_error_alt1+=error_alt1*0.01;
  integral_error_pos+=error_pos*0.01;

  derivative_error_roll1=(error_roll1-last_error_roll1)/0.01;
  derivative_error_pitch1=(error_pitch1-last_error_pitch1)/0.01;
  derivative_error_yaw1=(error_yaw1-last_error_yaw1)/0.01;
  derivative_error_alt1=(error_alt1-last_error_alt1)/0.01;
  derivative_error_pos=(error_pos-last_error_pos)/0.01;

  Proll1=Kp_roll*error_roll1;                 Ppitch1=Kp_pitch*error_pitch1;                  Pyaw1=Kp_yaw*error_yaw1;                Palt1=Kp_alt*error_alt1;                    Ppos=Kp_pos*error_pos;
  Iroll1=Ki_roll*integral_error_roll1;        Ipitch1=Ki_pitch*integral_error_pitch1;         Iyaw1=Ki_yaw*integral_error_yaw1;       Ialt1=Ki_alt*integral_error_alt1;           Ipos=Ki_pos*integral_error_pos;
  Droll1=Kd_roll*(derivative_error_roll1);    Dpitch1=Kd_pitch*(derivative_error_pitch1);     Dyaw1=Kd_yaw*(derivative_error_yaw1);   Dalt1=Kd_alt*(derivative_error_alt1);       Dpos=Kd_pos*(derivative_error_pos);
  last_error_roll1=error_roll1;               last_error_pitch1=error_pitch1;                 last_error_yaw1=error_yaw1;             last_error_alt1=error_alt1;                 last_error_pos=error_pos;

  PID_virtual_roll = Proll1 + Iroll1 + Droll1;   PID_virtual_pitch = Ppitch1 + Ipitch1 + Dpitch1;   PID_virtual_yaw = Pyaw1 + Iyaw1 + Dyaw1;   PID_virtual_alt = Palt1 + Ialt1 + Dalt1;  PID_virtual_pos = Ppos + Ipos + Dpos;
  PID_virtual_pos=abs(PID_virtual_pos);
  PID_virtual_pos=constrain(PID_virtual_pos,0,500);

//=====================================================Actual PID===============================================================
  error_roll=PID_virtual_roll-gx;
  error_pitch=PID_virtual_pitch-(gy*-1);
  error_yaw=(PID_virtual_yaw)-gz;
  error_alt=PID_virtual_alt-getAltitudeDerivative();
  error_vel=(PID_virtual_pos*-1*0)-gps_speed;//gps.speed_cmps();;

  integral_error_roll+=error_roll*0.01;
  integral_error_pitch+=error_pitch*0.01;
  integral_error_yaw+=error_yaw*0.01;
  integral_error_alt+=error_alt*0.01;
  integral_error_vel+=error_vel*0.01;

  derivative_error_roll=(error_roll-last_error_roll)/0.01;
  derivative_error_pitch=(error_pitch-last_error_pitch)/0.01;
  derivative_error_yaw=(error_yaw-last_error_yaw)/0.01;
  derivative_error_alt=(error_alt-last_error_alt)/0.01;
  derivative_error_vel=(error_alt-last_error_vel)/0.01;

  Proll=Kp_roll2*error_roll;                 Ppitch=Kp_pitch2*error_pitch;                  Pyaw=Kp_yaw2*error_yaw;                Palt=Kp_alt2*error_alt;                Pvel=Kp_vel*error_vel;
  Iroll=Ki_roll2*integral_error_roll;        Ipitch=Ki_pitch2*integral_error_pitch;         Iyaw=Ki_yaw2*integral_error_yaw;       Ialt=Ki_alt2*integral_error_alt;       Ivel=Ki_vel*integral_error_vel;
  Droll=Kd_roll2*(derivative_error_roll);    Dpitch=Kd_pitch2*(derivative_error_pitch);     Dyaw=Kd_yaw2*(derivative_error_yaw);   Dalt=Kd_alt2*(derivative_error_alt);   Dvel=Kd_vel*(derivative_error_vel);
  last_error_roll=error_roll;                last_error_pitch=error_pitch;                  last_error_yaw=error_yaw;              last_error_alt=error_alt;              last_error_vel=error_vel;

  PID_value_roll = Proll + Iroll + Droll;   PID_value_pitch = Ppitch + Ipitch + Dpitch;   PID_value_yaw = Pyaw + Iyaw + Dyaw;  PID_value_alt = Palt + Ialt + Dalt;  PID_value_vel = Pvel + Ivel + Dvel;
  previousTime_pid=currentTime_pid;

  if(PID_value_roll > PID_max){PID_value_roll = PID_max;} else if(PID_value_roll < PID_max*-1){PID_value_roll = PID_max*-1;}
  if(PID_value_pitch > PID_max){PID_value_pitch = PID_max;} else if(PID_value_pitch < PID_max*-1){PID_value_pitch = PID_max*-1;}
  if(PID_value_yaw > PID_max){PID_value_yaw = PID_max;} else if(PID_value_yaw < PID_max*-1){PID_value_yaw = PID_max*-1;}
  if(PID_value_alt > 200){PID_value_alt = 200;} else if(PID_value_alt < -200){PID_value_alt = -200;}
  PID_value_vel=constrain(PID_value_vel,-15,15);
  }

    rollControl  = PID_value_roll;
    pitchControl = PID_value_pitch;
    yawControl   = PID_value_yaw;
    altControl   = PID_value_alt;

//     if(ch7 == 0 && ch5 == 1 && ch6 == 0) {latitude_init = latitude; longitude_init = longitude; latitude_home = latitude; longitude_home = longitude; init_Home = 0; waypoint_index = 0; tracking = 0; cek_heading = 0; bearing = 0;pitch_on = 0;pitch_off = 0;}
//     if(((ch7  == 1 && head_mode == 1 && ch5 == 1 && ch6 == 0)||darurat == 1)) {waypointMission();}
    // if(ch5 == 1 && (head_mode ==0 || alt_mode==0)){ altitudeControl_ref = altitude_cm;}
    // if(ch5 == 1 && head_mode ==1 && alt_mode==1){ alt_input =  altitudeControl_ref;}
    if(ch5 == 1 && (head_mode ==0 || pst_mode==0)){
      mode_baru_aktif = true;
      target_pitch=target_roll=0;
    }
    if(ch5 == 1 && head_mode ==1 && pst_mode==1){
        if (mode_baru_aktif) {
            target_lat = gps_lat;
            target_lon = gps_lon;
            error_pos=error_vel=integral_error_pos=integral_error_vel=derivative_error_pos=derivative_error_vel=last_error_pos=last_error_vel=0;
            mode_baru_aktif = false;
        }
        double distance_cm_fast = calculate_distance_cm(gps_lat, gps_lon, target_lat, target_lon);
        distance_cm = distance_cm_fast * (float)0.8 + distance_cm_fast * (float)0.2;
        bearing_deg = calculate_bearing_deg(gps_lat, gps_lon, target_lat, target_lon);
        current_heading = fixedHeadingDegrees; //compass.heading_deg();
        angle_to_target = bearing_deg - current_heading;
        target_pitch = PID_value_vel * cos(radians(angle_to_target)) *-1;
        if(target_pitch >= 0){target_pitch = target_pitch;} else if(target_pitch < 0){target_pitch /= 2;}
        target_roll = PID_value_vel * sin(radians(angle_to_target)) *-1;
    }


       yawControl   = constrain(yawControl, -30, 30);
       pitchControl = constrain(pitchControl, -400, 400);
       rollControl  = constrain(rollControl, -400, 400);
       altControl  = constrain(altControl, -200, 200);
//       AltitudeControl = minMax(AltitudeControl, -125, 110);

//      if(head_mode == 1){
//        if(throttle_channel > 1000 && throttle_channel <= 1480) {throttle = map(throttle_channel,1000,1480,1320,1445);}
//        else if(throttle_channel > 1480) {throttle = map(throttle_channel,1481,2000,1446,1700);}
//       }
//      if(head_mode == 0){
//        throttle=  thrchaottle_nnel;
//      }
       pulse_length_esc1 = throttle_channel + (pitchControl) + (rollControl) + altControl; 
       pulse_length_esc3 = throttle_channel - (pitchControl) + altControl; 
       pulse_length_esc2 = throttle_channel + (pitchControl) - (rollControl) + altControl;
       pulse_length_servo1 = (yawControl*1);
       pulse_length_servo2 = (yawControl*1);

    pulse_length_esc1 = constrain(pulse_length_esc1, 1100, 2000);
    pulse_length_esc2 = constrain(pulse_length_esc2, 1100, 2000);
    pulse_length_esc3 = constrain(pulse_length_esc3, 1100, 2000);
    pulse_length_esc3 = map(pulse_length_esc3, 1100, 2000, 1100, 1850);
    pulse_length_servo1 = constrain(pulse_length_servo1, -30, 30);pulse_length_servo1 = map(pulse_length_servo1, -30, 30, servo1_down, servo1_up);
    pulse_length_servo2 = constrain(pulse_length_servo2, -30, 30);pulse_length_servo2 = map(pulse_length_servo2, -30, 30, servo2_down, servo2_up);
    statusmode = 0;
    // pitch_transisi = 0;
}

void controlPlane(){
  if(ch6==0){roll_input1=pitch_input1=yaw_input1=0;}
  if(ch7==1){
  currentTime_pid=millis();
  if(currentTime_pid-previousTime_pid>9){
  dt=(currentTime_pid-previousTime_pid);

//===============================================================VIRTUAL PID==============================================================
  error_roll2=(roll_input1)-roll_deg;
  error_pitch2=(pitch_input1)-pitch_deg;

  integral_error_roll2+=error_roll2*0.01;
  integral_error_pitch2+=error_pitch2*0.01;

  derivative_error_roll2=(error_roll2-last_error_roll2)/0.01;
  derivative_error_pitch2=(error_pitch2-last_error_pitch2)/0.01;

  Proll2=Kp_roll3*error_roll2;                 Ppitch2=Kp_pitch3*error_pitch2;
  Iroll2=Ki_roll3*integral_error_roll2;        Ipitch2=Ki_pitch3*integral_error_pitch2;
  Droll2=Kd_roll3*(derivative_error_roll2);    Dpitch2=Kd_pitch3*(derivative_error_pitch2);
  last_error_roll2=error_roll2;                last_error_pitch2=error_pitch2;

  PID_virtual_roll2 = Proll2 + Iroll2 + Droll2;   PID_virtual_pitch2 = Ppitch2 + Ipitch2 + Dpitch2;

//==============================================================ACTUAL PID========================================================
  error_roll3=PID_virtual_roll2-gx;
  error_pitch3=PID_virtual_pitch2-(gy*-1);

  integral_error_roll3+=error_roll3*0.01;
  integral_error_pitch3+=error_pitch3*0.01;

  derivative_error_roll3=(error_roll3-last_error_roll3)/0.01;
  derivative_error_pitch3=(error_pitch3-last_error_pitch3)/0.01;

  Proll3=Kp_roll4*error_roll3;                 Ppitch3=Kp_pitch4*error_pitch3;                
  Iroll3=Ki_roll4*integral_error_roll3;        Ipitch3=Ki_pitch4*integral_error_pitch3;        
  Droll3=Kd_roll4*(derivative_error_roll3);    Dpitch3=Kd_pitch4*(derivative_error_pitch3);   
  last_error_roll3=error_roll3;                last_error_pitch3=error_pitch3;                 

  PID_value_roll2 = Proll3 + Iroll3 + Droll3;   PID_value_pitch2 = Ppitch3 + Ipitch3 + Dpitch3;  
  previousTime_pid=currentTime_pid;

  if(PID_value_roll > PID_max2){PID_value_roll = PID_max2;} else if(PID_value_roll < PID_max2*-1){PID_value_roll = PID_max2*-1;}
  if(PID_value_pitch > PID_max2){PID_value_pitch = PID_max2;} else if(PID_value_pitch < PID_max2*-1){PID_value_pitch = PID_max2*-1;}
  }
  rollControl2  = PID_value_roll;
  pitchControl2 = PID_value_pitch;

  rollControl    = constrain(rollControl2, -30, 30);
  pitchControl   = constrain(pitchControl2, -30, 30);

  Servo1 = (rollControl2) - (pitchControl2*1); Servo1 = constrain(Servo1, -40,40); Servo1 = map(Servo1,-40,40,servo1_down1,servo1_up1); 
  Servo2 = (rollControl2) + (pitchControl2*1); Servo2 = constrain(Servo2, -40,40); Servo2 = map(Servo2,-40,40,servo2_down1,servo2_up1); 
 }
  if(ch7==0){
   Servo1 = (roll_input1) - (pitch_input1*1); Servo1 = constrain(Servo1, -40,40); Servo1 = map(Servo1,-40,40,servo1_down1,servo1_up1); 
   Servo2 = (roll_input1) + (pitch_input1*1); Servo2 = constrain(Servo2, -40,40); Servo2 = map(Servo2,-40,40,servo2_down1,servo2_up1); 
  }
}

