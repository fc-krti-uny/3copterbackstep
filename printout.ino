void print_out() {
  timeProgram = micros();
  if (timeProgram - previousTimeProgram >= 100000) 
  { 
    // Serial1.print(millis());Serial1.print("\t");
    Serial1.print(throttle_channel);Serial1.print("\t");
    // Serial1.print(pitch_channel);Serial1.print("\t");
    // Serial1.print(yaw_channel);Serial1.print("\t");
    // Serial1.print(yaw_reference);Serial1.print("\t");
    // Serial1.print(roll_input);Serial1.print("\t");
    // Serial1.print(yaw_control);Serial1.print("\t");
    // Serial1.print(yaw_input);Serial1.print("\t");
    // Serial1.print(PID_value_yaw);Serial1.print("\t");
    // Serial1.print(yaw_input);Serial1.print("\t");
    // Serial1.print(gx);Serial1.print("\t");
    // Serial1.print(gy);Serial1.print("\t");
    // Serial1.print(gz);Serial1.print("\t");
    // Serial1.print(statusAktif);Serial1.print("\t");
    atitude_data();
    // motorServo_ManualTrimming();
    // gainTuningFeedback();
    motor_output();
    Serial1.println();
    previousTimeProgram = micros();
  }
}

void atitude_data() {
  Serial1.print(roll_deg)      ;Serial1.print("\t");
  Serial1.print(pitch_deg)      ;Serial1.print("\t");
  Serial1.print(yaw_deg)      ;Serial1.print("\t");
  Serial1.print(fixedHeadingDegrees)      ;Serial1.print("\t");
  Serial1.print(altitude_m)      ;Serial1.print("\t");
  Serial1.print(z_position)      ;Serial1.print("\t");
  Serial1.print(gps_lat,7)      ;Serial1.print("\t");
  Serial1.print(gps_lon,7)      ;Serial1.print("\t");
}

void motor_output(){
  Serial1.print(pulse_length_esc1)      ;Serial1.print("\t");
  Serial1.print(pulse_length_esc2)      ;Serial1.print("\t");
  Serial1.print(pulse_length_esc3)      ;Serial1.print("\t");
  Serial1.print(pulse_length_servo1)      ;Serial1.print("\t");
  Serial1.print(pulse_length_servo2)      ;Serial1.print("\t");
  Serial1.print(Servo1)      ;Serial1.print("\t");
  Serial1.print(Servo2)      ;Serial1.print("\t");
}

// void gainTuningFeedback()
// {
//   Serial1.print(Kp_roll,2)      ;Serial1.print("\t");
//   Serial1.print(Ki_roll,4)      ;Serial1.print("\t");
//   Serial1.print(Kd_roll,3)      ;Serial1.print("\t");
//   Serial1.print(Kp_roll2,3)      ;Serial1.print("\t");

//   Serial1.print(Kp_pitch,2)      ;Serial1.print("\t");
//   Serial1.print(Ki_pitch,4)      ;Serial1.print("\t");
//   Serial1.print(Kd_pitch,3)      ;Serial1.print("\t");
//   Serial1.print(Kp_pitch2,3)      ;Serial1.print("\t");

  // Serial1.print(Kp_pos,2)      ;Serial1.print("\t");
  // Serial1.print(Kp_vel,2)      ;Serial1.print("\t");
  // Serial1.print(gain_yaw,3)      ;Serial1.print("\t");
  // Serial1.print(gain_vel_yaw,4)  ;Serial1.print("\t");
// }

// void motorServo_ManualTrimming(){
//     Serial1.print(servo1_up1)        ;Serial1.print("\t");
//     Serial1.print(servo1_down1)      ;Serial1.print("\t");
//     Serial1.print(servo2_up1)        ;Serial1.print("\t");
//     Serial1.print(servo2_down1)      ;Serial1.print("\t");
//     Serial1.print(servoAngleInitA)  ;Serial1.print("\t");
//     Serial1.print(servoAngleInitB)  ;Serial1.print("\t");
//   }
// void atitude_data() {
//   Serial1.print("roll:"); Serial1.print(roll_deg); Serial1.print(",");  //sintaks nampilin ke GCS
//   Serial1.print("pitch:"); Serial1.print(pitch_deg); Serial1.print(",");
//   Serial1.print("yaw:"); Serial1.print(yaw_deg); Serial1.print(",");
//   Serial1.print("height:"); Serial1.print(z_position); Serial1.print(",");
//   Serial1.print("batteryvolt:"); Serial1.print(vout); Serial1.print(",");  //baca volt  batre
//   Serial1.print("compass:"); Serial1.print(fixedHeadingDegrees) ;Serial1.print(",");
//   Serial1.print("Lat:"); Serial1.print(gps_lat, 7); Serial1.print(",");
//   Serial1.print("Lon:"); Serial1.print(gps_lon, 7); Serial1.print(",");
//   // Serial1.print(gps_lat,7)      ;Serial1.print("\t");
//   // Serial1.print(gps_lon,7)      ;Serial1.print("\t");
// }

// void motor_output(){
//  Serial1.print("esc1:");  Serial1.print(pulse_length_esc1)      ;Serial1.print(",");
//   Serial1.print("esc2:");  Serial1.print(pulse_length_esc2)      ;Serial1.print(",");
//   Serial1.print("esc3:");  Serial1.print(pulse_length_esc3)      ;Serial1.print(",");
//   // Serial1.print(pulse_length_servo1)      ;Serial1.print("\t");
//   // Serial1.print(pulse_length_servo2)      ;Serial1.print("\t");
//   // Serial1.print(Servo1)      ;Serial1.print("\t");
//   // Serial1.print(Servo2)      ;Serial1.print("\t");
// }