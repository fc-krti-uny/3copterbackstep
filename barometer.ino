void baro_initialized(){
  Wire.beginTransmission(MS5611_address);                        //Start communication with the MS5611.
    //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
    //These 2 byte values are stored in the memory location 0xA2 and up.
    for (start = 1; start <= 6; start++) {
      Wire.beginTransmission(MS5611_address);                    //Start communication with the MPU-6050.
      Wire.write(0xA0 + start * 2);                              //Send the address that we want to read.
      Wire.endTransmission();                                    //End the transmission.

      Wire.requestFrom(MS5611_address, 2);                       //Request 2 bytes from the MS5611.
      C[start] = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the C[x] calibration variable.
    }
}

float getAltitudeDerivative(){
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  float currentAltitude = altitude_cm;
  float rate = (currentAltitude - lastAltitude) / deltaTime;
  lastAltitude = currentAltitude;
  lastTime = currentTime;
  lastRate = rate;
  return rate;
}

void updateBaro() {
  barometer_counter ++;

  if (barometer_counter == 1) {
    if (temperature_counter == 0) {
      //Get temperature data from MS-5611
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(MS5611_address, 3);

      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      raw_temperature_rotating_memory[average_temperature_mem_location] = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;
    }
    else {
      //Get pressure data from MS-5611
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(MS5611_address, 3);
      raw_pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
    }

    temperature_counter ++;
    if (temperature_counter == 20) {
      temperature_counter = 0;
      //Request temperature data  
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x58);
      Wire.endTransmission();
    }
    else {
      //Request pressure data
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x48);
      Wire.endTransmission();
    }
  }
  if (barometer_counter == 2) {
    //Calculate pressure as explained in the datasheet of the MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);

    //Let's use a rotating mem
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];    //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                          //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];    //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                   //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;        //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                        //Calculate the avarage pressure value of the last 20 readings.
    if(millis() < 5000)actual_pressure_slow = actual_pressure_fast;                     //Keep the slow and fast avareges the same for the first 5 seconds.

    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
     
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;

    altitude_m = 44330*(1-pow((actual_pressure*0.01) /1013.25, 0.1903));
    altitude_cm = altitude_m*100;

  if(ch5 == 0){ AltitudeBaroGround = altitude_m; axis = 0; z_position = 0;}
  if(ch5 == 1)
  {
    if(axis == 0){ 
      alt_ref = AltitudeBaroGround;  estimation_altitude = 0; 
    }
    estimation_altitude = altitude_m - alt_ref;
    z_position = estimation_altitude; axis = 1;
  }
  }

  
  if (millis() > 5000 && start == 0) {
    start = 1;
    top_line = actual_pressure + 20;
    bottom_line = actual_pressure - 20;
  }

  while (loop_timer > micros());
  loop_timer = micros() + 4000;                                                         //Set the loop_timer variable to the current micros() value + 4000.
}
