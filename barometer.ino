void baro_initialized(){
  unsigned status;
  status = bmp.begin(0x76,0x58);
  if (!status) {
    Serial1.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial1.print("SensorID was: 0x"); Serial1.println(bmp.sensorID(),16);
    Serial1.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial1.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial1.print("        ID of 0x60 represents a BME 280.\n");
    Serial1.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

float getAltitudeDerivative(){
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  float currentAltitude = altitude_m;
  float rate = (currentAltitude - lastAltitude) / deltaTime;
  lastAltitude = currentAltitude;
  lastTime = currentTime;
  lastRate = rate;
  return rate;
}

void updateBaro() {

    // Serial1.print(bmp.readTemperature());

    // Serial1.print(bmp.readPressure());

    // Serial1.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */

    // actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
     
    // actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
    // if (actual_pressure_diff > 8)actual_pressure_diff = 8;
    // if (actual_pressure_diff < -8)actual_pressure_diff = -8;
    // if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    // actual_pressure = actual_pressure_slow;

    altitude_m = bmp.readAltitude(1013.25);
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
