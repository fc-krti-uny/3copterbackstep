void compass_init()
{
  // Initialize Initialize HMC5883L
  Serial1.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial1.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0, 0);
}

void compass_update()
{
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  headingDegrees = heading * 180/M_PI; 
fixedHeadingDegrees = headingDegrees;
  // Fix HMC5883L issue with angles
 
  // if (headingDegrees >= 90 && headingDegrees < 360)
  // {
  //   fixedHeadingDegrees = mapFloat(headingDegrees, 90, 360, 0, 270);
  // } else
  // if (headingDegrees >= 0 && headingDegrees < 90)
  // {
  //   fixedHeadingDegrees = mapFloat(headingDegrees, 0, 90, 270, 360);
  // }

    // yaw_reference = set_yaw - headingDegrees;
    // if(yaw_reference >   180) { yaw_reference -= 360;}
    // if(yaw_reference < - 180) { yaw_reference += 360;}
    // yaw_control = yaw_reference;
    // yawPrev = yaw_deg;

    // if(yaw_channel <= 1450 || yaw_channel >= 1550|| ch5==0)set_yaw = headingDegrees;
  // Tampilkan data mentah yang sudah digabungkan ke Serial1 Monitor
    // Serial1.print("Raw X: "); Serial1.print(rawX);
    // Serial1.print("\t| Raw Y: "); Serial1.print(rawY);
    // Serial1.print("\t| Raw Z: "); Serial1.print(rawZ);
    // Serial1.print("\t| Raw h: "); Serial1.print(headingDegrees);
    // Serial1.print("\t| Raw fh: "); Serial1.println(fixedHeadingDegrees);
  // }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}