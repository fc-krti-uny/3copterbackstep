void compass_init()
{
  Serial1.println("Mencoba inisialisasi sensor IST8310...");

  // 1. Reset sensor untuk memastikan kondisi awal yang bersih
  Wire2.beginTransmission(IST8310_I2C_ADDR);
  Wire2.write(CNTL2_REG);
  Wire2.write(0x01); // Kirim perintah reset
  if (Wire2.endTransmission() != 0) {
    Serial1.println("Gagal mengirim perintah reset. Cek koneksi.");
    while(1);
  }
  delay(10); // Beri waktu sensor untuk memproses reset

  // 2. Verifikasi identitas sensor dengan membaca register "Who Am I"
  Wire2.beginTransmission(IST8310_I2C_ADDR);
  Wire2.write(WAI_REG);
  Wire2.endTransmission();//lllll
  
  Wire2.requestFrom(IST8310_I2C_ADDR, 1);
  if (Wire2.available()) {
    byte deviceId = Wire2.read();
    if (deviceId == DEVICE_ID) {
      Serial1.println("Sensor IST8310 berhasil ditemukan dan diverifikasi!");
    } else {
      Serial1.print("Perangkat merespons, tapi ID tidak cocok! Diharapkan 0x10, diterima 0x");
      Serial1.println(deviceId, HEX);
      while(1); // Hentikan program jika sensor tidak cocok
    }
  } else {
    Serial1.println("Sensor IST8310 tidak ditemukan di alamat 0x0E. Cek wiring!");
    while(1); // Hentikan program jika tidak ada respons
  }
  
  // 3. Konfigurasi sensor (contoh: mengatur averaging untuk pembacaan yang lebih stabil)
  Wire2.beginTransmission(IST8310_I2C_ADDR);
  Wire2.write(AVGCNTL_REG);
  Wire2.write(0x24); // Mengatur rata-rata 16x untuk Y dan 16x untuk XZ
  Wire2.endTransmission();


  Serial1.println("Inisialisasi selesai. Memulai pembacaan data...");
  Serial1.println("-------------------------------------------------");
}

void compass_update()
{
    // 1. Minta sensor untuk melakukan satu kali pengukuran (Single Measurement Mode)
  Wire2.beginTransmission(IST8310_I2C_ADDR);
  Wire2.write(CNTL1_REG);
  Wire2.write(0x01);
  Wire2.endTransmission();

  // delay(20); // Beri waktu sensor untuk menyelesaikan pengukuran

  // 2. Baca 6 byte data (X, Y, Z) dari sensor
  Wire2.beginTransmission(IST8310_I2C_ADDR);
  Wire2.write(OUTPUT_X_L_REG); // Tunjuk ke register data X LSB sebagai awal pembacaan
  Wire2.endTransmission();

  Wire2.requestFrom(IST8310_I2C_ADDR, 6);

  // Pastikan kita menerima 6 byte data sebelum memprosesnya
  if (Wire2.available() >= 6) {
    // Data masuk dalam urutan: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    uint8_t x_lsb = Wire2.read();
    uint8_t x_msb = Wire2.read();
    uint8_t y_lsb = Wire2.read();
    uint8_t y_msb = Wire2.read();
    uint8_t z_lsb = Wire2.read();
    uint8_t z_msb = Wire2.read();

    // Gabungkan byte LSB dan MSB menjadi nilai integer 16-bit
    // (byte MSB digeser 8 bit ke kiri, lalu digabungkan dengan byte LSB)
    int16_t rawX = (int16_t)((x_msb << 8) | x_lsb);
    int16_t rawY = (int16_t)((y_msb << 8) | y_lsb);
    int16_t rawZ = (int16_t)((z_msb << 8) | z_lsb);

    // Negasikan nilai Z untuk menyesuaikan dengan konvensi navigasi 'right-hand rule'
    rawZ = -rawZ;

    // 2. Terapkan kalibrasi Hard/Soft Iron (2D) terlebih dahulu
    // PENTING: Lakukan ini sebelum kompensasi kemiringan!
    float magX = (rawX - x_offset) * x_scale;
    float magY = (rawY - y_offset) * y_scale;
    float magZ = rawZ; // Sumbu Z tidak dikalibrasi di metode 2D ini, kita gunakan nilai mentahnya

    // --- MULAI LOGIKA TILT COMPENSATION ---

    // 3. Hitung sudut Roll (phi) dan Pitch (theta) dari akselerometer
    // Hasilnya dalam radian, yang kita butuhkan untuk fungsi sin() dan cos()
    // float rollC = atan2(ay, az);
    // float pitchC = atan2(-ax, sqrt(ay * ay + az * az));
    // float acc_x = ax*2.0 / 32768.0;
    // float acc_y = ay*2.0 / 32768.0;

    // acc_x = constrain(acc_x, -1, 1);
    // acc_y = constrain(acc_y, -1, 1);

    // float pitchC = asin(acc_x);
    // float rollC  = asin(-acc_y);

    // // Untuk efisiensi, hitung nilai sin dan cos sekali saja
    // float cosRoll = cos(rollC);
    // float sinRoll = sin(rollC);
    // float cosPitch = cos(pitchC);
    // float sinPitch = sin(pitchC);

    // // 4. "De-rotate" data magnetometer untuk mendapatkan komponen horizontal
    // float X_horizontal = magX * cosPitch + magY * sinRoll * sinPitch - magZ * cosRoll * sinPitch;
    // float Y_horizontal = magY * cosRoll + magZ * sinRoll;
  // Calculate heading
  float heading = atan2(magY, magX);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (0.0 + (46.0 / 60.0)) / (180 / M_PI);
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
// fixedHeadingDegrees = headingDegrees;
  // Fix HMC5883L issue with angles
 
  if (headingDegrees >= 90 && headingDegrees < 360)
  {
    fixedHeadingDegrees = mapFloat(headingDegrees, 90, 360, 0, 270);
  } else
  if (headingDegrees >= 0 && headingDegrees < 90)
  {
    fixedHeadingDegrees = mapFloat(headingDegrees, 0, 90, 270, 360);
  }

    yaw_reference = set_yaw - fixedHeadingDegrees;
    if(yaw_reference >   180) { yaw_reference -= 360;}
    if(yaw_reference < - 180) { yaw_reference += 360;}
    yaw_control = yaw_reference;
    yawPrev = fixedHeadingDegrees;

    if(yaw_channel <= 1450 || yaw_channel >= 1550|| ch5==0)set_yaw = fixedHeadingDegrees;
  // Tampilkan data mentah yang sudah digabungkan ke Serial1 Monitor
    // Serial1.print("Raw X: "); Serial1.print(rawX);
    // Serial1.print("\t| Raw Y: "); Serial1.print(rawY);
    // Serial1.print("\t| Raw Z: "); Serial1.print(rawZ);
    // Serial1.print("\t| Raw h: "); Serial1.print(headingDegrees);
    // Serial1.print("\t| Raw fh: "); Serial1.println(fixedHeadingDegrees);
  }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}