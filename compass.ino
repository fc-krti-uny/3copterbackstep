void compass_init()
{

  Serial1.println("Mencoba inisialisasi sensor IST8310...");

  // 1. Reset sensor untuk memastikan kondisi awal yang bersih
  Wire.beginTransmission(IST8310_I2C_ADDR);
  Wire.write(CNTL2_REG);
  Wire.write(0x01); // Kirim perintah reset
  if (Wire.endTransmission() != 0) {
    Serial1.println("Gagal mengirim perintah reset. Cek koneksi.");
    while(1);
  }
  delay(10); // Beri waktu sensor untuk memproses reset

  // 2. Verifikasi identitas sensor dengan membaca register "Who Am I"
  Wire.beginTransmission(IST8310_I2C_ADDR);
  Wire.write(WAI_REG);
  Wire.endTransmission();
  
  Wire.requestFrom(IST8310_I2C_ADDR, 1);//ohou
  if (Wire.available()) {
    byte deviceId = Wire.read();
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
  Wire.beginTransmission(IST8310_I2C_ADDR);
  Wire.write(AVGCNTL_REG);
  Wire.write(0x24); // Mengatur rata-rata 16x untuk Y dan 16x untuk XZ
  Wire.endTransmission();

  Serial1.println("Inisialisasi selesai. Memulai pembacaan data...");
  Serial1.println("-------------------------------------------------");
}

void compass_update()
{
  // 1. Minta sensor untuk melakukan satu kali pengukuran (Single Measurement Mode)
  Wire.beginTransmission(IST8310_I2C_ADDR);
  Wire.write(CNTL1_REG);
  Wire.write(0x01);
  Wire.endTransmission();

  delay(20); // Beri waktu sensor untuk menyelesaikan pengukuran

  // 2. Baca 6 byte data (X, Y, Z) dari sensor
  Wire.beginTransmission(IST8310_I2C_ADDR);
  Wire.write(OUTPUT_X_L_REG); // Tunjuk ke register data X LSB sebagai awal pembacaan
  Wire.endTransmission();

  Wire.requestFrom(IST8310_I2C_ADDR, 6);

  // Pastikan kita menerima 6 byte data sebelum memprosesnya
  if (Wire.available() >= 6) {
    // Data masuk dalam urutan: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    uint8_t x_lsb = Wire.read();
    uint8_t x_msb = Wire.read();
    uint8_t y_lsb = Wire.read();
    uint8_t y_msb = Wire.read();
    uint8_t z_lsb = Wire.read();
    uint8_t z_msb = Wire.read();

    // Gabungkan byte LSB dan MSB menjadi nilai integer 16-bit
    // (byte MSB digeser 8 bit ke kiri, lalu digabungkan dengan byte LSB)
    int16_t rawX = (int16_t)((x_msb << 8) | x_lsb);
    int16_t rawY = (int16_t)((y_msb << 8) | y_lsb);
    int16_t rawZ = (int16_t)((z_msb << 8) | z_lsb);

    // Negasikan nilai Z untuk menyesuaikan dengan konvensi navigasi 'right-hand rule'
    rawZ = -rawZ;

    // Calculate heading
    heading = atan2(rawY, rawX);

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

  // Fix HMC5883L issue with angles
 
  if (headingDegrees >= 90 && headingDegrees < 360)
  {
    fixedHeadingDegrees = mapFloat(headingDegrees, 90, 360, 0, 270);
  } else
  if (headingDegrees >= 0 && headingDegrees < 90)
  {
    fixedHeadingDegrees = mapFloat(headingDegrees, 0, 90, 270, 360);
  }

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
  }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}