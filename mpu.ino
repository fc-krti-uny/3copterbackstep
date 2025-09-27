// ISR untuk interrupt dari MPU
void dmpDataReady() {
    mpuInterrupt = true;
}

// ==== Inisialisasi MPU ====
void init_MPU() {
    Serial1.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial1.println(F("Testing device connections..."));
    Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                         : F("MPU6050 connection failed"));

    // load dan konfigurasi DMP
    Serial1.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Set offset hasil kalibrasi (ganti dengan hasilmu)
    mpu.setXGyroOffset(-76);
    mpu.setYGyroOffset(384);
    mpu.setZGyroOffset(-42);
    mpu.setXAccelOffset(357);
    mpu.setYAccelOffset(-2485);
    mpu.setZAccelOffset(517);

    if (devStatus == 0) {
        // Enable DMP
        mpu.setDMPEnabled(true);

        // pasang interrupt
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // ukuran paket FIFO
        packetSize = mpu.dmpGetFIFOPacketSize();

        Serial1.println(F("DMP ready!"));
    } else {
        Serial1.print(F("DMP Initialization failed (code "));
        Serial1.print(devStatus);
        Serial1.println(F(")"));
    }

    pinMode(LED_PIN, OUTPUT);

    // pastikan bypass & sleep off
    mpu.setI2CMasterModeEnabled(false);
    mpu.setI2CBypassEnabled(false);
    mpu.setSleepEnabled(false);
}

// ==== Ambil YPR & gyro ====
void get_YPR() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (!mpuInterrupt) return;   // tidak ada data baru, keluar saja
    mpuInterrupt = false;

    mpuIntStatus = mpu.getIntStatus();
    fifoCount    = mpu.getFIFOCount();

    // cek overflow FIFO
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        // Serial1.println(F("FIFO overflow!"));
        return;   // keluar, jangan update variabel
    }

    // cek apakah cukup 1 paket data
    if (fifoCount < packetSize) return;

    // ambil data FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // quaternion → yaw, pitch, roll
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    pitch_deg = ypr[1] * 180.0f / M_PI;
    roll_deg  = ypr[2] * 180.0f / M_PI;
    yaw_deg   = ypr[0] * 180.0f / M_PI;
    if (yaw_deg < 0) yaw_deg += 360.0f;
    yaw_reference = set_yaw - yaw_deg;
    if(yaw_reference >   180) { yaw_reference -= 360;}
    if(yaw_reference < - 180) { yaw_reference += 360;}
    yaw_control = yaw_reference;
    
    yawPrev = yaw_deg;

    if(yaw_channel <= 1450 || yaw_channel >= 1550|| ch5==0 || ch6 == 2)set_yaw = yaw_deg;

    // ambil gyro mentah → dps
    mpu.getRotation(&gx, &gy, &gz);
    gx = gx / 16.4f; // ±250 dps scale
    gy = (gy / 16.4f)-9;
    gz = gz / 16.4f;
}