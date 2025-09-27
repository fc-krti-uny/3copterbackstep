void resetParser() {
  state = WAIT_SYNC1;
  ckA = ckB = 0;
  ubxLen = 0;
  payloadCnt = 0;
}

void feedUBX(uint8_t b) {
  switch (state) {
    case WAIT_SYNC1:
      if (b == UBX_SYNC1) state = WAIT_SYNC2;
      break;

    case WAIT_SYNC2:
      if (b == UBX_SYNC2) state = WAIT_CLASS;
      else state = WAIT_SYNC1;
      break;

    case WAIT_CLASS:
      ubxClass = b;
      ckA = ckB = 0;
      ckUpdate(b);
      state = WAIT_ID;
      break;

    case WAIT_ID:
      ubxId = b;
      ckUpdate(b);
      state = WAIT_LEN1;
      break;

    case WAIT_LEN1:
      ubxLen = b;
      ckUpdate(b);
      state = WAIT_LEN2;
      break;

    case WAIT_LEN2:
      ubxLen |= (uint16_t)b << 8;
      ckUpdate(b);

      // Hanya terima NAV-PVT dengan panjang 92, lainnya di-skip elegan
      if (ubxClass == UBX_CLASS_NAV && ubxId == UBX_ID_PVT && ubxLen == UBX_LEN_PVT) {
        payloadCnt = 0;
        state = WAIT_PAYLOAD;
      } else {
        // Buang len bytes: cara cepat → tinggal baca len bytes dan 2 checksum lalu reset
        // Di sini kita pakai mode "sinkron cepat": balik ke WAIT_SYNC1, biar cari header lagi.
        resetParser();
      }
      break;

    case WAIT_PAYLOAD:
      payload[payloadCnt++] = b;
      ckUpdate(b);
      if (payloadCnt >= ubxLen) state = WAIT_CK_A;
      break;

    case WAIT_CK_A:
      rxCkA = b;
      state = WAIT_CK_B;
      break;

    case WAIT_CK_B:
      rxCkB = b;
      // verifikasi checksum
      if (rxCkA == ckA && rxCkB == ckB) {
        memcpy(&navPvt, payload, sizeof(NAV_PVT_t));
        navPvtReady = true;
      }
      resetParser();
      break;
  }
}

void printNavPvt(const NAV_PVT_t& p) {
  // Cek minimal validitas: fixType >= 2 (2D/3D), numSV >= 4, akurasi masuk akal
  bool validFix = (p.fixType >= 2) && (p.numSV >= 4) && (p.hAcc > 0) && (p.hAcc < 1000000UL);
  // Serial.print("Fix: "); Serial.print(p.fixType);
  // Serial1.print(" Sats: "); Serial1.print(p.numSV);
  gps_sats = p.numSV;

  if (validFix) { 
    gps_lat = p.lat / 1e7;
    gps_lon = p.lon / 1e7;
    // double alt_m = p.hMSL / 1000.0;
    gps_speed = p.gSpeed / 10.0;   // mm/s -> m/s
    // gps_speed = gps_speed * (float)0.985 + gps_speed_fast * (float)0.015;
    // double spd_cms = p.gSpeed / 10.0;    // mm/s -> cm/s

    // Serial1.print(" Lat: "); Serial1.print(gps_lat, 7);
    // Serial1.print(" Lon: "); Serial1.print(gps_lon, 7);
    // Serial.print(" Alt(m): "); Serial.print(alt_m, 2);
    // Serial1.print(" Speed(m/s): "); Serial1.println(gps_speed, 2);
    // Serial.print(" ("); Serial.print(spd_cms, 1); Serial.print(" cm/s)");
  } else {
    // Serial1.print(" (no valid fix)");
  }
  // Serial.println();
}

void init_gps(){
  Serial2.begin(230400);
  // Serial.println("UBX NAV-PVT parser start");
}

void update_gps(){
    // Feed parser dengan byte dari GPS
  while (Serial2.available()) {
    feedUBX((uint8_t)Serial2.read());
  }

  // Kalau ada NAV-PVT baru & valid checksum → cetak
  if (navPvtReady) {
    navPvtReady = false;
    printNavPvt(navPvt);
  }
}


/**
 * @brief Menghitung jarak antara dua koordinat GPS dalam sentimeter.
 * Menggunakan formula Haversine untuk akurasi yang baik.
 * @param lat1 Lintang titik pertama dalam derajat.
 * @param lon1 Bujur titik pertama dalam derajat.
 * @param lat2 Lintang titik kedua dalam derajat.
 * @param lon2 Bujur titik kedua dalam derajat.
 * @return Jarak dalam sentimeter (cm).
 */
double calculate_distance_cm(double lat1, double lon1, double lat2, double lon2) {
    // Ubah derajat ke radian
    double lat1_rad = radians(lat1);
    double lon1_rad = radians(lon1);
    double lat2_rad = radians(lat2);
    double lon2_rad = radians(lon2);

    // Hitung selisih
    double dLat = lat2_rad - lat1_rad;
    double dLon = lon2_rad - lon1_rad;

    // Formula Haversine
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    // double c_slow = c_slow * (float)0.985 + c * (float)0.015;

    // Jarak dalam meter, lalu ubah ke sentimeter
    return EARTH_RADIUS_M * c *100; //100
}

/**
 * @brief Menghitung bearing (arah) awal dari titik 1 ke titik 2.
 * @param lat1 Lintang titik pertama dalam derajat.
 * @param lon1 Bujur titik pertama dalam derajat.
 * @param lat2 Lintang titik kedua dalam derajat.
 * @param lon2 Bujur titik kedua dalam derajat.
 * @return Bearing dalam derajat (0-360, 0 = Utara).
 */
double calculate_bearing_deg(double lat1, double lon1, double lat2, double lon2) {
    // Ubah derajat ke radian
    double lat1_rad = radians(lat1);
    double lon1_rad = radians(lon1);
    double lat2_rad = radians(lat2);
    double lon2_rad = radians(lon2);

    // Hitung selisih bujur
    double dLon = lon2_rad - lon1_rad;

    // Formula bearing
    double y = sin(dLon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) -
               sin(lat1_rad) * cos(lat2_rad) * cos(dLon);

    // Hitung bearing dalam radian menggunakan atan2, lalu ubah ke derajat
    double bearing_rad = atan2(y, x);
    double bearing_deg = degrees(bearing_rad);

    // Normalisasi hasil agar berada dalam rentang 0-360 derajat
    return fmod((bearing_deg + 360.0), 360.0);
}

void updateSpeed(double currLat, double currLon) {

}