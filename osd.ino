Max7456       osd;
unsigned long counter = 0;
byte          tab[] = {0xC8, 0xC9};


void osd_init() {
  // delay(1000); // Beri jeda 1 detik
  SPI.begin();
  osd.init(PA4);

  osd.setDisplayOffsets(60, 18);
  osd.setBlinkParams(_8fields, _BT_BT);

  osd.activateOSD();
  // osd.printMax7456Char(0x01, 0, 1);
  // osd.print("Az-Zawra Biantara", 0, 0);
  // osd.print("Current Arduino time :", 0, 5);
  // osd.print("Suhu:", 0, 5);
  // osd.print("Humi:", 12, 5);

  // osd.printMax7456Char(0xD1, 9, 6, true);
  // osd.print("00'00\"", 10, 6);
  // osd.printMax7456Chars(tab, 2, 12, 7);
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);

  //base time = 160ms,time on = time off.
}

void osd_baca() {
  //Sintaks nampilin data FC ke kamera
  // char rollBuffer[10];
  // dtostrf(roll_deg, 6, 2, rollBuffer);
  // osd.print(rollBuffer, 0, 1); 
  
  // char pitchBuffer[10];
  // dtostrf(pitch_deg, 6, 2, pitchBuffer);
  // osd.print(pitchBuffer, 7, 1); 

  char latBuffer[15];
  dtostrf(gps_lat, 10, 6, latBuffer); // 6 angka di belakang koma cukup akurat
  osd.print("LAT:", 0, 2);
  osd.print(latBuffer, 4, 2);

  char lonBuffer[15];
  dtostrf(gps_lon, 10, 6, lonBuffer);
  osd.print("LON:", 0, 3);
  osd.print(lonBuffer, 4, 3);

  
  if (counter % 2 == 0) {
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, HIGH);
  } else {
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
  }

  counter = millis() / 1000;

  osd.print(int(counter / 60), 10, 6, 2, 0, false, true);
  osd.print(int(counter % 60), 13, 6, 2, 0, false, true);

  char voltBuffer[10];
  dtostrf(vout, 5, 2, voltBuffer);  // total 5 digit, 2 angka di belakang koma
  osd.print("V:", 0, 5);               // label
  osd.print(voltBuffer, 2, 5);         // nilai

  char currBuffer[10];
  dtostrf(currentSense, 5, 2, currBuffer);
  osd.print("A:", 8, 5);               // label
  osd.print(currBuffer, 10, 5);        // nilai
  // delay(100);
}
