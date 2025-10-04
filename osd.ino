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
  char rollBuffer[10];
  dtostrf(roll_deg, 6, 2, rollBuffer);
  osd.print(rollBuffer, 0, 1); 
  
  char pitchBuffer[10];
  dtostrf(pitch_deg, 6, 2, pitchBuffer);
  osd.print(pitchBuffer, 7, 1); 

  char latBuffer[10];
  dtostrf(gps_lat, 6, 2, latBuffer);
  osd.print(latBuffer, 0, 3); 
  
  char lonBuffer[10];
  dtostrf(gps_lon, 6, 2, lonBuffer);
  osd.print(lonBuffer, 7, 3); 
  
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

  // delay(100);
}
