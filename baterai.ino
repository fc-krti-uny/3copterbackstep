void batterypercent() { // masukkan fungsi ini di loop
  adc = analogRead(PC1);
  vin = (adc / 4095.0) * 3.3;
  float voltage_filtered = vin * 4.7 * 5;
  vout = (0.1 * voltage_filtered) + ((1 - 0.1) * vout);
  currentADC = analogRead(PC0);
  float voltage = (currentADC / 4095.0) * 3.3;
  currentSense = voltage * CURRENT_SCALE;
}

//  Serial.print(vout);
//  Serial.print("V ");

// if(vout >=14.7){
//   float percentage = fmap(vout, 14.7, 16.8, 0, 100);
// //  Serial.print("(");
// //  Serial.print(percentage); 
// //  Serial.println("%)");
//   }
//  else{Serial.println("(0.0%)");}