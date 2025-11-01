// Fungsi untuk memecah data berdasarkan koma

void esc_read(){
  if (Serial6.available() > 0) {
    // Baca 1 baris penuh
    String data = Serial6.readStringUntil('\n');
    data.trim(); // hapus karakter \r kalau ada

    // Parsing data CSV
    int index1 = data.indexOf(',');
    int index2 = data.indexOf(',', index1 + 1);
    int index3 = data.indexOf(',', index2 + 1);
    int index4 = data.indexOf(',', index3 + 1);

    if (index1 > 0 && index2 > 0 && index3 > 0 && index4 > 0) {
      teganganInput  = data.substring(0, index1).toFloat();
      teganganOutput = data.substring(index1 + 1, index2).toFloat();
      arus           = data.substring(index2 + 1, index3).toFloat();
      suhu           = data.substring(index3 + 1, index4).toFloat();
      status_esc     = data.substring(index4 + 1);
      suhu /= 100;
      // Tampilkan hasil parsing
      // Serial1.println("=== DATA DITERIMA ===");
      // Serial1.print("Tegangan Input  : "); Serial1.println(teganganInput, 2);
      // Serial1.print("Tegangan Output : "); Serial1.println(teganganOutput, 2);
      // Serial1.print("Arus            : "); Serial1.println(arus, 2);
      // Serial1.print("Suhu            : "); Serial1.println(suhu, 2);
      // Serial1.print("Status          : "); Serial1.println(status_esc);
      // Serial1.println("=====================\n");
    }
  }
}