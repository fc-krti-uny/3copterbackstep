// // Asumsikan Anda sudah memiliki PID controller untuk Attitude (Roll, Pitch)
// // Buat dua objek PID baru
// PID pos_pid;
// PID vel_pid;

// // Variabel target
// float target_lat, target_lon;

// void loop() {
//     // ... baca semua sensor dan input RC ...
    
//     if (mode == POSHOLD) {
//         // Jika baru beralih ke PosHold, kunci posisi saat ini sebagai target
//         if (mode_baru_aktif) {
//             target_lat = gps.latitude;
//             target_lon = gps.longitude;
//             pos_pid.reset(); // Reset integral PID
//             vel_pid.reset();
//             mode_baru_aktif = false;
//         }

//         // LANGKAH 6.1: Hitung error posisi
//         float distance_cm = calculate_distance_cm(gps.latitude, gps.longitude, target_lat, target_lon);
//         float bearing_deg = calculate_bearing_deg(gps.latitude, gps.longitude, target_lat, target_lon);

//         // LANGKAH 6.2: PID Posisi (Error Jarak -> Target Kecepatan)
//         float target_velocity = pos_pid.update(distance_cm, 0);
//         target_velocity = constrain(target_velocity, 0, 500); // Batasi maks 5 m/s

//         // LANGKAH 6.3: PID Kecepatan (Error Kecepatan -> Target Sudut)
//         float current_velocity = gps.speed_cmps();
//         float target_angle = vel_pid.update(target_velocity, current_velocity);
//         target_angle = constrain(target_angle, -20, 20); // Batasi maks kemiringan 20 derajat

//         // LANGKAH 6.4: Proyeksikan sudut ke Roll dan Pitch
//         float current_heading = compass.heading_deg();
//         float angle_to_target = bearing_deg - current_heading;

//         // Ubah menjadi target roll dan pitch
//         float target_pitch = target_angle * cos(radians(angle_to_target));
//         float target_roll = target_angle * sin(radians(angle_to_target));
        
//         // Kirim target ke PID Stabilisasi Anda yang sudah ada
//         // (ini akan menggantikan input dari stik remot Anda)
//         stabilize_pid_roll.setpoint = target_roll;
//         stabilize_pid_pitch.setpoint = target_pitch;

//     } else {
//         // Logika untuk mode lain (Stabilize, AltHold, dll)
//         mode_baru_aktif = true; // Siapkan untuk PosHold berikutnya
//         stabilize_pid_roll.setpoint = rc_input_roll;
//         stabilize_pid_pitch.setpoint = rc_input_pitch;
//     }

//     // ... sisa kode Anda untuk menjalankan PID Stabilisasi dan Motor Mixing ...
// }