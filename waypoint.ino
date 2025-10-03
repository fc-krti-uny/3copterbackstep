// void waypointMission()
// {
//   if(tracking == 0){
//     if(init_Home == 0){ lat1  = latitude_init; long1 = longitude_init; lat2  = WaypointLatitude[waypoint_index]; long2 = WaypointLongitude[waypoint_index];}

//     if(init_Home == 1 && waypoint_index < waypointCount)
//     {
//       lat1  = latitude_init;
//       long1 = longitude_init;
//       lat2  = WaypointLatitude[waypoint_index];
//       long2 = WaypointLongitude[waypoint_index];
//     }

//     BearingCalculation();
//     waypointTime_current = millis();
//     if(waypointTime_current - waypointTime_previous >= 20){ checkBearing(); waypointTime_previous = millis();}
    
//     // yawControlServo   = (yaw_input + heading_control*trim_yaw); yawControlServo = constrain(yawControlServo, -10, 10);
//     if(setHeading == bearing )
//     {
//       if(cek_heading == 0)
//       { 
//         if(heading_control >= 0) { cek_heading = 1;}
//         if(heading_control <= 0) { cek_heading = 2;}
//       }
//       if(cek_heading == 1){if(heading_control <= 0){init_Home = 1; tracking = 1; waypoint = 0;} cek_heading = 1;}
//       if(cek_heading == 2){if(heading_control >= 0){init_Home = 1; tracking = 1; waypoint = 0;} cek_heading = 2;}
//    }
//   }

//    if(tracking == 1)
//    {
//     lat1  = gps_lat;
//     long1 = gps_lon;
//     lat2  = WaypointLatitude[waypoint_index]; 
//     long2 = WaypointLongitude[waypoint_index];

//      BearingCalculation();
//      setHeading = bearing;
    
//     // yawControlServo   = (yaw_input + heading_control*trim_yaw); yawControlServo = constrain(yawControlServo, -3, 3);
    
//     if(((lat1 > lat2 - 0.00005) && (lat1 < lat2 + 0.00005))&&((long1 > long2 - 0.00005) && (long1 < long2 + 0.00005)))
//     {
//       waypoint = 1;
//     }
//     if(waypoint == 1){ tracking = 0; init_Home = 1; waypoint_index++; latitude_init = gps_lat; longitude_init = gps_lon; cek_heading = 0; if(waypoint_index>(waypointCount-1)){tracking =2;}}
//    }

//    if(tracking == 2)
//    {
//     //  throttle = throttle_channel;
//     //  yawControlServo = (yaw_input + heading_control*trim_yaw) + (gyroZ_filt*gain_vel_yaw); yawControlServo = constrain(yawControlServo, -30, 30);
//     // kembali hover
//    }
// }

// void checkBearing()
// {
//   if(bearing < setHeading){ bearingHeading = bearing - setHeading;
//     if(bearingHeading < -180) { setHeading++;
//       if(setHeading > 360) { setHeading -= 360;}}
//     else{ setHeading--;
//       if(bearing > setHeading) { setHeading = bearing;}}} 
      
//   if(bearing > setHeading){ bearingHeading = bearing - setHeading;
//     if(bearingHeading > 180) { setHeading--;
//       if(setHeading < 0) { setHeading += 360;}}
//     else{ setHeading++;
//       if(bearing < setHeading) { setHeading = bearing;}}} 
// }

// void BearingCalculation()
// {
//   dLon = (long2 - long1)*DEG2RAD;
//   brlat1 = lat1*DEG2RAD;
//   brlat2 = lat2*DEG2RAD;
//   a1bearing = sin(dLon) * cos(brlat2);
//   a2bearing = sin(brlat1) * cos(brlat2) * cos(dLon);
//   a2bearing = cos(brlat1) * sin(brlat2) - a2bearing;
//   a2bearing = atan2(a1bearing, a2bearing);
//   if(a2bearing < 0)    { a2bearing += 6.28;}
//   if(a2bearing > 6.28) { a2bearing -= 6.28;}

//   bearing = RAD2DEG * a2bearing;
// }
