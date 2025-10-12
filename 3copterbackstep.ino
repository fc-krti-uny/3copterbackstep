//MPU
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <HardwareSerial.h>

//OSD
#include <SPI.h>
#include <max7456.h>
#define redLed 3
#define greenLed 4

//BAROMETER
#include "MS5611.h"
#include <KalmanFilter.h>
#include <AQMath.h>
#include <MatrixMath.h>
#include <kalmanvert.h>

//MAGNETO
#include <HMC5883L.h>

//MOTOR
#include "Servo.h"

// elrs
#include "CRSFforArduino.hpp"

//GPS
#include <Arduino.h>
#include <HardwareTimer.h>

//kompas
#include <HMC5883L.h>

//Neural Network
#include "AdaptiveController.h"

// Konstanta radius bumi dalam meter
#define EARTH_RADIUS_M 6371000.0

TwoWire Wire2(PB11,PB10);
//================================================= ELRS ======================================
CRSFforArduino *crsf = nullptr;
HardwareSerial Serial6(PD6,PD5); // define serial pin in this section
int rcChannelCount = crsfProtocol::RC_CHANNEL_COUNT;
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels); // is it important?

//================================================= BATERAI ======================================
float vin, vout, adc;

//================================================ IMU ==========================================
MPU6050 mpu;
// MPU6050 accelgyro;
int16_t mx, my, mz;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float gyroX_filt, gyroY_filt, gyroZ_filt;
float accelX, accelY, accelZ;

float G_Dt           = 0.005;
//----------------------Tuning Drone------------------------------------------

float Kp_roll       = 4;              float Kp_roll2  = 2.5;
float Ki_roll       = 0.35;              float Ki_roll2  = 0;//0.35`` 
float Kd_roll       = 0;                float Kd_roll2  = 0;//0.9

float Kp_pitch      = 3.5;                float Kp_pitch2 = 1.55;
float Ki_pitch      = 0.47;              float Ki_pitch2 = 0;//0.47
float Kd_pitch      = 0;                float Kd_pitch2 = 0;

float Kp_yaw        = 1.5;                float Kp_yaw2   = 0.6;
float Ki_yaw        = 0;                float Ki_yaw2   = 0;
float Kd_yaw        = 0;                float Kd_yaw2   = 0;

float Kp_alt        = 10;                float Kp_alt2   = 0;
float Ki_alt        = 0;                float Ki_alt2   = 0;
float Kd_alt        = 4;                float Kd_alt2   = 0;

float Kp_pos        = 0.3;              float Kp_vel   = 0.05;
float Ki_pos        = 0;                float Ki_vel   = 0;
float Kd_pos        = 0;                float Kd_vel   = 0;

//----------------------------------------------------------------------------
//----------------------Tuning Plane------------------------------------------
float Kp_roll3       = 0;              float Kp_roll4  = 0;
float Ki_roll3       = 0;              float Ki_roll4  = 0;
float Kd_roll3       = 0;                float Kd_roll4  = 0;

float Kp_pitch3      = 0;                float Kp_pitch4 = 0;
float Ki_pitch3      = 0;              float Ki_pitch4 = 0;
float Kd_pitch3      = 0;                float Kd_pitch4 = 0;

float Kp_alt3      = 0;                float Kp_alt4 = 0;
float Ki_alt3      = 0;              float Ki_alt4 = 0;
float Kd_alt3      = 0;                float Kd_alt4 = 0;

float gain_yaw1       = 2;
float gain_latitude1  = 0;
float gain_longitude1 = 0;

float gain_vel_yaw1   = 0.35;//0.35
float gain_vel_vel   = 0;
float gain_vel_long1  = 0;
//-----------------------------------------------------------------------------

float last_pitch     = 0;
float last_roll      = 0;
float last_yaw       = 0;

float last_pitch1     = -5;
float last_roll1      = -1.2;

double rad_yaw, rad_pitch, rad_roll;
double roll_kalman, pitch_kalman, yaw_kalman;
double accum_roll  = 0;
double accum_pitch = 0;
double k_acc = 0;
double k_gps = 0;
double yaw_deg;
double pitch_deg;
double roll_deg;
double psi_deg, theta_deg, phi_deg;
float pitchAcc;
float yaw_reference, set_yaw, yawPrev, yaw_head, yaw_control, yawtrans;
int yaw_ref = 0;
#define INTERRUPT_PIN PB12  // use pin 2 on Arduino Uno & most boards
#define LED_PIN PC13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
float dt,error_roll,error_pitch,error_yaw,error_alt,error_pos,error_vel,integral_error_roll,integral_error_pitch,integral_error_yaw,
      integral_error_alt,integral_error_pos,integral_error_vel,derivative_error_roll,derivative_error_pitch,derivative_error_yaw,
      derivative_error_alt,derivative_error_pos,derivative_error_vel,last_error_roll,last_error_pitch,last_error_yaw,last_error_alt,
      last_error_pos,last_error_vel;
float error_roll1, integral_error_roll1, derivative_error_roll1, last_error_roll1,error_pitch1, integral_error_pitch1, derivative_error_pitch1, last_error_pitch1,
      error_yaw1, integral_error_yaw1, derivative_error_yaw1, last_error_yaw1,error_alt1, integral_error_alt1, derivative_error_alt1, last_error_alt1;
float error_roll2, integral_error_roll2, derivative_error_roll2, last_error_roll2,error_pitch2, integral_error_pitch2, derivative_error_pitch2, last_error_pitch2,
      error_yaw2, integral_error_yaw2, derivative_error_yaw2, last_error_yaw2,error_alt2, integral_error_alt2, derivative_error_alt2, last_error_alt2;
float error_roll3, integral_error_roll3, derivative_error_roll3, last_error_roll3,error_pitch3, integral_error_pitch3, derivative_error_pitch3, last_error_pitch3,
      error_yaw3, integral_error_yaw3, derivative_error_yaw3, last_error_yaw3,error_alt3, integral_error_alt3, derivative_error_alt3, last_error_alt3;
float PID_value_roll,PID_value_pitch,PID_value_yaw,PID_value_alt,PID_value_pos,PID_value_vel,Proll,Ppitch,Pyaw,Palt,Ppos,Pvel,Iroll,Ipitch,Iyaw,Ialt,Ipos,Ivel,
       Droll,Dpitch,Dyaw,Dalt,Dpos,Dvel,
       PID_virtual_roll,PID_virtual_pitch,PID_virtual_yaw,PID_virtual_alt,PID_virtual_pos;
float Proll1, Iroll1, Droll1, Ppitch1, Ipitch1, Dpitch1, Pyaw1, Iyaw1, Dyaw1,Palt1, Ialt1, Dalt1;
float Proll2, Iroll2, Droll2, Ppitch2, Ipitch2, Dpitch2, Pyaw2, Iyaw2, Dyaw2,Palt2, Ialt2, Dalt2;
float Proll3, Iroll3, Droll3, Ppitch3, Ipitch3, Dpitch3, Pyaw3, Iyaw3, Dyaw3,Palt3, Ialt3, Dalt3;
float PID_virtual_roll2,PID_virtual_pitch2,PID_virtual_alt2,PID_value_roll2,PID_value_pitch2;;
int PID_max = 400;
int PID_max2 = 30;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//================================================ REMOTE ==========================================
volatile int channel1         = 0;
volatile int roll_channel     = 0;
volatile int channel2         = 0;
volatile int throttle_channel = 0;
volatile int channel3         = 0;
volatile int pitch_channel    = 0;
volatile int channel4         = 0;
volatile int yaw_channel      = 0;
volatile int channel5         = 0;
volatile int ch5_channel      = 0;
volatile int channel6         = 0;
volatile int ch6_channel      = 0;
volatile int channel7         = 0;
volatile int ch7_channel      = 0;
volatile int channel8         = 0;
volatile int ch8_channel      = 0;
volatile int channel9         = 0;
volatile int ch9_channel      = 0;

int ch5, ch6, ch7, ch8, ch9, throttle, pst_mode, alt_mode,head_mode,head_mode1, statusmode;
int transisi = 0;
int armStatus = 0;
int pulse_escx=0;
int pulse_escy=0; 
int pulse_esc1=0;
int pulse_esc2=0; 
int pulse_esc3 =0; 
int pulse_esc4=0;
int pulse_esc11=60;
int pulse_esc22=0; 
int pulse_esc33=0; 
int pulse_esc44=60;
int roll_input, pitch_input, yaw_input, alt_input;
int roll_input1, pitch_input1, yaw_input1;
int pitch_transisi,pitch_transisi1,pitch_transisi2;
float pitch_ch_previous, roll_ch_previous, yaw_ch_previous, throt_ch_previous;
char inChar;
bool mati = 0;
int statusAktif = 0;
int transisi_servo = 5;

//================================================ MOTOR ===========================================
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

Servo motA, motB, motC, motD, myservoX, myservoY, myservoA, myservoB;
float rollControl, pitchControl, yawControl, altControl,yawFiltered;
float rollControl2, pitchControl2, yawControl2, altControl2;
unsigned long pulse_length_esc1 = 1000,
              pulse_length_esc2 = 1000,
              pulse_length_esc3 = 1000,
              pulse_length_esc4 = 1000;

//================================================SERVO===========================================

int servoAngleInit1 = 85;
int servoAngleInit2 = 90;

int servoAngleInitA = 90;
int servoAngleInitB = 97;

int servo1_up   = 55;
int servo1_down = 115;
int servo2_up   = 60;
int servo2_down = 120;

int servo1_up1   = 50;
int servo1_down1 = 130;
int servo2_up1   = 63;
int servo2_down1 = 143;

int pulse_length_servo1, pulse_length_servo2;
int Servo1, Servo2;
float rollControlServo, pitchControlServo, yawControlServo;

//=============================================BAROMETER==========================================================================---- 

uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location, start;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;


float altitude_m;
float altitude_cm;

uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;

int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
uint32_t loop_timer;
float top_line, bottom_line;

uint8_t MS5611_address = 0x77;             //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.

  float lastAltitude = 0;
  float totalAltitudeChange = 0;
  unsigned long lastTime = millis();
  float lastRate = 0;

float AltitudeBaroGround,z_position,alt_ref,estimation_altitude,altitudeControl_ref;
int axis;
// ======================================================== COMPAS ======================================================================
// Alamat I2C untuk sensor IST8310
const int IST8310_I2C_ADDR = 0x0E;

// Alamat Register
const int WAI_REG = 0x00;        // Register "Who Am I"
const int DEVICE_ID = 0x10;      // Nilai yang diharapkan dari WAI_REG

const int OUTPUT_X_L_REG = 0x03; // Data LSB X
const int OUTPUT_X_H_REG = 0x04; // Data MSB X
const int OUTPUT_Y_L_REG = 0x05; // Data LSB Y
const int OUTPUT_Y_H_REG = 0x06; // Data MSB Y
const int OUTPUT_Z_L_REG = 0x07; // Data LSB Z
const int OUTPUT_Z_H_REG = 0x08; // Data MSB Z

const int CNTL1_REG = 0x0A;      // Register Kontrol 1
const int CNTL2_REG = 0x0B;      // Register Kontrol 2

const int AVGCNTL_REG = 0x41;    // Register untuk Averaging

// HMC5883L compass;
float headingDegrees,heading;
float fixedHeadingDegrees;
float compensateRoll, compensatePitch;
float cosComRoll, sinComRoll, cosComPitch, sinComPitch;
float Yh, Xh, Ymag_correct, Xmag_correct;
float setHeading;
float heading_reference,heading_control;
int head = 0;
const float x_offset = -2; 
const float y_offset = 32;
const float x_scale  = 1;
const float y_scale  = 1;
//=============================================== GPS ================================================
// Konstanta radius bumi dalam meter
#define EARTH_RADIUS_M 6371000.0
#define RX_PIN PD9
#define TX_PIN PD8
HardwareSerial Serial2(PD9, PD8); //RX TX3
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_CLASS_NAV 0x01
#define UBX_ID_PVT    0x07
#define UBX_LEN_PVT   92

float distance_cm,target_pitch,target_roll,target_lat,target_lon,current_heading,angle_to_target,bearing_deg,gps_speed;
bool mode_baru_aktif;
int fwhead1;
double prevLat = 0, prevLon = 0;
unsigned long prevMillis = 0;
double gps_lat,gps_lon,gps_sats;
double jarak,waktu,alt_m;

// Pastikan tidak ada padding byte pada struct (offset harus persis)
#pragma pack(push, 1)
struct NAV_PVT_t {
  uint32_t iTOW;     // 0  ms of week
  uint16_t year;     // 4
  uint8_t  month;    // 6
  uint8_t  day;      // 7
  uint8_t  hour;     // 8
  uint8_t  min;      // 9
  uint8_t  sec;      // 10
  uint8_t  valid;    // 11
  uint32_t tAcc;     // 12
  int32_t  nano;     // 16
  uint8_t  fixType;  // 20 (0..5)
  uint8_t  flags;    // 21
  uint8_t  flags2;   // 22
  uint8_t  numSV;    // 23
  int32_t  lon;      // 24 (1e-7 deg)
  int32_t  lat;      // 28 (1e-7 deg)
  int32_t  height;   // 32 (mm)
  int32_t  hMSL;     // 36 (mm)
  uint32_t hAcc;     // 40 (mm)
  uint32_t vAcc;     // 44 (mm)
  int32_t  velN;     // 48 (mm/s)
  int32_t  velE;     // 52 (mm/s)
  int32_t  velD;     // 56 (mm/s)
  int32_t  gSpeed;   // 60 (mm/s)
  int32_t  headMot;  // 64 (1e-5 deg)
  uint32_t sAcc;     // 68 (mm/s)
  uint32_t headAcc;  // 72 (1e-5 deg)
  uint16_t pDOP;     // 76
  uint8_t  reserved1[6]; // 78..83
  int32_t  headVeh;  // 84 (1e-5 deg)
  int16_t  magDec;   // 88 (1e-2 deg)
  uint16_t magAcc;   // 90 (1e-2 deg)
};
#pragma pack(pop)
static_assert(sizeof(NAV_PVT_t) == UBX_LEN_PVT, "NAV_PVT size must be 92 bytes");

// State machine
enum ParseState : uint8_t {
  WAIT_SYNC1, WAIT_SYNC2, WAIT_CLASS, WAIT_ID, WAIT_LEN1, WAIT_LEN2,
  WAIT_PAYLOAD, WAIT_CK_A, WAIT_CK_B
};

ParseState state = WAIT_SYNC1;
uint8_t  ubxClass = 0, ubxId = 0;
uint16_t ubxLen = 0, payloadCnt = 0;
uint8_t  payload[UBX_LEN_PVT];
uint8_t  ckA = 0, ckB = 0, rxCkA = 0, rxCkB = 0;

NAV_PVT_t navPvt;
bool navPvtReady = false;

inline void ckUpdate(uint8_t b) {
  ckA = ckA + b;
  ckB = ckB + ckA;
}

//================================================ WAYPOINT ===========================================

#define DEG2RAD 0.01746031746
#define RAD2DEG 57.2727272727
float dLon, brlat1, brlat2, a1bearing, a2bearing, bearing, bearingHeading;
float lat1, lat2, long1, long2;
float latitude_init, longitude_init;

float WaypointLatitude[1]  = {-7.752695};  
float WaypointLongitude[1] = {110.209059}; 
//float WaypointLatitude[3]  = {-7.751326,-7.750563,-7.750566  }; 
//float WaypointLongitude[3] = {110.348207,110.348688, 110.349594}; 
int waypointCount = 1;
int cek_heading = 0;
int tracking, init_Home = 0, waypoint = 0, waypoint_index = 0;

//================================================ TIMER ==============================================
unsigned long timerGPS;
unsigned long timerGPS2;
unsigned long timerXV;
unsigned long waypointTime_current, waypointTime_previous;
unsigned long currentTime_compass, previousTime_compass;
unsigned long currentTime_carometer, previousTime_barometer;
unsigned long currentTime_GPS, previousTime_GPS;
unsigned long currentTime_gps, previousTime_gps;
unsigned long currentTime_pid, previousTime_pid;
unsigned long transisiTime_current, transisiTime_previous;
unsigned long transisiTime_current1;

uint32_t timeProgram, previousTimeProgram;
uint32_t timeServo, previousTimeServo;

// =========================================================================
//         BOBOT & BIAS FINAL DARI SIMULASI (UNTUK ROLL & PITCH)
// =========================================================================

// --- Bobot untuk Sumbu ROLL (Ganti dengan hasil terbaik Anda) ---
const float weights_ih_roll[INPUT_NEURONS][HIDDEN_NEURONS] = {
    {2.537, 4.378, 0.083, -0.154, 3.697, 0.008, 17.964, 5.412},
    {2.448, 4.144, -0.002, 0.089, 3.441, 0.073, 17.136, 5.218},
    {-0.092, 0.161, -0.013, 0.045, 0.162, -0.062, 0.706, 0.364}
};
const float weights_ho_roll[HIDDEN_NEURONS][OUTPUT_NEURONS] = {
    {0.655}, {-0.177}, {0.026}, {-0.064}, {-0.020}, {0.135}, {0.560}, {0.185}
};
const float bias_h_roll[HIDDEN_NEURONS] = {2.633, 2.906, -0.037, -0.049, 2.592, -0.064, 13.559, 4.126};
const float bias_o_roll[OUTPUT_NEURONS] = {-4.176};

// --- Bobot untuk Sumbu PITCH (Ganti dengan hasil terbaik Anda) ---
const float weights_ih_pitch[INPUT_NEURONS][HIDDEN_NEURONS] = {
    {0.0419, 0.3599, 0.4946, 0.0916, 0.5315, 0.4539, -0.0248, 0.4286},
    {-0.0484, 0.4873, 0.2803, 0.0704, 0.5461, 0.4451, 0.1018, 0.4270},
    {0.0178, 0.0906, -0.1254, 0.0865, -0.1460, -0.1074, -0.0158, 0.0138}
};
const float weights_ho_pitch[HIDDEN_NEURONS][OUTPUT_NEURONS] = {
    {-0.0380}, {-19.570}, {-12.006}, {-0.0976}, {-21.671}, {-11.858}, {0.1287}, {0.6235}
};
const float bias_h_pitch[HIDDEN_NEURONS] = {-0.0618, 14.857, 12.972, -0.0853, 15.262, 12.930, -0.0955, 7.897};
const float bias_o_pitch[OUTPUT_NEURONS] = {-9.917};


// =========================================================================
//                      PARAMETER & OBJEK KONTROLER
// =========================================================================
const float LEARNING_RATE_ROLL = 0.0001; // Ganti dengan nilai optimal roll
const float LEARNING_RATE_PITCH = 0.01;   // Ganti dengan nilai optimal pitch
const float MODEL_ZETA = 0.9;
const float MODEL_OMEGA_N = 50.0;

// Buat DUA objek kontroler, satu untuk setiap sumbu, dengan bobotnya masing-masing
AdaptiveController roll_controller(LEARNING_RATE_ROLL, MODEL_ZETA, MODEL_OMEGA_N, 
                                   weights_ih_roll, weights_ho_roll, bias_h_roll, bias_o_roll);
                                   
AdaptiveController pitch_controller(LEARNING_RATE_PITCH, MODEL_ZETA, MODEL_OMEGA_N, 
                                    weights_ih_pitch, weights_ho_pitch, bias_h_pitch, bias_o_pitch);

// Variabel Waktu
unsigned long last_time_micros = 0;

// Placeholder untuk variabel sistem
// float target_roll = 0.0, target_pitch = 0.0;

HardwareSerial Serial1(PA1,PA0);
//==================================================================================================================================================
// int vout = 23;
void setup() {
  Serial1.begin(57600);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();
  Wire2.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  // Inisialisasi sensor
  init_MPU();
  elrsinit();
  motor_setup();
 baro_initialized();
  compass_init();
  init_gps();
 osd_init();
  // Serial2.setRxBufferSize(512); // kalau core STM32 support
}

void loop() {
  // static uint32_t lastIMU=0;
  // if (millis()-lastIMU >= 20) {  // 50 Hz
  //   lastIMU = millis();

  // }
  get_YPR();
  compass_update();
  SerialEvent();
  mapremote();
  update_motor();
  update_gps();
  osd_baca();
  updateBaro();

//  roll_controller.compute(roll_input, roll_deg, gx); pitch_controller.compute(pitch_input, pitch_deg, gy);
//  roll_controller.learn(roll_input, roll_deg, gx);
//  pitch_controller.learn(pitch_input, pitch_deg, gy);
//  Kp_roll2 = roll_controller.getKp(); Kp_pitch2 = pitch_controller.getKp();

  timeProgram = micros();
  if (timeProgram - previousTimeProgram >= 100000)
  {
    print_out();
    previousTimeProgram = micros();
  }
  
}
