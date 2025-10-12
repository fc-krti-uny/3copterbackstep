#ifndef ADAPTIVE_CONTROLLER_H
#define ADAPTIVE_CONTROLLER_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

// --- Definisi Arsitektur NN (sesuaikan jika Anda mengubahnya di simulasi) ---
const int INPUT_NEURONS = 3;
const int HIDDEN_NEURONS = 8;
const int OUTPUT_NEURONS = 1;

// --- Class Internal untuk Neural Network ---
class SimpleNN {
public:
    // Constructor untuk menerima bobot yang sudah terlatih
    SimpleNN(float learningRate, 
             const float weights_ih_init[INPUT_NEURONS][HIDDEN_NEURONS], 
             const float weights_ho_init[HIDDEN_NEURONS][OUTPUT_NEURONS], 
             const float bias_h_init[HIDDEN_NEURONS], 
             const float bias_o_init[OUTPUT_NEURONS]);

    float forward_pass(float inputs[INPUT_NEURONS]);
    void update_weights(float adaptation_error);

private:
    float weights_ih[INPUT_NEURONS][HIDDEN_NEURONS];
    float weights_ho[HIDDEN_NEURONS][OUTPUT_NEURONS];
    float bias_h[HIDDEN_NEURONS];
    float bias_o[OUTPUT_NEURONS];
    float hidden_outputs[HIDDEN_NEURONS];
    float last_inputs[INPUT_NEURONS];
    float learning_rate;
    float relu(float x);
    float relu_derivative(float x);
};

// --- Class Internal untuk Model Referensi ---
class ReferenceModel {
public:
    ReferenceModel(float zeta, float omega_n, float dt);
    float update(float r);

private:
    float a1, a2, b0, b1, b2, gain;
    float ym_prev1, ym_prev2, r_prev1, r_prev2;
};

// --- Class Utama Library Anda ---
class AdaptiveController {
public:
    // Constructor untuk menerima semua parameter dan bobot
    AdaptiveController(float learning_rate, float model_zeta, float model_omega_n,
                       const float weights_ih_init[INPUT_NEURONS][HIDDEN_NEURONS], 
                       const float weights_ho_init[HIDDEN_NEURONS][OUTPUT_NEURONS], 
                       const float bias_h_init[HIDDEN_NEURONS], 
                       const float bias_o_init[OUTPUT_NEURONS]);
                       
    float compute(float target_angle, float actual_angle, float actual_rate);
    void learn(float target_angle, float actual_angle, float actual_rate);
    float getKp();

private:
    SimpleNN nn;
    ReferenceModel model;
    float last_Kp;
};

#endif