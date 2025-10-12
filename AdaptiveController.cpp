#include "AdaptiveController.h"
#include <math.h>   // Untuk fungsi sqrt, exp, cos, sin
#include <string.h> // Untuk memcpy

// ==========================================================
// Implementasi Class SimpleNN
// ==========================================================
SimpleNN::SimpleNN(float learningRate, const float weights_ih_init[INPUT_NEURONS][HIDDEN_NEURONS], 
                   const float weights_ho_init[HIDDEN_NEURONS][OUTPUT_NEURONS], 
                   const float bias_h_init[HIDDEN_NEURONS], const float bias_o_init[OUTPUT_NEURONS]) 
: learning_rate(learningRate) 
{
    // Cek apakah bobot yang diberikan adalah placeholder (misal, semuanya nol)
    // Jika ya, lakukan inisialisasi Glorot. Jika tidak, gunakan bobot dari simulasi.
    bool use_glorot_init = true; 
    if (weights_ih_init[0][0] != 0.0) { // Cek sederhana, jika bobot pertama tidak nol
        use_glorot_init = false;
    }

    if (use_glorot_init) {
        // --- INISIALISASI GLOROT/XAVIER (BARU & BENAR) ---
        // Menjaga agar neuron tidak saturasi di awal

        // Inisialisasi Input -> Hidden
        float limit_ih = sqrt(6.0 / (INPUT_NEURONS + HIDDEN_NEURONS));
        for (int i = 0; i < INPUT_NEURONS; i++) {
            for (int j = 0; j < HIDDEN_NEURONS; j++) {
                weights_ih[i][j] = ((float)rand() / RAND_MAX) * 2 * limit_ih - limit_ih;
            }
        }

        // Inisialisasi Hidden -> Output
        float limit_ho = sqrt(6.0 / (HIDDEN_NEURONS + OUTPUT_NEURONS));
        for (int i = 0; i < HIDDEN_NEURONS; i++) {
            for (int j = 0; j < OUTPUT_NEURONS; j++) {
                weights_ho[i][j] = ((float)rand() / RAND_MAX) * 2 * limit_ho - limit_ho;
            }
        }
        
        // Inisialisasi semua bias ke nol adalah praktik terbaik
        memset(bias_h, 0, sizeof(bias_h));
        memset(bias_o, 0, sizeof(bias_o));

    } else {
        // Gunakan bobot dari simulasi seperti sebelumnya
        memcpy(weights_ih, weights_ih_init, sizeof(weights_ih));
        memcpy(weights_ho, weights_ho_init, sizeof(weights_ho));
        memcpy(bias_h, bias_h_init, sizeof(bias_h));
        memcpy(bias_o, bias_o_init, sizeof(bias_o));
    }
}

float SimpleNN::relu(float x) {
    // Ganti nama fungsi menjadi 'activation' dan gunakan tanh
    return tanhf(x); // tanhf adalah versi float dari tanh
}
float SimpleNN::relu_derivative(float x) {
    // x di sini adalah output dari aktivasi (yaitu, hasil dari tanhf)
    // Turunan yang benar adalah 1 - (output_aktivasi)^2
    return 1.0 - (x * x);
}

float SimpleNN::forward_pass(float inputs[INPUT_NEURONS]) {
    for(int i=0; i<INPUT_NEURONS; i++) { last_inputs[i] = inputs[i]; }
    for (int j = 0; j < HIDDEN_NEURONS; j++) {
        float sum = 0;
        for (int i = 0; i < INPUT_NEURONS; i++) { sum += inputs[i] * weights_ih[i][j]; }
        sum += bias_h[j];
        hidden_outputs[j] = relu(sum);
    }
    float output_sum = 0;
    for (int j = 0; j < HIDDEN_NEURONS; j++) { output_sum += hidden_outputs[j] * weights_ho[j][0]; }
    output_sum += bias_o[0];
    return output_sum;
}

// Di dalam AdaptiveController.cpp

// GANTI FUNGSI LAMA YANG ERROR DENGAN VERSI BERSIH INI
void SimpleNN::update_weights(float adaptation_error) {
    
    // --- Kode Asli Backpropagation ---
    float gradient_clip_value = 1.0;
    float output_gradient = constrain(adaptation_error, -gradient_clip_value, gradient_clip_value);
    
    // Update bobot dari Hidden ke Output
    for (int j = 0; j < HIDDEN_NEURONS; j++) {
        weights_ho[j][0] += learning_rate * output_gradient * hidden_outputs[j];
    }
    bias_o[0] += learning_rate * output_gradient;

    // Hitung gradien untuk Hidden Layer
    float hidden_gradients[HIDDEN_NEURONS];
    for (int j = 0; j < HIDDEN_NEURONS; j++) {
        float error_propagated = constrain(output_gradient * weights_ho[j][0], -gradient_clip_value, gradient_clip_value);
        
        // Gunakan turunan dari fungsi aktivasi yang benar (1 - x*x untuk tanh)
        hidden_gradients[j] = error_propagated * (1.0 - (hidden_outputs[j] * hidden_outputs[j]));
    }

    // Update bobot dari Input ke Hidden
    for (int i = 0; i < INPUT_NEURONS; i++) {
        for (int j = 0; j < HIDDEN_NEURONS; j++) {
            weights_ih[i][j] += learning_rate * hidden_gradients[j] * last_inputs[i];
        }
    }
    
    // Update bias untuk Hidden Layer
    for (int j = 0; j < HIDDEN_NEURONS; j++) {
        bias_h[j] += learning_rate * hidden_gradients[j];
    }
}

// ==========================================================
// Implementasi Class ReferenceModel
// ==========================================================
ReferenceModel::ReferenceModel(float zeta, float omega_n, float dt) {
    float wd = omega_n * sqrt(1 - (zeta * zeta));
    float e_zt = exp(-zeta * omega_n * dt);
    float cos_wd = cos(wd * dt);
    float sin_wd = sin(wd * dt);

    a1 = -2 * e_zt * cos_wd;
    a2 = e_zt * e_zt;
    b0 = omega_n * omega_n;
    b1 = e_zt * ( (zeta * omega_n / wd) * sin_wd - cos_wd );
    b2 = e_zt * e_zt;
    
    if ((1 + a1 + a2) != 0) {
        gain = (dt * dt * (1 + b1 + b2)) / (1 + a1 + a2);
    } else {
        gain = 0;
    }
    
    ym_prev1 = 0.0; ym_prev2 = 0.0; r_prev1 = 0.0; r_prev2 = 0.0;
}

float ReferenceModel::update(float r) {
    float ym = gain*b0*r + gain*b1*r_prev1 + gain*b2*r_prev2 - a1*ym_prev1 - a2*ym_prev2;
    ym_prev2 = ym_prev1; ym_prev1 = ym;
    r_prev2 = r_prev1; r_prev1 = r;
    return ym;
}

// ==========================================================
// Implementasi Class Utama AdaptiveController
// ==========================================================
AdaptiveController::AdaptiveController(float learning_rate, float model_zeta, float model_omega_n,
                                       const float weights_ih_init[INPUT_NEURONS][HIDDEN_NEURONS], 
                                       const float weights_ho_init[HIDDEN_NEURONS][OUTPUT_NEURONS], 
                                       const float bias_h_init[HIDDEN_NEURONS], const float bias_o_init[OUTPUT_NEURONS]) 
:   nn(learning_rate, weights_ih_init, weights_ho_init, bias_h_init, bias_o_init), 
    model(model_zeta, model_omega_n, 0.01) // dt bisa diupdate dinamis jika perlu
{
    last_Kp = 0.0;
}

float AdaptiveController::compute(float target_angle, float actual_angle, float actual_rate) {
    const float MAX_ANGLE = 30.0;  // Perkiraan sudut maks
    const float MAX_RATE = 200.0; // Perkiraan kecepatan sudut maks

    // Normalisasi input BARU
    float norm_target_angle = constrain(target_angle / MAX_ANGLE, -1.0, 1.0);
    float norm_actual_angle = constrain(actual_angle / MAX_ANGLE, -1.0, 1.0);
    float norm_actual_rate = constrain(actual_rate / MAX_RATE, -1.0, 1.0);
    float nn_inputs[] = {norm_target_angle, norm_actual_angle, norm_actual_rate};

    float Kp = nn.forward_pass(nn_inputs);
    Kp = constrain(Kp, 1.5, 2.5);
    
    last_Kp = Kp;

    // Hitung sinyal P-controller (target - rate)
    // Ini perlu disesuaikan dengan logika backstepping Anda
    float virtual_control = 4.5 * (target_angle - actual_angle); // PID Virtual sederhana
    float error = virtual_control - actual_rate;
    return Kp * error;
}

void AdaptiveController::learn(float target_angle, float actual_angle, float actual_rate) {
    // Abaikan semua perhitungan e_adapt untuk tes ini
    /*
    float virtual_control = 4.5 * (target_angle - actual_angle);
    float ideal_rate = model.update(virtual_control);
    float e_adapt = actual_rate - ideal_rate;
    */

    // --- TES BELAJAR PAKSA ---
    // Berikan sinyal error konstan yang kuat.
    float forced_error_signal = 1.0; 

    // Panggil update_weights dengan sinyal paksa ini
    nn.update_weights(forced_error_signal);
}

float AdaptiveController::getKp(){
    return last_Kp;
}