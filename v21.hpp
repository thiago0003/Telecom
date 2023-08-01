#ifndef V21_HPP
#define V21_HPP

#include <functional>
#include "config.hpp"
//#include <math.h>

class V21_RX
{
public:
    V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples)
        :omega_mark(omega_mark),omega_space(omega_space),get_digital_samples(get_digital_samples) {};
    void demodulate(const float *in_analog_samples, unsigned int n);
private:
    float v0i_prev = 0.; 
    float v0r_prev = 0.;
    float v1i_prev = 0.; 
    float v1r_prev = 0.;
    float decision_prev = 0.;
    float decision_prev_prev = 0.;
    float filtered_decision_prev = 0.; 
    float filtered_decision_prev_prev = 0.;
    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;
    float ring_buffer[SAMPLES_PER_SYMBOL +1] = {};
    int ring_buffer_it = 0;
    int counter = 0;

    float omega_mark, omega_space;
};

class V21_TX
{
public:
    V21_TX(float omega_mark, float omega_space) :omega_mark(omega_mark),omega_space(omega_space),phase(0.f) {};
    void modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n);
private:
    float omega_mark, omega_space;
    float phase;
};

#endif
