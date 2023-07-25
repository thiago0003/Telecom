#include <math.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <deque>
#include <stdexcept>
#include "v21.hpp"
#include "uart.hpp"
#include "config.hpp"

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    unsigned int digital_samples[n];

    FILE *dc   = fopen("../dc.raw", "wb");
    FILE *f_dc = fopen("../f_dc.raw", "wb");
    FILE *f_v0 = fopen("../v0.raw", "wb");
    FILE *f_v1 = fopen("../v1.raw", "wb");

    static enum class State {
        NO_STATE,
        BEARER_NOT_DETECT,
        BEARER_DETECT
    } state = State::BEARER_NOT_DETECT;

    unsigned int  R = 300;
    unsigned int fs = 48000;
    float         T = 1/fs;
    unsigned int  L = std::floor( fs / R );

    int bit_num = 0;

    float omega0=2*M_PI*1850;
    float omega1=2*M_PI*1650;

    omega_mark = omega0;
    omega_space = omega1;

    float butter_a[3] = { 1.0        , -1.94447766 , 0.94597794 };
    float butter_b[3] = { 0.00037507 ,  0.00075014 , 0.00037507 };

    float v0i, v0r, v1i, v1r, filtered_decision, decision;
    float r = 0.99;

    float v0, v1;

    // substitua o loop abaixo, que gera um sinal UART constantemente ocioso, pelo seu código
    for (int i = 0; i < n; i++) {
        int mod = i % (SAMPLES_PER_SYMBOL + 1);
        ring_buffer[mod] = in_analog_samples[i];

        v0r = ring_buffer[mod] - pow(r,L) * cos(omega0*L*T)*ring_buffer[mod] + r*cos(omega0*T)*v0r_prev - r*sin(omega0*T)*v1i_prev;
        v0i = -1*pow(r,L)*sin(omega0*L*T)*ring_buffer[mod] + r*cos(omega0*T)*v0i_prev + r*sin(omega0*T)*v1r_prev;
        v1r = ring_buffer[mod] - pow(r,L) * cos(omega1*L*T)*ring_buffer[mod] + r*cos(omega1*T)*v0r_prev - r*sin(omega1*T)*v1i_prev;
        v1i = -1*pow(r,L)*sin(omega1*L*T)*ring_buffer[mod] + r*cos(omega1*T)*v0i_prev + r*sin(omega1*T)*v1r_prev;

        // define-se um sinal de decisão, que se for positivo indica que devemos gerar nível lógico alto
        decision = v1r*v1r + v1i*v1i - v0r*v0r - v0i*v0i;
        v0 = v0r*v0r + v0i*v0i;
        v1 = v1r*v1r + v1i*v1i;

        // Passa-se o sinal por um filtro passa-baixas com corte em 300 Hz.
        filtered_decision = butter_b[0] * decision + butter_b[1] * decision_prev + butter_b[2] * decision_prev_prev - butter_a[1] * filtered_decision_prev - butter_a[2] * filtered_decision_prev_prev;

        fwrite(&v0, 1, sizeof(float), f_v0);
        fwrite(&v1, 1, sizeof(float), f_v1);

        //Estado portadora detectada 
        if(filtered_decision > 120.)
        {
            bit_num = 50;
            state = State::BEARER_NOT_DETECT;  
        }
        else
        {
            bit_num--;
            if(bit_num == 0.)
                state = State::BEARER_DETECT;
        }

        if(state == State::BEARER_DETECT)
            digital_samples[i] = 1;
        else if(state == State::BEARER_NOT_DETECT)
            digital_samples[i] = 0;
    
        v0i_prev = v0i;
        v0r_prev = v0r;
        v1i_prev = v1i;
        v1r_prev = v1r;
        filtered_decision_prev_prev = filtered_decision_prev;
        decision_prev_prev = decision_prev;
    }

    fwrite(&decision, n, sizeof(float), dc);
    fwrite(&filtered_decision, n, sizeof(float), f_dc);

    fclose(dc);
    fclose(f_dc);
    fclose(f_v0);
    fclose(f_v1);

    get_digital_samples(digital_samples, n);
}

void V21_TX::modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n)
{
    while (n--) {
        *out_analog_samples++ = sin(phase);
        phase += (*in_digital_samples++ ? omega_mark : omega_space) * SAMPLING_PERIOD;

        // evita que phase cresça indefinidamente, o que causaria perda de precisão
        phase = remainder(phase, 2*M_PI);
    }
}
