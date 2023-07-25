#include <math.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <deque>
#include <stdexcept>
#include "v21.hpp"
#include "uart.hpp"
#include "config.hpp"
#include "ring_buffer.hpp"

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    unsigned int digital_samples[n];

    FILE *dc = fopen("../dc.raw", "wb");
    FILE *f_dc = fopen("../f_dc.raw", "wb");
    FILE *f_v0 = fopen("../v0.raw", "wb");
    FILE *f_v1 = fopen("../v1.raw", "wb");

    CircularBuffer ring_buffer(SAMPLES_PER_SYMBOL);

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

    float v0i[n], v0r[n], v1i[n], v1r[n], filtered_decision[n], decision[n];
    float r = 0.99;

    float v0, v1;

    memset(v0i, 0, sizeof(v0i));
    memset(v0r, 0, sizeof(v0r));
    memset(v1i, 0, sizeof(v1i));
    memset(v1r, 0, sizeof(v1r));
    memset(decision, 0, sizeof(decision));

    // substitua o loop abaixo, que gera um sinal UART constantemente ocioso, pelo seu código
    for (int i = 0; i < n; i++) {
        ring_buffer.push_back(in_analog_samples[i]);
        int mod = i % SAMPLES_PER_SYMBOL;

        v0r[i] = ring_buffer[mod] - pow(r,L) * cos(omega0*L*T)*ring_buffer[mod] + r*cos(omega0*T)*(i - 1 >= 0 ? v0r[i-1] : 0) - r*sin(omega0*T)*(i - 1 >= 0 ? v0i[i-1] : 0);
        v0i[i] = -1*pow(r,L)*sin(omega0*L*T)*ring_buffer[mod] + r*cos(omega0*T)*(i - 1 >= 0 ? v0i[i-1] : 0) + r*sin(omega0*T)*(i - 1 >= 0 ? v0r[i-1] : 0);
        v1r[i] = ring_buffer[mod] - pow(r,L) * cos(omega1*L*T)*ring_buffer[mod] + r*cos(omega1*T)*(i - 1 >= 0 ? v1r[i-1] : 0) - r*sin(omega1*T)*(i - 1 >= 0 ? v1i[i-1] : 0);
        v1i[i] = -1*pow(r,L)*sin(omega1*L*T)*ring_buffer[mod] + r*cos(omega1*T)*(i - 1 >= 0 ? v1i[i-1] : 0) + r*sin(omega1*T)*(i - 1 >= 0 ? v1r[i-1] : 0);

        // v0r[i] = ring_buffer[mod] - pow(r, L) * cos(omega_mark  * L * T) * ring_buffer[mod] + r * cos(omega_mark  * T) * v0r[i -1] - r * sin(omega_mark  * T) * v0i[i -1];
        // v0i[i] = - pow(r, L) * sin(omega_mark  * L * T) * ring_buffer[mod] + r * cos(omega_mark  * T) * v0i[i - 1] + r * sin(omega_mark *  T) * v0r[i - 1];
        // v1r[i] = ring_buffer[mod] - pow(r, L) * cos(omega_space * L * T) * ring_buffer[mod] + r * cos(omega_space * T) * v1r[i -1] - r * sin(omega_space * T) * v1i[i -1];
        // v1i[i] = - pow(r, L) * sin(omega_space * L * T) * ring_buffer[mod] + r * cos(omega_space * T) * v1i[i - 1] + r * sin(omega_space * T) * v1r[i - 1];

        // define-se um sinal de decisão, que se for positivo indica que devemos gerar nível lógico alto
        decision[i] = v1r[i]*v1r[i] + v1i[i]*v1i[i] - v0r[i]*v0r[i] - v0i[i]*v0i[i];
        v0 = v0r[i]*v0r[i] + v0i[i]*v0i[i];
        v1 = v1r[i]*v1r[i] + v1i[i]*v1i[i];

        // Passa-se o sinal por um filtro passa-baixas com corte em 300 Hz.
        filtered_decision[i] = butter_b[0] * decision[i] + butter_b[1] * decision[i -1] + butter_b[2] * decision[i -2] - butter_a[1] * filtered_decision[i - 1] - butter_a[2] * filtered_decision[i -2];

        fwrite(&v0, 1, sizeof(float), f_v0);
        fwrite(&v1, 1, sizeof(float), f_v1);
        
        //Estado portadora detectada 
        if(filtered_decision[i] > 0 || abs(filtered_decision[i]) < 120)
        {
            bit_num++;
            if(bit_num > 50)
                state = State::BEARER_NOT_DETECT;  
        }
        else
        {
            bit_num = 0;
            state = State::BEARER_DETECT;
        }

        if(state == State::BEARER_DETECT)
            digital_samples[i] = 1;
        else if(state == State::BEARER_NOT_DETECT)
            digital_samples[i] = 0;
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
