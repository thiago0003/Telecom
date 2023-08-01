#include <math.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <deque>
#include <stdexcept>
#include "v21.hpp"
#include "uart.hpp"
#include "config.hpp"

//#define DEBUG_TO_A_FILE

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    unsigned int digital_samples[n];

#ifdef DEBUG_TO_A_FILE
    FILE *dc   = fopen("../dc.raw", "wb");
    FILE *f_dc = fopen("../f_dc.raw", "wb");
    FILE *f_v0 = fopen("../v0.raw", "wb");
    FILE *f_v1 = fopen("../v1.raw", "wb");
#endif

    constexpr int  L = SAMPLES_PER_SYMBOL;
    constexpr float T = SAMPLING_PERIOD;
    
    constexpr float r = 0.99;

    const float rL = pow(r, L);
    const float cos0LT = cos(omega_space*L*T);
    const float sin0LT = sin(omega_space*L*T);
    const float cos0T = cos(omega_space*T);
    const float sin0T = sin(omega_space*T);

    const float cos1LT = cos(omega_mark*L*T);
    const float sin1LT = sin(omega_mark*L*T);
    const float cos1T = cos(omega_mark*T);
    const float sin1T = sin(omega_mark*T);

    //const float butter_a[3] = { 1.0        , -1.94447766 , 0.94597794 };
    //const float butter_b[3] = { 0.00037507 ,  0.00075014 , 0.00037507 };

    // substitua o loop abaixo, que gera um sinal UART constantemente ocioso, pelo seu código
    for (int i = 0; i < n; i++) {
        const float s = in_analog_samples[i];
        ring_buffer[ring_buffer_it] = s;
        ring_buffer_it = (ring_buffer_it + 1) % (L + 1);
        const float sL = ring_buffer[ring_buffer_it];

        const float v0r = s - rL*cos0LT*sL + r*cos0T*v0r_prev - r*sin0T*v1i_prev;
        const float v0i = -rL*cos0LT*s + r*cos0T*v0r_prev + r*sin0T*v1r_prev;
        const float v1r = s - rL*cos1LT*s + r*cos1T*v0r_prev - r*sin1T*v1i_prev;
        const float v1i = -rL*sin1LT*s + r*cos1T*v0i_prev + r*sin1T*v1r_prev;

        v0i_prev = v0i; v0r_prev = v0r; v1i_prev = v1i; v1r_prev = v1r;

        const float v0 = v0r*v0r + v0i*v0i;
        const float v1 = v1r*v1r + v1i*v1i;

        // define-se um sinal de decisão, que se for positivo indica que devemos gerar nível lógico alto
        const float decision = v1 - v0;

        // Passa-se o sinal por um filtro passa-baixas com corte em 300 Hz.
        const float filtered_decision = 0.00037507* decision + 0.00075014*decision_prev + 0.00037507*decision_prev_prev + 1.94447766 * filtered_decision_prev - 0.94597794 * filtered_decision_prev_prev;

        filtered_decision_prev_prev = filtered_decision_prev; 
        filtered_decision_prev = filtered_decision;

        decision_prev_prev = decision_prev;
        decision_prev = decision;

#ifdef DEBUG_TO_A_FILE
        fwrite(&v0, 1, sizeof(float), f_v0);
        fwrite(&v1, 1, sizeof(float), f_v1);
#endif

        //Estado portadora detectada 
        if(std::fabs(filtered_decision) > 60.)
        {
            counter = 50;
        }
        else if(counter > 0 && std::fabs(filtered_decision) < 50.)
        {
            --counter;
        }

        if(counter > 0)
            digital_samples[i] =  filtered_decision >= 0.;
        else
            digital_samples[i] = 1;
    }

#ifdef DEBUG_TO_A_FILE
    fwrite(&decision, n, sizeof(float), dc);
    fwrite(&filtered_decision, n, sizeof(float), f_dc);

    fclose(dc);
    fclose(f_dc);
    fclose(f_v0);
    fclose(f_v1);
#endif

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
