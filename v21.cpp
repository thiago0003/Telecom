#include <math.h>
#include <numbers>
#include "v21.hpp"
#include <vector>
#include <stdio.h>

 

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    //Implementar 2 filtros passa-bandas
        //Um filtro para o Tom de marca 
        //Um filtro para o Tom de espaço
        //Filtro 1 - Filtro 2 -> Filtragem passa-baixa
        //Se a diferença filtrada > na frequência do espaço = 0
        //Se a diferença filtrada > na marca = 1
    
    //Implementar uma estratégia para detectar a ausência de uma portadora. Quando não houver portadora insira 1 no buffer de saída.

    static enum class State {
        NO_STATE,
        BEARER_NOT_DETECT,
        BEARER_DETECT
    } state = State::BEARER_NOT_DETECT;

    unsigned int digital_samples[n];

    int R = 300;
    int fs = 48000;
    float T = (float) 1 / fs;
    int L = fs / R;
    float r = 0.99;

    omega_mark = 2 * std::numbers::pi * 1850;
    omega_space = 2 * std::numbers::pi * 1650; 

    int n_amostras = 0;

    float a[] = {1.0, -1.94447766, 0.94597794};
    float b[] = {0.00037507, 0.00075014, 0.00037507};
    
    float v0i[n], v0r[n], v1i[n], v1r[n], filtered_decision[n], decision[n];

    CircularBuffer buffer_;

    FILE *f_v0 = fopen("v0.raw", "wb");
    FILE *f_v1 = fopen("v1.raw", "wb");

    for(int i = 0; i < n; i++)
    {
        buffer_.push_back(in_analog_samples[i]);
        int ii = i % SAMPLES_PER_SYMBOL;
        //v0r[n] = s[n] - r**L * cos(omega0*L*T)*s[n-L] + r*cos(omega0*T)*v0r[n-1] - r*sin(omega0*T)*v0i[n-1]
        //v0i[n] = -r**L*sin(omega0*L*T)*s[n-L] + r*cos(omega0*T)*v0i[n-1] + r*sin(omega0*T)*v0r[n-1]
        //v1r[n] = s[n] - r**L * cos(omega1*L*T)*s[n-L] + r*cos(omega1*T)*v1r[n-1] - r*sin(omega1*T)*v1i[n-1]
        //v1i[n] = -r**L*sin(omega1*L*T)*s[n-L] + r*cos(omega1*T)*v1i[n-1] + r*sin(omega1*T)*v1r[n-1]

        v0r[i] = buffer_[ii] - pow(r, L) * cos(omega_mark * L * T) * (ii - L >= 0 ? buffer_[ii -L] : 0) + r * cos(omega_mark * T) * v0r[i -1] - r * sin(omega_mark * T) * v0i[i -1];
        v0i[i] = - pow(r, L) * sin(omega_mark * L * T) * (ii - L >= 0 ? buffer_[ii - L] : 0) + r * cos(omega_mark * T) * v0i[i - 1] + r * sin(omega_mark * T) * v0r[i -1];
        v1r[i] = buffer_[ii] - pow(r, L) * cos(omega_space * L * T) * (ii - L >= 0 ? buffer_[ii - L] : 0) + r * cos(omega_space * T) * v1r[i -1] - r * sin(omega_space * T) * v1i[i -1];
        v1i[i] = - pow(r, L) * sin(omega_space * L * T) * (ii - L >= 0 ? buffer_[ii -L] : 0) + r * cos(omega_space * T) * v1i[i - 1] + r * sin(omega_space * T) * v1r[i - 1];

        decision[i] = v1r[i] * v1r[i] + v1i[i] * v1i[i] - v0r[i] * v0r[i] - v0i[i] * v0i[i];


        // ******** TESTES ******** //
        float v0 = v0r[i] * v0r[i] - v0i[i] * v0i[i];
        float v1 = v1r[i] * v1r[i] + v1i[i] * v1i[i];

        fwrite(&v0, 1, sizeof(float), f_v0);
        fwrite(&v1, 1, sizeof(float), f_v1);

        // ************************* //


        //Passar a decisão pelo passa baixa 
        filtered_decision[i] = b[0] * decision[i] + b[1] * decision[i -1] + b[2] * decision[i -2] - a[1] * filtered_decision[i - 1] - a[2] * filtered_decision[i -2];
            
        //Estado portadora detectada 
        if(filtered_decision[i] > 0 || abs(filtered_decision[i]) < 120)
        {
            n_amostras++;

            //Entra no estado de não detecção
            if(n_amostras > 50) // 50 amostras consecutivas
                state = State::BEARER_NOT_DETECT;  
            
        }
        else
        {
            n_amostras = 0;
            state = State::BEARER_DETECT;
        }

        if(state == State::BEARER_DETECT)
            digital_samples[i] = 1;
        else if(state == State::BEARER_NOT_DETECT)
            digital_samples[i] = 0;
    }

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
        phase = remainder(phase, 2*std::numbers::pi);
    }
}
