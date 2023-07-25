#include "uart.hpp"
#include "ring_buffer.hpp"

void UART_RX::put_samples(const unsigned int* buffer, unsigned int n)
{
    // Usar uma máquina de estados para receber os dados UART, percorrendo cada amostra
    // buffer e lidando com cada estado
    static enum class State {
        IDLE,                   // Estado 1: receptor está esperando o próximo start bit
        SEARCHING_START_BIT,    // Estado 2: receptor está procurando pelo start bit
        RECEIVING_BYTE          // Estado 3: receptor está recebendo os bits de dados do byte
    } state = State::IDLE;

    for (unsigned int i = 0; i < n; i++)
    {
        int mod = i % (SAMPLES_PER_SYMBOL + 1);
        unsigned int sample = buffer[i];
        ring_buffer[mod] = sample;

        switch (state)
        {
        // Receptor verifica se a amostra atual é um nível lógico baixo (0), o que indica início do start bit, e,
        // se for, muda para o estado SEARCHING_START_BIT e começa a contagem do número de bits consecutivos 
        // de nível lógico baixo
        case State::IDLE:
            if (sample == 0) {
                state = State::SEARCHING_START_BIT;
                bit_counter = 0;
                bit_counter_noisy_tolerance = 0;
            }
            break;

        // Receptor continua contando os bits de nível lógico baixo e, se o contador atingir pelo menos 25 
        // e a próxima amostra depois das últimas 30 amostras também for 0, o receptor considera que encontrou
        // o start bit e muda para o estado RECEIVING_BYTE. E se isso não acontecer, muda para o estado IDLE de novo
        case State::SEARCHING_START_BIT:
            bit_counter++;

            if (sample == 0) {
                if (bit_counter >= 30) {
                    state = State::RECEIVING_BYTE;
                    received_byte = 0;
                    bit_counter = 0;
                    bit_counter_noisy_tolerance = 0;

                    

                }
            } else {
                bit_counter_noisy_tolerance++;

                // Se amostra ultrapassar do limite de tolerância não será reconhecido start bit
                if (bit_counter_noisy_tolerance > 5) {
                    state = State::IDLE;
                }
            }
            break;

        // Receptor continua recebendo os bits de dados do byte e, a cada múltiplo de SAMPLES_PER_SYMBOL amostras,
        // ele adiciona o bit atual ao byte recebido (deslocamento de bits). Depois de 9*SAMPLES_PER_SYMBOL amostras,
        // ou seja quando todos os 8 bits de dados são recebidos, o receptor volta para o estado IDLE e chama a 
        // função get_byte passando o byte recebido
        case State::RECEIVING_BYTE:
            bit_counter++;
            if ((bit_counter % SAMPLES_PER_SYMBOL - (SAMPLES_PER_SYMBOL / 2)) == 0) { // Garantir que receptor meça no meio do bit
                received_byte |= (sample << (bit_counter / SAMPLES_PER_SYMBOL - 1));

                if (bit_counter >= (9 * SAMPLES_PER_SYMBOL)) {
                    state = State::IDLE;
                    get_byte(received_byte);
                }
            }
            break;
        }
    }
}

void UART_TX::put_byte(uint8_t byte)
{
    samples_mutex.lock();
    put_bit(0);  // start bit
    for (int i = 0; i < 8; i++) {
        put_bit(byte & 1);
        byte >>= 1;
    }
    put_bit(1);  // stop bit
    samples_mutex.unlock();
}

void UART_TX::get_samples(unsigned int *buffer, unsigned int n)
{
    samples_mutex.lock();
    std::vector<unsigned int>::size_type i = 0;
    while (!samples.empty() && i < n) {
        buffer[i++] = samples.front();
        samples.pop_front();
    }
    samples_mutex.unlock();

    while (i < n) {
        // idle
        buffer[i++] = 1;
    }
}

void UART_TX::put_bit(unsigned int bit)
{
    for (int i = 0; i < SAMPLES_PER_SYMBOL; i++) {
        samples.push_back(bit);
    }
}
