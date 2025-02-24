#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include <math.h>

#define BUZZER_PIN 21
#define TEMPO_ASA_BRANCA 120

const int melody_asa_branca[] = {
    392, 8, 440, 8, 494, 4, 587, 4, 587, 4, 494, 4,
    523, 4, 523, 2, 392, 8, 440, 8, 494, 4, 587, 4,
    587, 4, 523, 4, 494, 2, 0, 8, 392, 8, 392, 8,
    440, 8, 494, 4, 587, 4, 0, 8, 587, 8, 523, 8,
    494, 8, 392, 4, 523, 4, 0, 8, 523, 8, 494, 8,
    440, 8, 440, 4, 494, 4, 0, 8, 494, 8, 440, 8,
    392, 8, 392, 2, 0, 8, 392, 8, 392, 8, 440, 8,
    494, 4, 587, 4, 0, 8, 587, 8, 523, 8, 494, 8,
    392, 4, 523, 4, 0, 8, 523, 8, 494, 8, 440, 8,
    440, 4, 494, 4, 0, 8, 494, 8, 440, 8, 392, 8,
    392, 4, 698, 8, 587, 8, 659, 8, 523, 8, 587, 8,
    494, 8, 523, 8, 440, 8, 494, 8, 392, 8, 440, 8,
    392, 4, 698, 8, 587, 8, 659, 8, 523, 8, 587, 8,
    494, 8, 523, 8, 440, 8, 494, 8, 392, 8, 440, 8,
    392, -2, 0, 4
};


// Calcula a duração de uma semibreve em milissegundos
const int WHOLE_NOTE_ASA_BRANCA = (60000 * 4) / TEMPO_ASA_BRANCA; // Usado para calcular a duração de cada nota 

void configure_buzzer(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM); // Define o pino 21 para função pwm
    uint slice_num = pwm_gpio_to_slice_num(pin); // Pega o slice correspondente a este pino
    pwm_config config = pwm_get_default_config(); // Peega um conjunto de valores padrões para a configuração do pwm
    pwm_config_set_clkdiv(&config, 4.0); // Define o divisor de clock
    pwm_init(slice_num, &config, true); // Inicializa o pwm naquele slice
    pwm_set_gpio_level(pin, 0); // Define o duty cycle pra 0
}

void play_note(int pin, int freq, int duracao) {
    if (freq == 0) { // Se não houver frequência so espere pela duração fornecida
        sleep_ms(duracao);
        return;
    }
    uint slice_num = pwm_gpio_to_slice_num(pin); // Pega o slice correspondete ao pino
    uint32_t clock_hz = clock_get_hz(clk_sys); // Pega o clock do sistema que é 125Mhz
    uint32_t wrap = 12500;// Configura o wrap a 12500
    uint32_t clkdiv = clock_hz / (freq * wrap); //  Calcula o divisor de clock
    pwm_set_wrap(slice_num, wrap - 1); // Define o wrap no slice correspondente
    pwm_set_clkdiv(slice_num, clkdiv); // Define o divisor de clock no slice correspondente
    pwm_set_gpio_level(pin, wrap / 2);// Define o duty cicle a 50%
    sleep_ms(duracao * 0.9); // Toca a nota por 90% da duração
    pwm_set_gpio_level(pin, 0); // Define o duty cicle a 0%
    sleep_ms(duracao * 0.1); // Pausa por 10% da durção
}

int play_melody() {
    configure_buzzer(BUZZER_PIN); // Configura o pino do buzzer antes de tocar a melodia

    // Calcula o número de notas na melodia dividindo o tamanho total do array pelo tamanho de um elemento
    // e depois dividindo por 2, pois cada nota é representada por dois valores (frequência e duração).
    int num_notas = sizeof(melody_asa_branca) / sizeof(melody_asa_branca[0]) / 2; // 94 notas

    // Percorre o array de notas, avançando de 2 em 2, pois cada nota tem dois valores: frequência e duração.
    for (int i = 0; i < num_notas * 2; i += 2) {
        int freq = melody_asa_branca[i]; // Obtém a frequência da nota
        int duracao; // Variável para armazenar a duração da nota
        int divisor = melody_asa_branca[i + 1]; // Obtém o divisor para calcular a duração da nota
        
        // Se o divisor for positivo, a duração é calculada normalmente.
        if (divisor > 0) {
            duracao = WHOLE_NOTE_ASA_BRANCA / divisor;
        } 
        // Se o divisor for negativo, significa que a nota deve ser pontuada (1.5x o tempo normal).
        else {
            duracao = (WHOLE_NOTE_ASA_BRANCA / abs(divisor)) * 1.5;
        }

        // Toca a nota no buzzer com a frequência e duração calculadas.
        play_note(BUZZER_PIN, freq, duracao);
    }
    
    return 0; // Retorna 0 indicando que a execução da melodia foi concluída com sucesso.
}
