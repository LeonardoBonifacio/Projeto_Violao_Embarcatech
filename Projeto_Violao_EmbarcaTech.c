#include <stdio.h>        // Biblioteca de entrada e saída padrão
#include <math.h>         // Funções matemáticas para autocorrelação

// Bibliotecas do RP2040
#include "pico/stdlib.h"  
#include "hardware/adc.h" 
#include "hardware/i2c.h" 
#include "hardware/pwm.h"
#include "hardware/pio.h" 
#include "pico/bootrom.h" 

// Bibliotecas personalizadas
#include "lib/ssd1306.h"
#include "ws2812.pio.h"  
#include "lib/audio.h"

// Configuração do microfone e ADC
#define MIC_ADC_PIN 28  

// Configuração dos LEDs
#define LED_COUNT 25   
#define MATRIZ_LED_PIN 7 
#define RED_LED 13 
#define GREEN_LED 11 

// Configuração dos botões
#define BUTTON_A 5  
#define BUTTON_B 6  
#define BUTTON_JOYSTICK 22  

// Configuração do tempo
#define WAIT_TIME_DISPLAY 2000  

// Variáveis de tempo e estado
static float last_valid_frequency = 0;            
static uint32_t last_update_display = 0;       


#define SAMPLE_RATE_PADRAO 8500 // Média das sample rates
#define BUFFER_SIZE 1500   // Número de amostras para análise
#define FILTER_SIZE 5  // Ajuste para otimizar a suavização

// Configurações i2c e inicialização do display oled
#define I2C_PORT i2c1 // Interface 1
#define I2C_SDA 14 // Pino para SDA
#define I2C_SCL 15 // Pino para SCL
#define ADDRESS_DISPLAY 0x3C // Endereço do display ssd1306
ssd1306_t ssd; // Inicializa a estrutura do display

// Estrutura para armazenar notas musicais
typedef struct {
    char *note;
    float frequency;
} Note;

// Tabela de notas do violão
Note tuning_table[] = {
    {"E2", 82.41}, 
    {"A2", 110.00}, 
    {"D3", 146.83},
    {"G3", 196.00}, 
    {"B3", 246.94}, 
    {"E4", 329.63}
};

// Buffer para armazenar quais LEDs estão ligados matriz 5x5 formando um x
bool wrong[LED_COUNT] = {
    1, 0, 0, 0, 1, 
    0, 1, 0, 1, 0, 
    0, 0, 1, 0, 0, 
    0, 1, 0, 1, 0, 
    1, 0, 0, 0, 1
};

// Buffer para armazenar quais LEDs estão ligados matriz 5x5 formando um right
bool right[LED_COUNT] = {
    0, 0, 0, 1, 0, 
    1, 0, 1, 0, 0, 
    0, 1, 0, 0, 0, 
    0, 0, 0, 0, 1, 
    0, 0, 0, 0, 0
};


// Envia um valor de cor (24 bits) para a matriz de LEDs
static inline void send_led_data(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

// Converte valores RGB para um formato de 32 bits (GRB)
static inline uint32_t color_to_grb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

// Define os LEDs da matriz conforme o padrão especificado (right/wrong/off)
void update_led_matrix(uint8_t r, uint8_t g, uint8_t b, const bool *pattern) {
    uint32_t color = color_to_grb(r, g, b);
    for (int i = 0; i < LED_COUNT; i++) {
        send_led_data(pattern[i] ? color : 0);
    }
}

// Função para atualizar os leds,o buzzer e o display  conforme a frequência alvo seja atingida ou não
void att_infos(float freq, float freq_alvo, float margem) {
    if (freq >= freq_alvo  && freq < freq_alvo + margem) {
        gpio_put(GREEN_LED, 1);
        gpio_put(RED_LED, 0);
        update_led_matrix(0,255,0,right);
        ssd1306_draw_string(&ssd, "Afinado", 40, 45);
    } else {
        gpio_put(GREEN_LED, 0);
        gpio_put(RED_LED, 1);
        update_led_matrix(255,0,0,wrong);
        ssd1306_draw_string(&ssd, (freq < freq_alvo) ? "Aperte a corda" : "Afrouxe a corda", 5, 45);

    }
}

// Preenche um buffer de audio com amostras do microfone 
void capture_audio(uint16_t *buffer) {
    uint32_t delay_us = 1000000 / SAMPLE_RATE_PADRAO; // Calcula apenas uma vez
    for (int i = 0; i < BUFFER_SIZE; i++) { // Itera o tamanhao do buffer
        buffer[i] = adc_read(); //  Lê o valor de 0 a 4095 e o armazena na posição [i] do buffer
        // Aguarda um tempo antes de capturar a próxima amostra para garantir a taxa de amostragem correta.
        sleep_us(delay_us);// Cerca de us
    }
}

//Método de autocorrelação para estimar a frequência de um sinal de áudio
float autocorrelation_frequency(uint16_t *buffer, int length, int sample_rate) {
    // Calcula a média das amostras para remover qualquer componente DC do sinal.
    float mean = 0;
    for (int i = 0; i < length; i++) {
        mean += buffer[i];
    }
    mean /= length;

    // Aloca memória dinamicamente para armazenar a versão normalizada do buffer.
    float *norm_buffer = malloc(length * sizeof(float));
    if (!norm_buffer) return 0; // Falha na alocação

    //Remove a média de cada amostra, centrando o sinal em torno de zero.
    for (int i = 0; i < length; i++) {
        norm_buffer[i] = buffer[i] - mean;
    }

    int best_lag = 0;// Guarda o atraso (lag) ótimo que corresponde à melhor correlação
    float best_corr = 0;//Guarda o valor da melhor correlação encontrada.

    // Percorre diferentes valores de atraso (lag) entre 10 e length/2
    // Para cada lag, calcula a soma dos produtos norm_buffer[i] * norm_buffer[i + lag], que mede a autocorrelação do sinal.
    // Atualiza best_corr e best_lag quando encontra um lag com maior correlação.
    for (int lag = 10; lag < length / 2; lag++) {
        float sum = 0;
        for (int i = 0; i < length - lag; i++) {
            sum += norm_buffer[i] * norm_buffer[i + lag];
        }
        if (sum > best_corr) {
            best_corr = sum;
            best_lag = lag;
        }
    }
    // Se nenhuma boa correlação foi encontrada ou se best_lag for muito grande, retorna 0, pois não há uma frequência válida.
    if (best_lag == 0 || best_lag >= length / 2) return 0;

    // Define os valores de lag_before e lag_after, que são os lags vizinhos para interpolação.
    int lag_before = best_lag - 1;
    int lag_after = best_lag + 1;
    // Se lag_before ou lag_after estiverem fora dos limites válidos, a frequência é calculada diretamente como (float)sample_rate / best_lag
    if (lag_before < 10 || lag_after >= length / 2) return (float)sample_rate / best_lag;

    // Calcula os valores de autocorrelação para os lags lag_before, best_lag e lag_after.
    float corr_before = 0, corr_best = 0, corr_after = 0;
    for (int i = 0; i < length - lag_after; i++) {
        corr_before += norm_buffer[i] * norm_buffer[i + lag_before];
        corr_best += norm_buffer[i] * norm_buffer[i + best_lag];
        corr_after += norm_buffer[i] * norm_buffer[i + lag_after];
    }

    // Ajuste fino da frequência usando uma interpolação quadrática nos valores corr_before, 
    // best_corr e corr_after para obter uma estimativa mais precisa do precise_lag.
    float precise_lag = best_lag;
    if (best_lag > 1 && best_lag < length / 2 - 1) {
        float y1 = best_corr;
        float y0 = corr_before;
        float y2 = corr_after;
    
        float denom = (y0 - 2 * y1 + y2);
        if (denom != 0) {
            precise_lag = best_lag + 0.5f * (y0 - y2) / denom;
        } else {
            precise_lag = best_lag;
        }

    }
    
    // Libera a memória alocada dinamicamente (norm_buffer).
    free(norm_buffer);
    // Se precise_lag estiver fora dos limites aceitáveis, retorna 0.
    if (precise_lag < 10 || precise_lag > length / 2) return 0;
    // Calcula a frequência estimada
    return (float)sample_rate / precise_lag;
}

// Encontra a nota mais próxima com base na frequência detectada já que a frequência detectada não é precisa
Note find_closest_note(float frequency) {
    // Armazena a menor diferença encontrada entre frequency e as frequências das notas na tabela.
    float min_diff = fabs(frequency - tuning_table[0].frequency);
    // Armazena a nota mais próxima encontrada
    Note closest_note = tuning_table[0];

    // Percorre as 6 notas de tuning_table que são as cordas do violão
    for (int i = 0; i < 6; i++) {
        // Calcula a diferença absoluta (fabs) entre a frequência detectada (frequency) e a frequência da nota tuning_table[i]
        float diff = fabs(frequency - tuning_table[i].frequency);
        // Se a diferença diff for menor que min_diff (ou seja, se encontramos uma nota mais próxima)
        if (diff < min_diff) {
            min_diff = diff; // Atualiza min_diff com a nova menor diferença.
            closest_note = tuning_table[i]; // Atualiza closest_note com a nota mais próxima encontrada até o momento.
        }
    }
    return closest_note; // Retorna a nota mais proxima
}

// Aplica um filtro de média móvel a um buffer de áudio, suavizando os valores para reduzir ruídos
uint16_t apply_filtro_media_mobile(uint16_t *buffer, int tamanho) {
    // Cria um array temporário saida para armazenar os valores filtrados.
    static uint16_t saida[BUFFER_SIZE]; // Mesmo tamanho do buffer original

    // Percorre cada elemento do buffer original.
    for (int i = 0; i < tamanho; i++) {
        int soma = 0; // Acumula os valores das amostras dentro da janela do filtro.
        int count = 0; // Conta quantos valores foram somados (para calcular a média corretamente)
        // Percorre os vizinhos ao redor da amostra buffer[i], formando a janela de suavização
        for (int j = -FILTER_SIZE / 2; j <= FILTER_SIZE / 2; j++) {
            // Verifica limites do array para evitar acessar índices inválidos
            if (i + j >= 0 && i + j < tamanho) {
                // Soma os valores da janela e incrementa count (quantidade de valores somados)
                soma += buffer[i + j];
                count++;
            }
        }
        // Calcula a média dos valores da janela e armazena no array saida
        saida[i] = soma / count;
    }

    // Copia os valores filtrados de saida de volta para buffer, substituindo os valores originais pelos valores suavizados
    for (int i = 0; i < tamanho; i++) buffer[i] = saida[i];
}

void display_note(float frequency) {
    // Ignorar leituras fora da faixa e manter a última frequência válida
    if (frequency >= 60 && frequency <= 370) {
        last_valid_frequency = frequency;
        last_update_display = time_us_32() / 1000;
    }

    // Verifica se o tempo de exibição da última frequência já passou
    if ((time_us_32() / 1000) - last_update_display > WAIT_TIME_DISPLAY) return;  // Mantém a última frequência na tela por um tempo maior

    Note detected_note = find_closest_note(last_valid_frequency); // Captura a nota atraves da última frequência 

    ssd1306_fill(&ssd, false); // Limpa o display

    char buffer[20]; // Cria um buffer para armazenar a primeira string que será mostrada na tela
    sprintf(buffer, "Nota %s", detected_note.note);// Preenche o buffer com as informações da nota mais proxima a frequência capturada
    ssd1306_draw_string(&ssd, buffer, 10, 10); // Desenha o buffer na tela

    sprintf(buffer, "Freq %.2fHz", last_valid_frequency);// Presenche novamente o buffer com as informações da Frequência capturada
    ssd1306_draw_string(&ssd, buffer, 10, 25); // Desenha o buffer na tela

}

// Inicializa o ADC
void init_adc() {
    adc_init();
    adc_gpio_init(MIC_ADC_PIN);
    adc_select_input(2);
}

void init_i2c_and_display_ssd1306(){
  i2c_init(I2C_PORT, 400 * 1000);// Inicialização da comunicação I2C. Usando em 400Khz.

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Configuração do pino GPIO como função I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Configuração do pino GPIO como função I2C
  gpio_pull_up(I2C_SDA); // Configura as linhas em pull up
  gpio_pull_up(I2C_SCL); //  Configura as linhas em pull up

  ssd1306_init(&ssd, WIDTH, HEIGHT, false, ADDRESS_DISPLAY, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_fill(&ssd, false);// Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_send_data(&ssd); // Envia os dados para o display

}

int main() {
    // Inicializa o adc no gpio do microfone selecionando o canal 2
    init_adc(); 
    // Inicializa a comunicaççao i2c e o display ssd1306
    init_i2c_and_display_ssd1306(); 
    // Inicializa o gpio responsável por controlar o botão do joystick
    gpio_init(BUTTON_JOYSTICK); 
    // Define a direção do botão do joystick como entrada
    gpio_set_dir(BUTTON_JOYSTICK,GPIO_IN); 
    // Coloca o botão do joystick em Pull up para evitar instabilidades
    gpio_pull_up(BUTTON_JOYSTICK); 
    // Inicializa o gpio responsável pelo led vermelho
    gpio_init(RED_LED);
    // Inicializa o gpio responsável pelo led verde
    gpio_init(GREEN_LED);
    // Define a direção do led vermelho como saída
    gpio_set_dir(RED_LED,GPIO_OUT);
    // Define a direção do led verde como saida
    gpio_set_dir(GREEN_LED,GPIO_OUT);
    // Inicializa a GPIO responsável pelo botao b
    gpio_init(BUTTON_B);
    // Define a direção do botão b como entrada
    gpio_set_dir(BUTTON_B,GPIO_IN);
    // Colocar botão b em pull up para evitar instabilidades
    gpio_pull_up(BUTTON_B);
    // Inicializa a GPIO responsável pelo botao a
    gpio_init(BUTTON_A);
    // Define a direção do botão a como entrada
    gpio_set_dir(BUTTON_A,GPIO_IN);
    // Colocar botão a em pull up para evitar instabilidades
    gpio_pull_up(BUTTON_A);
    // Inicializa o Buzzer
    configure_buzzer(BUZZER_PIN);

    PIO pio = pio0;// Seleciona o bloco pio que será usado
    int sm = 0; // Define qual state machine será usada
    uint offset = pio_add_program(pio, &ws2812_program);// Carrega o programa PIO para controlar os WS2812 na memória do PIO.
    ws2812_program_init(pio, sm, offset, MATRIZ_LED_PIN, 800000, false); //Inicializa a State Machine para executar o programa PIO carregado.

    // Cria um buffer para armazenar os valores adc lidos 
    uint16_t audio_buffer[BUFFER_SIZE];

    // Definir um tempo de atraso no loop principal
    absolute_time_t next_sample_time = make_timeout_time_ms(50);

    // Tela inicial
    ssd1306_draw_string(&ssd,"BEM VINDO",28,10);
    ssd1306_draw_string(&ssd,"AO",55,30);
    ssd1306_draw_string(&ssd,"PROGRAMA",30,50);
    ssd1306_send_data(&ssd);
    sleep_ms(1000);
    ssd1306_fill(&ssd,false);

    while (1){
        ssd1306_draw_string(&ssd,"PRESSIONE",28,5);
        ssd1306_draw_string(&ssd,"A para Notas",6,20);
        ssd1306_draw_string(&ssd,"B para musica",6,35);
        ssd1306_draw_string(&ssd,"JOYBTN afinador",6,50);
        ssd1306_send_data(&ssd);
        if (gpio_get(BUTTON_A) == 0) for (int i = 0; i < 6; i++) play_note(BUZZER_PIN,tuning_table[i].frequency,800);
        if (gpio_get(BUTTON_B) == 0) play_melody();
        if (gpio_get(BUTTON_JOYSTICK) == 0)break;
    }
    

    while (1) {
        capture_audio(audio_buffer);
        
        apply_filtro_media_mobile(audio_buffer, BUFFER_SIZE);
        float frequency = autocorrelation_frequency(audio_buffer, BUFFER_SIZE, SAMPLE_RATE_PADRAO);

        Note detected_note = find_closest_note(frequency);
    
        // Exibe a nota e frequência
        display_note(frequency);
        // Atualiza as informações dos leds, display e buzzer para a corda esteja afinada
        if (frequency > 60 && frequency < 87) {
            att_infos(frequency, 82.41, 2.50f);
        } else if (frequency > 90 && frequency < 116) {
            att_infos(frequency, 110.00, 2.50f);
        } else if (frequency > 120 && frequency < 160) {
            att_infos(frequency, 146.83, 2.50f);
        } else if (frequency > 170 && frequency < 220) {
            att_infos(frequency, 196.00, 3.50f);
        } else if (frequency > 230 && frequency < 260) {
            att_infos(frequency, 246.94, 4.00f);
        } else if (frequency > 270 && frequency < 370) {
            att_infos(frequency, 329.63, 4.00F);
        }
    
        // Envia tudo para o display de uma vez, evitando piscadas
        ssd1306_send_data(&ssd);

        // Espera pelo tempo determinado
        busy_wait_until(next_sample_time);
        // Baseado no último tempo, criar outro tempo adicionadno 50ms
        next_sample_time = delayed_by_ms(next_sample_time, 50);
    }
    
}
