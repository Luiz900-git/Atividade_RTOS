#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"


// Biblioteca gerada pelo arquivo .pio durante compilação.
#include "ws2818b.pio.h"

// Definição do número de LEDs e pino.
#define LED_COUNT 25
#define LED_PIN 7

    
// Configuração do pino do buzzer
#define BUZZER_PIN 21

// Configuração da frequência do buzzer (em Hz)
#define BUZZER_FREQUENCY 3000

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

#define led1 11 // verde
#define led2 12 // azul
#define led3 13 // vermelho

#define BUTTON_A 5    // GPIO conectado ao Botão A

volatile int x = 2;

// Flag global para modo noturno
volatile bool nightMode = false;

// Definição de pixel GRB
struct pixel_t {
    uint8_t G, R, B; // Três valores de 8-bits compõem um pixel.
   };
   typedef struct pixel_t pixel_t;
   typedef pixel_t npLED_t; // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.
   
   // Declaração do buffer de pixels que formam a matriz.
   npLED_t leds[LED_COUNT];
   
   // Variáveis para uso da máquina PIO.
   PIO np_pio;
   uint sm;
   
   /**
   * Inicializa a máquina PIO para controle da matriz de LEDs.
   */
   void npInit(uint pin) {
   
    // Cria programa PIO.
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;
   
    // Toma posse de uma máquina PIO.
    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
      np_pio = pio1;
      sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
    }
   
    // Inicia programa na máquina PIO obtida.
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);
   
    // Limpa buffer de pixels.
    for (uint i = 0; i < LED_COUNT; ++i) {
      leds[i].R = 0;
      leds[i].G = 0;
      leds[i].B = 0;
    }
   }
   
   /**
   * Atribui uma cor RGB a um LED.
   */
   void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
   }
   
   /**
   * Limpa o buffer de pixels.
   */
   void npClear() {
    for (uint i = 0; i < LED_COUNT; ++i)
      npSetLED(i, 0, 0, 0);
   }
   
   /**
   * Escreve os dados do buffer nos LEDs.
   */
   void npWrite() {
    // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
    for (uint i = 0; i < LED_COUNT; ++i) {
      pio_sm_put_blocking(np_pio, sm, leds[i].G);
      pio_sm_put_blocking(np_pio, sm, leds[i].R);
      pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
   }

void vBlinkLed4Task(void*pvParameters){

    


     // Inicializa matriz de LEDs NeoPixel.
     npInit(LED_PIN);

    while(true){

         // Inicializa matriz de LEDs NeoPixel.
        //npInit(LED_PIN);
        //npClear();
        //npWrite();

        // Aqui, você desenha nos LEDs.
        if(nightMode){
        npSetLED(2, 0, 255, 0); // Define o LED de índice 0 para vermelho.
        npSetLED(7, 0, 255, 0); // Define o LED de índice 12 (centro da matriz) para verde.
        npSetLED(12, 0, 255, 0);
        npSetLED(17, 0, 255, 0);
        npWrite(); // Escreve os dados nos LEDs
        vTaskDelay(pdMS_TO_TICKS(2000));

        npSetLED(2, 255, 255, 0); // Define o LED de índice 0 para vermelho.
        npSetLED(7, 255, 255, 0); // Define o LED de índice 12 (centro da matriz) para verde.
        npSetLED(12, 255, 255, 0);
        npSetLED(17, 255, 255, 0);
        npWrite(); // Escreve os dados nos LEDs
        vTaskDelay(pdMS_TO_TICKS(2000));

        npSetLED(2, 255, 0, 0); // Define o LED de índice 0 para vermelho.
        npSetLED(7, 255, 0, 0); // Define o LED de índice 12 (centro da matriz) para verde.
        npSetLED(12, 255, 0, 0);
        npSetLED(17, 255, 0, 0);
        npWrite(); // Escreve os dados nos LEDs
        vTaskDelay(pdMS_TO_TICKS(2000));

        //npClear();
        //npWrite(); // Escreve os dados nos LEDs.
        } else{
            npSetLED(2, 255, 255, 0); // Define o LED de índice 0 para vermelho.
        npSetLED(7, 255, 255, 0); // Define o LED de índice 12 (centro da matriz) para verde.
        npSetLED(12, 255, 255, 0);
        npSetLED(17, 255, 255, 0);
        npWrite(); // Escreve os dados nos LEDs
        vTaskDelay(pdMS_TO_TICKS(2000));

        npClear();
        npWrite();
        vTaskDelay(pdMS_TO_TICKS(1000));

        }
    }


}

void vButton1Task(void * pvParameters){
    


// Configuração do GPIO do Botão A como entrada com pull-up interno
gpio_init(BUTTON_A);
gpio_set_dir(BUTTON_A, GPIO_IN);
gpio_pull_up(BUTTON_A);

while (true) {
    // Verifica o estado do botão
    if (gpio_get(BUTTON_A) == 0) {  // Botão pressionado (nível lógico baixo)
        nightMode = !nightMode;  // Alterna o modo
        vTaskDelay(pdMS_TO_TICKS(500));
        //sleep_ms(1000);
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Pequeno atraso para evitar busy loop


    
}

}

void vBlinkLed1Task(void*pvParameters)
{

    

    gpio_init(led1);
    gpio_set_dir(led1, GPIO_OUT);
    gpio_init(led3);
    gpio_set_dir(led3, GPIO_OUT);
    while (true)
    {
        if(nightMode){
        gpio_put(led1, true);
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_put(led3, true);
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_put(led1, false);
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_put(led3, false);
        //vTaskDelay(pdMS_TO_TICKS(1));
        }else {

            gpio_put(led1, true);
        //vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_put(led3, true);
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_put(led1, false);
        //vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_put(led3, false);
        vTaskDelay(pdMS_TO_TICKS(1000));

        }
    }

}

void vBlinkLed2Task(void*pvParameters)
{


   // Definição de uma função para inicializar o PWM no pino do buzzer
void pwm_init_buzzer(uint pin) {
    // Configurar o pino como saída de PWM
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Obter o slice do PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configurar o PWM com frequência desejada
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096)); // Divisor de clock
    pwm_init(slice_num, &config, true);

    // Iniciar o PWM no nível baixo
    pwm_set_gpio_level(pin, 0);
}

// Definição de uma função para emitir um beep com duração especificada
void beep(uint pin, uint duration_ms) {
    // Obter o slice do PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configurar o duty cycle para 50% (ativo)
    pwm_set_gpio_level(pin, 2048);

    // Temporização
    sleep_ms(duration_ms);

    // Desativar o sinal PWM (duty cycle 0)
    pwm_set_gpio_level(pin, 0);

    // Pausa entre os beeps
    sleep_ms(50); // Pausa de 100ms
}
     // Inicializar o PWM no pino do buzzer
     pwm_init_buzzer(BUZZER_PIN);
    while (true)
    {
        if(nightMode){
        beep(BUZZER_PIN, 1000); // Bipe de 500ms
        vTaskDelay(pdMS_TO_TICKS(1000));
        beep(BUZZER_PIN, 500); // Bipe de 500ms
        vTaskDelay(pdMS_TO_TICKS(1500));
        beep(BUZZER_PIN, 500); // Bipe de 500ms
        vTaskDelay(pdMS_TO_TICKS(1500));
        }else{

        beep(BUZZER_PIN, 1000); // Bipe de 500ms
        vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

}

void vDisplay3Task(void*pvParameters)
{


    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL);                                        // Pull up the clock line
    ssd1306_t ssd;                                                // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display
    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    char str_y[5]; // Buffer para armazenar a string
    int contador = 0;
    bool cor = true;
    while (true)
    {
        if(nightMode){
        sprintf(str_y, "%d", contador); // Converte em string
        //contador++;                     // Incrementa o contador
        ssd1306_fill(&ssd, !cor);                          // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);           // Desenha uma linha
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);           // Desenha uma linha
        ssd1306_draw_string(&ssd, "MODO NORMAL", 8, 6); // Desenha uma string
        ssd1306_draw_string(&ssd, "AVANCAR", 20, 16);  // Desenha uma string
        //ssd1306_draw_string(&ssd, "  FreeRTOS", 10, 28); // Desenha uma string
        //ssd1306_draw_string(&ssd, "Contador  LEDs", 10, 41);    // Desenha uma string
        //ssd1306_draw_string(&ssd, str_y, 40, 52);          // Desenha uma string
        ssd1306_send_data(&ssd);                           // Atualiza o display
        sleep_ms(2000);

        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);           // Desenha uma linha
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);           // Desenha uma linha
        ssd1306_draw_string(&ssd, "MODO NORMAL", 8, 6); // Desenha uma string
        ssd1306_draw_string(&ssd, "ATENCAO", 20, 16);  // Desenha uma string
        ssd1306_send_data(&ssd);                           // Atualiza o display
        sleep_ms(2000);
       

        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);           // Desenha uma linha
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);           // Desenha uma linha
        ssd1306_draw_string(&ssd, "MODO NORMAL", 8, 6); // Desenha uma string
        ssd1306_draw_string(&ssd, "ESPERAR", 20, 16);  // Desenha uma string
        ssd1306_send_data(&ssd);                           // Atualiza o display
        sleep_ms(2000);
        }else{

        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);           // Desenha uma linha
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);           // Desenha uma linha
        ssd1306_draw_string(&ssd, "MODO NOTURNO", 8, 6); // Desenha uma string
        ssd1306_draw_string(&ssd, "ATENCAO", 20, 16);  // Desenha uma string
        ssd1306_send_data(&ssd);                           // Atualiza o display
        sleep_ms(3000);
        }

    }


}

// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

int main()
{
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    // Fim do trecho para modo BOOTSEL com botão B

    stdio_init_all();

    xTaskCreate(vBlinkLed1Task, "Blink Task Led1", configMINIMAL_STACK_SIZE,
         NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vBlinkLed2Task, "Blink Task Led2", configMINIMAL_STACK_SIZE, 
       NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vDisplay3Task, "Cont Task Disp3", configMINIMAL_STACK_SIZE, 
        NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vBlinkLed4Task, "Blink Task Led4", configMINIMAL_STACK_SIZE,
        NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vButton1Task, "Button Task", configMINIMAL_STACK_SIZE,
        NULL, tskIDLE_PRIORITY, NULL);
    vTaskStartScheduler();
    panic_unsupported();
}






