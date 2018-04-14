#include <avr/io.h>

uint16_t sensor_temp = 0;
uint16_t sensor_umidade = 0;

volatile uint8_t seg = 1, botaoAtivo = 0, cont = 0, tmp1, tmp2, aux = 0;

uint16_t lerSensor(uint8_t sensor) {
  ADMUX &= 0b01000000; // configura para ADC0

  if (sensor != 0) {
    ADMUX |= 0b00000001; // configura para ADC1
  }

  ADCSRA |= 0b01000000;
  while (!(ADCSRA & 0b00010000));

  return ADC;
}

ISR(TIMER2_OVF_vect) {

    sensor_temp = lerSensor(0);
    if (sensor_temp == 0) sensor_temp = 1;
    OCR0A = sensor_temp >> 2;

    sensor_umidade = lerSensor(1);
    if (sensor_umidade == 0) sensor_umidade = 1;
    OCR0B = sensor_umidade >> 2;

    if (botaoAtivo) {
        cont++;
        if (cont == 61) { // a cada 61 conta 1 segundo
            seg++;
            cont = 0;
        }
        if (seg <= 10) {
            OCR1A = (int) (sensor_umidade * 0.0175 * seg)
            tmp1 = OCR1A
        } else if (seg < 12) {
            OCR1A = tmp1
        } else if (seg < 15) {
            OCR1A = (int) (((-0.7598) * sensor_umidade * seg) + (9.2932 * sensor_temp))
            tmp1 = OCR1A
        } else if (seg < 20) {
            OCR1A = tmp1
        } else if (seg < 25) {
            
        } else if (seg < 30) {
            
        } else {
            seg = 1
            botaoAtivo = 0
            OCR1A = 0
            OCR1B = 0
        }
    }

}

int main(void) {
    ADCSRA = 0b10000111; // ADEN = 1 (habilita o conversor AD) e realiza a divisao do clock por 128
    ADMUX = 0b01000000; // Configura referencia como VCC e habilita leitura da porta analogica 0

    DDRD = 0xFF; // Todas as portas D como saida

    // Configurando PWM usando o TIMER0 - PARA LEDS
    TCCR0A = 0b10100011; //PWM não invertido nos pinos OC0A e OC0B
    TCCR0B = 0b00000011; //liga TC0, prescaler = 64
    OCR0A = 0;    //controle do ciclo ativo do PWM 0C0A
    OCR0B = 0;    //controle do ciclo ativo do PWM OC0B

    // Configuracao PWN usando TIMER1 (CONVERTE 16 BITS PARA 8 BITS) - PARA ATUADORES
    TCCR1A = 0b10100010; // PWM nao invertido nos pinos 0C1A e OC1B
    TCCR1B = 0b00011001; // liga TC1, prescaler = 1
    ICR1 = 255; // valor maximo pra contagem
    OCR1A = 0;
    OCR1B = 0;

    // Configurando a interrupcao do TIMER2
    cli();
    //TCCR2B = 0b00000111; // TC2 com prescaler de 1024. Ti=16.384 ms
    TIMSK2 = 0b00000001; // interrupção do TC2 habilitada
    sei(); // ativa as interrupções globais

    while(1) {
        // ADMUX &= 0b11110000; // Captura valor do ADC0
        // ADCSRA |= 0b01000000; // ADSC = 1 (inicia a conversao)
        // while(!(ADCSRA & 0b00010000)); // 0b00010000 enquanto o bit de interrupcao de conversao nao for 1, continua no loop   

        // sensor_temp = ADC;
        // OCR0A = sensor_temp >> 2;


        // ADMUX |= 0b00000001; // Captura valor do ADC1
        // ADMUX &= 0b11110001; // Captura valor do ADC1
        // ADCSRA |= 0b01000000; // ADSC = 1 (inicia a conversao)
        // while(!(ADCSRA & 0b00010000));

        // sensor_umidade =  ADC;
        // OCR0B = sensor_umidade >> 2;
    }
}