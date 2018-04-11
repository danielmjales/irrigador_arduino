#include <avr/io.h>


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // nao usar
  ADCSRA = 0b10000111; // ADEN = 1 (habilita o conversor AD) e realiza a divisao do clock por 128
  ADMUX = 0b01000000; // Configura referencia como VCC e habilita leitura da porta analogica 0

//  DDRB |= 0b00000011;
//  PORTB |= 0b00000011;
  DDRD = 0xFF; // led
  // Configurando PWM usando o TIMER0
  TCCR0A = 0b10100011; //PWM não invertido nos pinos OC0A e OC0B
  TCCR0B = 0b00000011; //liga TC0, prescaler = 64
  OCR0A = 0;    //controle do ciclo ativo do PWM 0C0A
  OCR0B = 0;    //controle do ciclo ativo do PWM OC0B
      
}



uint16_t valorADC = 0;

void loop() {
  // put your main code here, to run repeatedly:

  ADCSRA |= 0b01000000; // ADSC = 1 (inicia a conversao)
  while(!(ADCSRA & 0b00010000)); // 0b00010000 enquanto o bit de interrupcao de ocnversao nao for 1, continua no loop   

  valorADC = ADC;
  // Serial.println(valorADC);
  //PORTB &= 0b11111110;
  OCR0A = ADC >> 1;
  Serial.println(OCR0A);
}
