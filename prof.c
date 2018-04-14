/*
 * ProjetoAula.c
 *
 * Created: 3/7/2018 4:07:38 PM
 * Author : mfern
 */ 

#include <avr/io.h>

volatile uint16_t sf2 = 0; //Sensor fronta de distância

ISR(ADC_vect)
{
	sf2 = ADC;
}

int main(void)
{
  DDRB |= 0b00001111; //Seta pb0, pb1 (motor 1), pb2 e pb3 (motor 2) como saídas para controle de direção dos motores
  //PB1 e PB0 - Roda da direita (10 - Frente e 01 para trás)
  //PB3 e PB2 - Roda da esquerda (10 - Frente e 01 para trás)
  PORTB &= 0b11110000; //Inicializa as portas com zero
  
  //Configurar o ADC
  ADMUX |= 0b01000000;
  ADCSRA |= 0b10000111; // CLK/128
  
  //Configurar PWM OC0A (PD6 - Roda Direita) e OC0B (PD5)
   DDRD |=  0b01100000; //PD5 e PD6 como saída
   
   //Configura o circuito para modo FAST PWM 
   TCCR0A = 0b10100011; //Configura os pinos PD5 e PD6 para receber os sinal PWM pelo contator
   TCCR0B = 0b00000101; //Configura a divisão do clock para o contador nesta caso o divisão é por 1
  
  //Variáveis para serem utilizadas com o conversaro ADC 
  //uint16_t  = unsigned short int
  uint16_t sf = 0; //Sensor fronta de distância
  uint16_t sd = 0; //Sensor da direita de distância
  uint16_t se = 0; //Sensor da esquerda de dsitância
  
  PORTB |= 0b00001010;
  OCR0A = 200;
  OCR0B = 200;
  
  //Habilitar a interrupção do ADC
  ADCSRA |= 0b00001000;
  
  sei(); //Habilita interrupção global
  //cli()
  while(1)
  {
    //OCR0A = 127;
    //OCR0B = 127;
    
    ADMUX &= 0b11110000; //Captura o sinal no ADC0
     ADCSRA |= 0b01000000; //Inicia a conversão ADSC vai para 1 e ADIF para zero
     //Pool para esperar o final da conversão (ADIF vai para 1)
     
	 //while(!(ADCSRA & 0b00010000));
     //sf= ADC;
     
     //
     //ADMUX |= 0b00000001;
     //ADMUX &= 0b11110001;
    //ADCSRA |= 0b01000000; //Inicia a conversão ADSC vai para 1 e ADIF para zero
    ////Pool para esperar o final da conversão (ADIF vai para 1)
    //while(!(ADCSRA & 0b00010000));
    //se= ADC;
    //
    //ADMUX |= 0b00000010;
    //ADMUX &= 0b11110010;
    //ADCSRA |= 0b01000000; //Inicia a conversão ADSC vai para 1 e ADIF para zero
    ////Pool para esperar o final da conversão (ADIF vai para 1)
    //while(!(ADCSRA & 0b00010000));
    //sd= ADC;    
    //
    if (sf2 > 512)
    {
      //Girar no mesmo eixo a uma velocidade lenta
      OCR0A = 190;
      OCR0B = 190;
      PORTB = (PINB & 0b11110000) | (0b00001001); //Gira no mesmo eixo
    }
    else
    {
      //Segue direto e mais rápido
      OCR0A = 200;
      OCR0B = 200;
      PORTB = (PINB & 0b11110000) | (0b00001010); //Segue direto
    }
    //if (sd < 1000)
    //{
      //PORTB |= 0b00000100;
      //PORTB &= 0b11111100;
    //}
    
    
  }
  
    
}

