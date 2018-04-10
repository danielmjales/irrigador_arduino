#include <avr/io.h>

volatile unit8_t sensor_temp = 0;

ISR(ADC_vect)
{
    sensor_temp = ADC;
}



int main(void){

    DDRB |= 0b00000011;
    PORTB &= 11111100;


    ADMUX |= 0b01000000;
    ADCSRA |= 10001111;

    sei();
}