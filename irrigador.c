#include <avr/io.h>
#include <avr/interrupt.h>

#define conv_ascii 48
#define tam_vetor 5

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

    
    
    if ( (PIND & 0b00010000) && botaoAtivo == 0 ) {
        botaoAtivo = 1;
        //PORTD |= 0b00000100;
    }

    if (botaoAtivo) {
        cont++;
        if (cont == 61) { // a cada 61 conta 1 segundo
            seg++;
            cont = 0;
        }
        if (seg <= 10) {
            OCR1A = (int) (sensor_umidade * 0.0175 * seg);
            tmp1 = OCR1A;
        } else if (seg <= 12) {
            OCR1A = tmp1;
        } else if (seg <= 15) {
            OCR1A = (int) tmp1 + (((-0.0166) * sensor_umidade * seg) + (0.3744 * sensor_temp));
            tmp2 = OCR1A;
        } else if (seg <= 20) {
            OCR1A = tmp2;
        } else if (seg <= 25) {
            OCR1A = (int) tmp2 + (((0.0149) * sensor_umidade * seg) + ((-0.1720) * sensor_temp));
            tmp1 = OCR1A;
        } else if (seg <= 30) {
            OCR1A = (int) tmp1 + (((-0.0399) * sensor_umidade * seg) + ((1.196) * sensor_temp));
        } else {
            seg = 1;
            botaoAtivo = 0;
            OCR1A = 0;
            OCR1B = 0;
            PORTD = 0x00;
        }
    }

}

int main(void) {
    ADCSRA = 0b10000111; // ADEN = 1 (habilita o conversor AD) e realiza a divisao do clock por 128
    ADMUX = 0b01000000; // Configura referencia como VCC e habilita leitura da porta analogica 0

    DDRD = 0b01100100; // Todas as portas D como saida
    PORTD &= 0b11111011;
    
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
    TCCR2B = 0b00000111; // TC2 com prescaler de 1024. Ti=16.384 ms
    TIMSK2 = 0b00000001; // interrupção do TC2 habilitada
    sei(); // ativa as interrupções globais

    initUSART();
    escreve_USART("Valores Sensor1 \n");
    uint8_t digitos[tam_vetor];
    
    while(1) {

        if (botaoAtivo) {
            PORTD |= 0b00000100;
        } else {
          PORTD &= 0b11111011;
        }
        
        ident_num((unsigned int) sensor_umidade, digitos);
        txByte(digitos[3]);
        txByte(digitos[2]);
        txByte(digitos[1]);
        txByte(digitos[0]);
        digitos[4] = ';';
        txByte(digitos[4]);
        txByte('\n');

        ident_num((unsigned int) sensor_temp, digitos);
        txByte(digitos[3]);
        txByte(digitos[2]);
        txByte(digitos[1]);
        txByte(digitos[0]);
        digitos[4] = ';';
        txByte(digitos[4]);
        txByte('\n');

        ident_num((unsigned int) OCR1A, digitos);
        //txByte(digitos[3]);
        txByte(digitos[2]);
        txByte(digitos[1]);
        txByte(digitos[0]);
        //digitos[4] = ';';
        txByte(digitos[4]);
        txByte('\n');
    }
}

void initUSART()
{
  // Configurado para 9600bps
  UBRR0L = 103;
  UBRR0H = 0;

  // U2X=1 - Dobra a velocidade
  //UCSRA = (1<<U2X);

  // UCSZ2 = 0 - 8 bits
  UCSR0B |= _BV(RXEN0) | _BV(TXEN0);


  // UCSZ1 = 1 e UCSZ0 = 1 -> 8 bits
  // USBS0 = 0 -> 1 - stop bits
  // UPM0 = 0 e UMP0 = 0 -> sem bit de paridade
  UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00);
}

void txByte(uint8_t info)
{
  // Transmissгo de dados
  //Bit UDRE indica quando o buffer de tx pode enviar dados
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = info;

}

// Realiza a leitura do buffer da serial e disponibiliza os dados
// no registrador UDR0
uint8_t rxByte()
{
  //Bit RXC sinaliza quando existem bytes nгo lidos no buffer
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;
}

uint8_t transceiver(uint8_t *data) {
  //txByte(data);
  escreve_USART(data);
  return rxByte();
}

//Conversão de um número em seus digitos individuais.
//O papel dessa função é poder exibir corretamente os valores na porta serial, já
// que algun serão maiores que 255 e alguns bugs estranhos acontecem mesmo com valores
// menores;
//------------------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp)
{
  unsigned char n;
  for (n = 0; n < tam_vetor; n++)
    disp[n] = 0 + conv_ascii;
  do
  {
    *disp = (valor % 10) + conv_ascii;
    valor /= 10;
    disp++;
    //limpa vetor para armazenagem dos digitos
    //pega o resto da divisão por 10
    //pega o inteiro da divisão por 10
  } while (valor != 0);
}
//-----------------------------------------------------------------------------------

//Usamos essa função apenas para percorrer um vetor de char (string)
//e escrevê-la na saída serial.
void escreve_USART(char *c)
//escreve String
{
  for (; *c != 0; c++) txByte(*c);
}

void escreve_USART(unsigned char c[], int t) {
  for (int i = t; i >= 0; i--) {
    txByte(c[i]);
  }
}