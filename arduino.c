#include <avr/io.h>
#include <avr/interrupt.h>

#define conv_ascii 48
#define tam_vetor 5

uint16_t sensor_temp = 0;
uint16_t sensor_umidade = 0;

volatile uint8_t seg = 1, botaoAtivo = 0, cont = 0, tmp1=0, tmp2=0, aux = 0;

uint16_t lerSensor(uint8_t sensor) {
  // Configura ADMUX para ADC0
  ADMUX &= 0b01000000;

  if (sensor != 0) {
    // Configura ADMUX para ADC1
    ADMUX |= 0b00000001;
  }

  // Habilita o início da conversão
  ADCSRA |= 0b01000000;
  // Pool para verificar se a conversão já finalizou
  while (!(ADCSRA & 0b00010000));

  return ADC;
}

// Interrupção a partir do Contador 2
// Nossa interrupção ocorre quando ocorre overflow no contador 2 (TCCR2B)
// Que no caso vai ocorrer a cada 16.384ms
ISR(TIMER2_OVF_vect) {
  // Faz a leitura do sensor de temperatura
  sensor_temp = lerSensor(0);
  // Caso o valor do sensor for zero,atribuir ao mesmo o valor 1
  // para não ocorrer multiplicações por zero na curva
  if (sensor_temp == 0) sensor_temp = 1;
  // Atribui o valor do sensor ao registrador OCR0A
  // fazendo a divisão desse valor por 2, irá converter 10 bits em 8 bits
  OCR0A = sensor_temp >> 2;

  // O mesmo ocorre com o sentor de umidade
  sensor_umidade = lerSensor(1);
  if (sensor_umidade == 0) sensor_umidade = 1;
  OCR0B = sensor_umidade >> 2;

  
  // Verifica de o PD4 (botão) está com 5V e verifica se o botão
  // não está ativo, caso sim ativa o estado do botão
  if ( (PIND & 0b00010000) && botaoAtivo == 0 ) botaoAtivo = 1;

  // Se o botão estivar ativo começa a fazer a curva do PWM
  if (botaoAtivo) {
    // A partir de cálculos definimos que a cada cont = 61 significa que passou 1 segundo
    cont++;
    if (cont == 61) {
        seg++;
        cont = 0;
    }
    // Curva do PWM
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
        OCR1A = (int) tmp2 + ((0.0149 * sensor_umidade * seg) + ((-0.1720) * sensor_temp));
        tmp1 = OCR1A;
    } else if (seg <= 30) {
        OCR1A = (int) tmp1 + (((-0.0399) * sensor_umidade * seg) + ((1.196) * sensor_temp));
    } else {
        // Quando terminar a curva atribui o estado do botão para desligado, 
        // limpa dos registrados e desliga todos os leds
        seg = 1;
        botaoAtivo = 0;
        OCR1A = 0;
        OCR1B = 0;
        PORTD = 0x00;
    }
  }
}

int main(void) {
  // ADEN = 1 (habilita o conversor AD) e realiza a divisao do clock por 128
  ADCSRA = 0b10000111;
  // Configura referencia como Vin do Arduino e habilita leitura da porta analogica 0
  ADMUX = 0b01000000;

  // Configura portas dos leds como saida (PD3, PD5, PD6 -> São portas com saida PWM) e botão (PD4)
  DDRD = 0b01100100;
  // Inicia o valor do LED (PD3 - botão) como desligado e os outros leds ligados
  PORTD &= 0b11111011;

  // Configura as portas PC0 e PC1 como entradas para receber os valores dos dois sensores
  DDRC = 0b11111100;
  // Inicia os valores como zero
  PORTC &= 0b0000000;

  
  // Configurando PWM usando o TIMER0 - PARA LEDS
  // --------------------------------------------
  // PWM não invertido nos pinos OC0A e OC0B
  TCCR0A = 0b10100011;
  // Liga TC0, prescaler = 64
  TCCR0B = 0b00000011;

  // Configuracao PWN usando TIMER1 (CONVERTE 16 BITS PARA 8 BITS) - PARA ATUADOR
  // ----------------------------------------------------------------------------
  // PWM nao invertido nos pinos 0C1A e OC1B
  TCCR1A = 0b10100001;
  // Liga TC1, prescaler = 1
  TCCR1B = 0b00001001;
  
  // Configurando a interrupcao do TIMER2
  // ------------------------------------
  // Desliga as interrupções globais
  cli();
  // TC2 com prescaler de 1024. Ti=16.384 ms
  TCCR2B = 0b00000111;
  // Interrupção do TC2 habilitada
  TIMSK2 = 0b00000001;
  // Ativa as interrupções globais
  sei();


  // Inicia a comunicação serial
  initUSART();

  // Vetor que possui os valores da serial convertidos
  uint8_t digitos[tam_vetor];
  
  while(1) {
    // Se botão estivar ativo
    if (botaoAtivo) {
      // Liga o led de operação, identificando que o sistema está funcionando
      PORTD |= 0b00000100;
    } else {
      // Caso o botão não esteja ativo, desliga o led relacionado ao botão
      PORTD &= 0b11111011;
    }
        
    // Faz a comunicação usart retornando os valores, respectivamente,
    // do sensor de umidade, temperatura e do pwm de saída    
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
    txByte(digitos[2]);
    txByte(digitos[1]);
    txByte(digitos[0]);
    txByte(digitos[4]);
    txByte('\n');
  }
}

void initUSART() {
  // Configurado para 9600bps
  UBRR0L = 103;
  UBRR0H = 0;

  UCSR0B |= _BV(RXEN0) | _BV(TXEN0);
  UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00);
}

void txByte(uint8_t info) {
  // Transmissгo de dados
  // Bit UDRE indica quando o buffer de tx pode enviar dados
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = info;

}

// Realiza a leitura do buffer da serial e disponibiliza os dados
// no registrador UDR0
uint8_t rxByte() {
  // Bit RXC sinaliza quando existem bytes nгo lidos no buffer
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;
}

uint8_t transceiver(uint8_t *data) {
  escreve_USART(data);
  return rxByte();
}

// Conversão de um número em seus digitos individuais.
// O papel dessa função é poder exibir corretamente os valores na porta serial, já
// que algun serão maiores que 255 e alguns bugs estranhos acontecem mesmo com valores
// menores;
// ------------------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp) {
  unsigned char n;
  for (n = 0; n < tam_vetor; n++)
    disp[n] = 0 + conv_ascii;
  do
  {
    *disp = (valor % 10) + conv_ascii;
    valor /= 10;
    disp++;
    // limpa vetor para armazenagem dos digitos
    // pega o resto da divisão por 10
    // pega o inteiro da divisão por 10
  } while (valor != 0);
}
// -----------------------------------------------------------------------------------

// Usamos essa função apenas para percorrer um vetor de char (string)
// e escrevê-la na saída serial.
void escreve_USART(char *c) {
  for (; *c != 0; c++) txByte(*c);
}

void escreve_USART(unsigned char c[], int t) {
  for (int i = t; i >= 0; i--) {
    txByte(c[i]);
  }
}