#include <avr/io.h>

# define MPU6050_ADDRESS 0x68
# define Enable       8            //D8                 //PORTB 0                    
# define Step_3       7            //D7                 //PORTD 7                    
# define Step_2       6            //D6                 //PORTD 6                    
# define Step_1       5            //D5                 //PORTD 5                    
# define Dir_3        4            //D4                 //PORTD 4                    
# define Dir_2        3            //D3                 //PORTD 3                    
# define Dir_1        2            //D2                 //PORTD 2  
# define MS3          9            //D9                 //PORTB 1 //các chân MS3 cua 2 MOtor1 và MS3 Motor2 nối chung
# define MS2          10           //D10                //PORTB 2 //các chân MS2 cua 2 MOtor1 và MS2 Motor2 nối chung
# define MS1          11           //D11                //PORTB 3 //các chân MS1 cua 2 MOtor1 và MS1 Motor2 nối chung
# define BAUD_RATE 9600
# define UBRR_VALUE ((F_CPU / (BAUD_RATE * 16UL)) - 1)

void  pin_INI() {
  DDRB |= (1 << Enable) | (1 << Step_1) | (1 << Step_2) | (1 << Step_3) | (1 << Dir_1) | (1 << Dir_2) | (1 << Dir_3);
  DDRC |= (1 << MS1) | (1 << MS2) | (1 << MS3);

  PORTB &= ~((1 << Enable) | (1 << Step_1) | (1 << Step_2) | (1 << Step_3) | (1 << Dir_1) | (1 << Dir_2) | (1 << Dir_3));
  PORTC |= (1 << MS1) | (1 << MS2) | (1 << MS3);
}

void timer_INI() {
  TCCR2A = 0;                                                               
  TCCR2B = 0;                                                              
  TCCR2B |= (1 << CS21);                                                    
  OCR2A = 39;                                                              
  TCCR2A |= (1 << WGM21);        
  TIMSK2 |= (1 << OCIE2A); 
}

void uart_INI() {
  UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
  UBRR0L = (uint8_t)(UBRR_VALUE);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

int8_t Dir_M1, Dir_M2, Dir_M3;   
volatile int Count_timer1, Count_timer2, Count_timer3;
volatile int32_t Step1, Step2, Step3;
int16_t Count_TOP1, Count_BOT1, Count_TOP2, Count_BOT2, Count_TOP3, Count_BOT3; 
float Input, Output, I, Input_last, M, Motor;
float Filter_AngleX;

float Kp = 25.0;
float Ki = 0.01;
float Kd = 0.01;

float  Offset = -3.3;
unsigned long loop_timer;

ISR(TIMER2_COMPA_vect) {
  if (Dir_M1 != 0) {                                                         
    Count_timer1++;
    if (Count_timer1 <= Count_TOP1)PORTD |= 0b00100000;                       
    else PORTD &= 0b11011111;                                               
    if (Count_timer1 > Count_BOT1) {
      Count_timer1 = 0;                           
      if (Dir_M1 > 0)Step1++;
      else if (Dir_M1 < 0)Step1--;
    }
  }

  if (Dir_M3 != 0) {
    Count_timer3++;
    if (Count_timer3 <= Count_TOP3)PORTD |= 0b10000000;
    else PORTD &= 0b01111111;
    if (Count_timer3 > Count_BOT3) {
      Count_timer3 = 0;
      if (Dir_M3 > 0)Step3++;
      else if (Dir_M3 < 0)Step3--;
    }
  }
}

void Speed_L(int16_t x) {
  if (x < 0) {
    Dir_M1 = -1;
    PORTD &= 0b11111011;
  }
  else if (x > 0) {
    Dir_M1 = 1;
    PORTD |= 0b00000100;
  }
  else Dir_M1 = 0;

  Count_BOT1 = abs(x);
  Count_TOP1 = Count_BOT1 / 2;
}

void Speed_R(int16_t x) {
  if (x < 0) {
    Dir_M3 = -1;
    PORTD &= 0b11101111;
  }
  else if (x > 0) {
    Dir_M3 = 1;
    PORTD |= 0b00010000;
  }
  else Dir_M3 = 0;

  Count_BOT3 = abs(x);
  Count_TOP3 = Count_BOT3 / 2;
}

void I2C_Init() {
  TWBR = ((F_CPU / 100000UL) - 16) / 2;
  TWCR = (1 << TWEN);
}

void I2C_Start() {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

void I2C_Write(uint8_t data) {
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

uint8_t I2C_ReadACK() {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  while (!(TWCR & (1 << TWINT)));
  return TWDR;
}

uint8_t I2C_ReadNACK() {
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  return TWDR;
}

void MPU6050_ReadAccelerometer(float Filter_AngleX) {
  int16_t accelX, accelY, accelZ;
  float weight = 0.1, AngleX;
  I2C_Start();
  I2C_Write((MPU6050_ADDRESS << 1) | 0);
  I2C_Write(0x3B);
  I2C_Start();
  I2C_Write((MPU6050_ADDRESS << 1) | 1);
  accelX = (I2C_ReadACK() << 8) | I2C_ReadACK();
  accelY = (I2C_ReadACK() << 8) | I2C_ReadACK();
  accelZ = (I2C_ReadACK() << 8) | I2C_ReadNACK();
  AngleX = (atan(accelY / sqrt(accelX * accelX + accelZ * accelZ)) * 180 / 3.14);
  Filter_AngleX = weight * AngleX + (1.0 - weight) * Filter_AngleX;
}

int main()
{
  pin_INI();                        
  timer_INI();  
  uart_INI();
  while(1)
  {
  MPU6050_ReadAccelerometer(Filter_AngleX);
  Input = Filter_AngleX + Offset;                           
  I += Input * Ki;
  if (I < -400) I = -400;
  if (I > 400) I = 400;
  Output = Kp * Input + I + Kd * (Input - Input_last);
  Input_last = Input;                                        

  if (Output > -2.5 && Output < 2.5)Output = 0;
  if (Output >= 400) Output = 400;
  if (Output <= -400) Output = -400;

  if (Output > 0) M = 410 - (1 / (Output + 9)) * 5500;
  else if (Output < 0)  M = -410 - (1 / (Output - 9)) * 5500;
  else M = 0;

  if (M > 0)Motor = 400 - M;
  else if (M < 0)Motor = -400 - M;
  else Motor = 0;

  Speed_L(Motor);
  Speed_R(Motor);

  while (loop_timer > micros());
  loop_timer += 1000;
  }
  return 0;
}