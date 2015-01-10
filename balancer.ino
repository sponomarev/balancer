#include <Servo.h>
#include <SPI.h>

#define MAX_SAMPLES 550 

// Arduino pins
#define MOTOR_PIN 3             // ESC
#define VALUE_PIN A0            // Potentiometer
#define IR_PIN 2                // LM393 IR sensor
#define CS_PIN 10               // ADXL345 CS
#define DATAREADY_PIN 4         // ADXL345 INT1

// ADXL345 registers
const char POWER_CTL = 0x2D;	  // Power Control
const char DATA_FORMAT = 0x31;  // Data Format
const char DATAX0 = 0x32;	      // X-Axis Data 0 (first byte)

Servo esc;

byte lowbyte_x[MAX_SAMPLES], highbyte_x[MAX_SAMPLES];
byte zero[MAX_SAMPLES];

void setup()
{
  // Setup ESC
  esc.attach(MOTOR_PIN);

  // Setup Serial
  Serial.begin(115200);

  // Setup LM393
  pinMode(IR_PIN, INPUT);

  // Setup ADXL345
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  pinMode(DATAREADY_PIN, INPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  writeRegister(POWER_CTL, 0x08);  // Measurement mode  
  writeRegister(DATA_FORMAT, 0x01); // +/- 4G range
  writeRegister(0x2C, 0x0F); // 3200 HZ
  writeRegister(0x2E, 0x80); // set dataready to interrupt pin INT1 and active high
}

void loop()
{
  process_esc();
  process_data();
}

void process_esc()
{
  int value = analogRead(VALUE_PIN);
  int pwm_speed = map(value, 0, 1023, 1000, 2000);
  esc.writeMicroseconds(pwm_speed);
}

void process_data()
{
  for (int sample_index = 0; sample_index < MAX_SAMPLES; sample_index++)
  { 
    while (digitalRead(DATAREADY_PIN) == LOW);

    unsigned char values[2];
    readRegister(DATAX0, 2, values);

    zero[sample_index] = digitalRead(IR_PIN);
    lowbyte_x[sample_index] = values[0];
    highbyte_x[sample_index] = values[1];
  }

  for (int t=0; t < MAX_SAMPLES; t++)
  {
    int x = ((int)highbyte_x[t] << 8) | (int)lowbyte_x[t];    
    Serial.print(x, DEC);
    Serial.print("\t");
    Serial.println(zero[t]);
  }
  Serial.println ("End");
}

void writeRegister(char registerAddress, unsigned char value)
{
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(registerAddress);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

void readRegister(char registerAddress, int numBytes, unsigned char * values)
{
  char address = 0x80 | registerAddress;
  if(numBytes > 1) address = address | 0x40;

  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address);
  for(int i=0; i < numBytes; i++)
  {
    values[i] = SPI.transfer(0x00);
  }
  digitalWrite(CS_PIN, HIGH);
}
