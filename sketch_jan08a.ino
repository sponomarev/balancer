#include <Servo.h>
#include <SPI.h>

#define MAX_SAMPLES 550

#define MOTOR_PIN 3
#define VALUE_PIN A0

#define IR_PIN 2
#define CS_PIN 10

// ADXL345 constants
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0

//This buffer will hold values read from the ADXL345 registers.
unsigned char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x;

byte lowbyte_x[MAX_SAMPLES], highbyte_x[MAX_SAMPLES];
byte zero[MAX_SAMPLES];

int val = 0;  
int sample_index,t;
const int dataReadyPin = 4;

Servo esc;

void setup()
{
  // Setup ESC
  esc.attach(MOTOR_PIN);

  // Setup SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);

  // Setup Serial
  Serial.begin(115200);

  pinMode(dataReadyPin, INPUT);
  pinMode(IR_PIN, INPUT);

  // Setup ADXL345
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  writeRegister(POWER_CTL, 0x08);  // Measurement mode  
  writeRegister(DATA_FORMAT, 0x02); // +/- 4G range
  writeRegister(0x2C, 0x0F); // 3200 HZ
  writeRegister(0x2E, 0x80); // set dataready to interrupt pin INT1 and active high

  delay(100);  
}

void loop()
{
  dispatch_esc();

  for (sample_index = 0; sample_index < MAX_SAMPLES; sample_index++)
  { 
    while (digitalRead(dataReadyPin) == LOW);
    readRegister(DATAX0, 6, values);

    zero[sample_index] = digitalRead(IR_PIN) ;

    lowbyte_x[sample_index]=values[0];
    highbyte_x[sample_index]=values[1];
  }

  for (t=0; t < MAX_SAMPLES; t++)
  {
    x = ((int)highbyte_x[t] << 8) | (int)lowbyte_x[t];    
    Serial.print(x, DEC);
    Serial.print("\t");
    Serial.println(zero[t]);   
  }
  Serial.println ("End");
}

void dispatch_esc()
{
  int value = analogRead(VALUE_PIN);
  value = map(value, 0, 1023, 1000, 2000);
  esc.writeMicroseconds(value);

  Serial.println(value);
}

void writeRegister(char registerAddress,unsigned char value){
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(registerAddress);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

void readRegister(char registerAddress, int numBytes,unsigned char * values){
  char address = 0x80 | registerAddress;

  if(numBytes > 1) address = address | 0x40;  
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address);
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  digitalWrite(CS_PIN, HIGH);
}



