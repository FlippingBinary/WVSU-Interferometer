// This file can be saved as a ".ino" file and compiled in the Arduino IDE
#include <Arduino.h>
#include <Wire.h>
#include <WString.h>

#define SLAVE_ADDRESS 0x27
#define LED_RED 0x0
#define LED_YELLOW 0x1
#define LED_GREEN 0x2
int number = 0;
int state = 0;
String message;

void receiveData(int byteCount);
void sendData();

void setup()
{
//  asm(".global _printf_float");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, LOW);
  
  Serial.begin(9600); // start USB serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("Ready !");
  message.reserve(256);
}

void loop()
{
  delay(100);
}

// callback for received data
void receiveData(int byteCount)
{

  while (Wire.available())
  {
    message = Wire.readString(byteCount);
    Serial.print("data received: ");
    Serial.println(message);

    switch (message[0])
    {
    case 1:
    case 'r':
    case 'R':
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_GREEN, LOW);
      break;
    case 2:
    case 'y':
    case 'Y':
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_YELLOW, HIGH);
      digitalWrite(LED_GREEN, LOW);
      break;
    case 3:
    case 'g':
    case 'G':
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_GREEN, HIGH);
      break;
    default:
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_GREEN, LOW);
      break;
    }
  }
}

// callback for sending data
void sendData()
{
  if (++number >= 'z')
    number = '0';
  message = String("Hello, ");
  message += number;
  message += ", how are you?\n";
  Serial.print("Sending ");
  Serial.print(message);
  Wire.write(message.c_str());
}

