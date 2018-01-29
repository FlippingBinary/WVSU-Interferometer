// This file can be saved as a ".ino" file and compiled in the Arduino IDE
#include <Arduino.h>
#include <Wire.h>
#include <WString.h>

#define SLAVE_ADDRESS 0x27
#define LED_RED 0x0
#define LED_YELLOW 0x1
#define LED_GREEN 0x2
#define GEIGER_A 0x20
#define GEIGER_B 0x21
#define GEIGER_C 0x22
#define GEIGER_D 0x23
#define SI_X 0x7
#define SI_Y 0x8
#define SI_Z 0x9
#define NOISE_X 0x99999
#define NOISE_Y 0x99999
#define NOISE_Z 0x99999

//-- For signal detectors
void readSignal(uint8_t, signal_t &);
typedef struct signal_t
{
  volatile uint8_t count = 0;
  uint8_t last = LOW;
  uint8_t state = LOW;
} signal_t;

//-- For Geiger circuits
signal_t geiger_a;
signal_t geiger_b;
signal_t geiger_c;
signal_t geiger_d;

//-- For silicone detectors
signal_t si_x;
signal_t si_y;
signal_t si_z;
signal_t noise_x;
signal_t noise_y;
signal_t noise_z;

//-- For i2c
void receiveData(int byteCount);
void sendData();
String message;

void setup()
{
  //  asm(".global _printf_float"); // This enables the use of float in printf
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  pinMode(GEIGER_A, INPUT);
  pinMode(GEIGER_B, INPUT);
  pinMode(GEIGER_C, INPUT);
  pinMode(GEIGER_D, INPUT);

  pinMode(SI_X, INPUT);
  pinMode(SI_Y, INPUT);
  pinMode(SI_Z, INPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, LOW);

  Serial.begin(9600); // start USB serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  if (Serial)
    Serial.println("Ready !");
  message.reserve(256);
}

void loop()
{
  //-- Read Geiger circuits
  readSignalFalling(GEIGER_A, geiger_a);
  readSignalFalling(GEIGER_B, geiger_b);
  readSignalFalling(GEIGER_C, geiger_c);
  readSignalFalling(GEIGER_D, geiger_d);

  //-- Read silicone detectors
  readSignalFalling(SI_X, si_x);
  readSignalFalling(SI_Y, si_y);
  readSignalFalling(SI_Z, si_z);

  //-- Read silicone noise
  readSignalRising(NOISE_X, noise_x);
  readSignalRising(NOISE_Y, noise_y);
  readSignalRising(NOISE_Z, noise_z);

  Serial.print("Counted: ");
  Serial.print(geiger_a.count);
  Serial.print(", ");
  Serial.print(geiger_b.count);
  Serial.print(", ");
  Serial.print(geiger_c.count);
  Serial.print(", ");
  Serial.print(geiger_d.count);
  Serial.print(" at ");
  Serial.println(millis());

  delay(250);
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
  signal_t g_a, g_b, g_c, g_d;
  signal_t s_x, s_y, s_z;
  signal_t n_x, n_y, n_z;
  unsigned char sreg_backup;

  //-- Backup interrupt state
  sreg_backup = SREG;
  //-- Disable interrupts
  cli();
  //-- DON'T DO SLOW STUFF WHILE INTERRUPTS ARE DISABLED!
  g_a = geiger_a;
  g_b = geiger_b;
  g_c = geiger_c;
  g_d = geiger_d;
  geiger_a.count = 0;
  geiger_b.count = 0;
  geiger_c.count = 0;
  geiger_d.count = 0;
  s_x = si_x;
  s_y = si_y;
  s_z = si_z;
  si_x.count = 0;
  si_y.count = 0;
  si_z.count = 0;
  n_x = noise_x;
  n_y = noise_y;
  n_z = noise_z;
  noise_x.count = 0;
  noise_y.count = 0;
  noise_z.count = 0;
  //-- Restore interrupt state
  SREG = sreg_backup;
  //-- Polling is safe again
  char message[256];
  sprintf(message, "Geiger: [%03d, %03d, %03d, %03d] at %d",
          g_a.count, g_b.count, g_c.count, g_d.count,
          millis());
  Serial.print("Sending \"");
  Serial.print(message);
  Serial.println("\"");
  Wire.write(message);
}

void readSignalFalling(uint8_t pin, signal_t &sig)
{
  sig.last = sig.state;
  sig.state = digitalRead(pin);
  if ((sig.last == HIGH) && (sig.state == LOW))
  {
    sig.count++;
  }
}

void readSignalRising(uint8_t pin, signal_t &sig)
{
  sig.last = sig.state;
  sig.state = digitalRead(pin);
  if ((sig.last == LOW) && (sig.state == HIGH))
  {
    sig.count++;
  }
}
