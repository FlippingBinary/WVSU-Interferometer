#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial2

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

IntervalTimer myTimer;

int led = 13;
int stage = 0;
int lasttime = 0;


void displaySensorDetails(void)
{
  sensor_t accel, mag;
  accelmag.getSensor(&accel, &mag);
  sensor_t sensor;
  gyro.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("GYROSCOPE");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(sensor.sensor_id, HEX);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(accel.name);
  Serial.print  ("Driver Ver:   "); Serial.println(accel.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(accel.sensor_id, HEX);
  Serial.print  ("Min Delay:    "); Serial.print(accel.min_delay); Serial.println(" s");
  Serial.print  ("Max Value:    "); Serial.print(accel.max_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(accel.min_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(accel.resolution, 8); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(mag.name);
  Serial.print  ("Driver Ver:   "); Serial.println(mag.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(mag.sensor_id, HEX);
  Serial.print  ("Min Delay:    "); Serial.print(accel.min_delay); Serial.println(" s");
  Serial.print  ("Max Value:    "); Serial.print(mag.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(mag.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(mag.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");

  delay(500);
}

void takeSample(void) {
  sensors_event_t aevent, mevent;
  
  /* Get a new sensor event */
  sensors_event_t event;
  gyro.getEvent(&event);
  /* Get a new sensor event */
  accelmag.getEvent(&aevent, &mevent);
/*
  Serial.print("G ");
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");
  Serial.println("rad/s ");

  Serial.print("A ");
  Serial.print("X: "); Serial.print(aevent.acceleration.x, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(aevent.acceleration.y, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(aevent.acceleration.z, 4); Serial.print("  ");
  Serial.println("m/s^2");

  Serial.print("M ");
  Serial.print("X: "); Serial.print(mevent.magnetic.x, 1); Serial.print("  ");
  Serial.print("Y: "); Serial.print(mevent.magnetic.y, 1); Serial.print("  ");
  Serial.print("Z: "); Serial.print(mevent.magnetic.z, 1); Serial.print("  ");
  Serial.println("uT");

  Serial.println("");
*/
  Serial.print(millis()); Serial.print(", ");
  Serial.print(event.gyro.x, 4); Serial.print(", ");
  Serial.print(event.gyro.y, 4); Serial.print(", ");
  Serial.print(event.gyro.z, 4); Serial.print(", ");
  Serial.print(aevent.acceleration.x, 4); Serial.print(", ");
  Serial.print(aevent.acceleration.y, 4); Serial.print(", ");
  Serial.print(aevent.acceleration.z, 4); Serial.print(", ");
  Serial.print(mevent.magnetic.x, 4); Serial.print(", ");
  Serial.print(mevent.magnetic.y, 4); Serial.print(", ");
  Serial.print(mevent.magnetic.z, 4); Serial.println("");

  HWSERIAL.print(millis()); HWSERIAL.print(", ");
  HWSERIAL.print(event.gyro.x, 4); HWSERIAL.print(", ");
  HWSERIAL.print(event.gyro.y, 4); HWSERIAL.print(", ");
  HWSERIAL.print(event.gyro.z, 4); HWSERIAL.print(", ");
  HWSERIAL.print(aevent.acceleration.x, 4); HWSERIAL.print(", ");
  HWSERIAL.print(aevent.acceleration.y, 4); HWSERIAL.print(", ");
  HWSERIAL.print(aevent.acceleration.z, 4); HWSERIAL.print(", ");
  HWSERIAL.print(mevent.magnetic.x, 4); HWSERIAL.print(", ");
  HWSERIAL.print(mevent.magnetic.y, 4); HWSERIAL.print(", ");
  HWSERIAL.print(mevent.magnetic.z, 4); HWSERIAL.println("");
}

void setup(void)
{
  pinMode(led, OUTPUT);     
  digitalWrite(led, HIGH);
  Serial.begin(9600);
  HWSERIAL.begin(9600);
  /* Wait for the Serial Monitor */
  while (!HWSERIAL) {
    delay(1);
  }

  /* Initialise the sensor */
  if(!gyro.begin())
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while(1);
  }

  /* Initialise the sensor */
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();

//  myTimer.begin(takeSample, 20000); // 1000000 == 1 second
/*
  int incomingByte;
  while ( stage < 3 ) {
    if (HWSERIAL.available() > 0) {
      incomingByte = HWSERIAL.read();
      Serial.print("UART received: ");
      Serial.print(incomingByte, DEC);
      if ( incomingByte == '1' ) Serial.print(" FOUND 1!");
      if ( incomingByte == '2' ) Serial.print(" FOUND 2!");
      if ( incomingByte == '<' ) Serial.print(" FOUND <!");
      Serial.println("");
      Serial.println("");
      Serial.println("");
    } else {
      Serial.println("No data");
      delay(100);
    }
  }
  */
  delay(100);

  HWSERIAL.println("gyroX,gyroY,gyroZ,acclX,acclY,acclZ,magX,magY,magZ");
  while ( !Serial ) delay(1);
}

void loop(void)
{
  if ( ( millis() - lasttime ) >= 20 ) {
    lasttime = millis();
    takeSample();
  }
}

