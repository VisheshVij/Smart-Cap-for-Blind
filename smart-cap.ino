//THIS IS CODE FOR SMART CAP (FOR ARDUINO NANO) by Vishesh Vij

//Libraries Used
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//Connections 
#define trigPin1 A0 //used analog pins because it was near to make the solder connections you can use any other Digital pin too
#define echoPin1 A1
#define motorRight 9
#define motorLeft 10
#define trigPin2 7
#define echoPin2 6
#define trigPin3 5
#define echoPin3 4
#define smsPin   12

// MPU6050 I2C address
const int MPU_addr = 0x68; 
int16_t AcX, AcY, AcZ;

// Thresholds - these may need tuning based on your specific cap
float thresholdHigh = 2.5; // G-force spike indicating impact
float thresholdLow = 0.4;  // G-force drop indicating freefall
bool fallDetected = false;

//Ultrasonic sensors 
long duration, distance, FIRSTSensor,SECONDSensor,THIRDSensor;

void setup()
{
  Serial.begin (115200);

  // --- MPU6050 Connection Check ---
  Wire.beginTransmission(MPU_addr);
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("Success: MPU6050 found and active.");
    
    // Wake up the MPU6050 since it starts in sleep mode
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); 
    Wire.write(0);    
    Wire.endTransmission(true);
  } 
  else {
    Serial.println("ERROR: MPU6050 not found! Check SDA/SCL wiring.");
    // Halt the program if the sensor is missing (optional)
    while(1) { 
      digitalWrite(13, HIGH); // Blink LED to signal error
      delay(200);
      digitalWrite(13, LOW);
      delay(200);
    }
  }

  pinMode(motorRight,OUTPUT);
  pinMode(motorLeft,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(smsPin,OUTPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  //digitalWrite(13, LOW);

  Serial.println("Smart-Cap is Ready!");
}


void loop() 

{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // Start with register 0x3B (Accel X)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);

  // Read Accelerometer data
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  // Convert raw values to G-force (for +/- 2g range)
  float totalAcc = sqrt(pow(AcX, 2) + pow(AcY, 2) + pow(AcZ, 2)) / 16384.0;

  digitalWrite(motorRight, LOW);
  digitalWrite(motorLeft, LOW);
  digitalWrite(smsPin, LOW);

  if (totalAcc > thresholdHigh) {
    Serial.println("IMPACT DETECTED!");
    delay(500); // Wait briefly to check orientation after fall
    
    // 2. Post-Fall Check (Are they still down?)
    // Re-read data to see if the user is stationary/horizontal
    if (checkInactivity()) {
      triggerEmergency();
    }
  }
  
SonarSensor(trigPin1, echoPin1);
FIRSTSensor = distance;
SonarSensor(trigPin2, echoPin2);
SECONDSensor = distance;
SonarSensor(trigPin3, echoPin3);
THIRDSensor = distance;
Serial.print("S1:");Serial.println(FIRSTSensor); delayMicroseconds(10);
Serial.print("S2:");Serial.println(SECONDSensor);delayMicroseconds(10);
Serial.print("S3:");Serial.println(THIRDSensor); delayMicroseconds(10);



if((FIRSTSensor <= 10)) 
{digitalWrite(motorLeft, HIGH);digitalWrite(motorRight, LOW);delay(100);}
else if((FIRSTSensor >= 10)){
  digitalWrite(motorRight, LOW);
  digitalWrite(motorLeft, LOW);
}


if((SECONDSensor <= 10) ) 
{digitalWrite(motorRight, HIGH);digitalWrite(motorLeft, HIGH);delay(100);}
else if((SECONDSensor >= 10) ) {
  digitalWrite(motorRight, LOW);digitalWrite(motorLeft, LOW);
}

if((THIRDSensor <= 10) ) 
{digitalWrite(motorRight, HIGH);digitalWrite(motorLeft, LOW);delay(100);}
else if((THIRDSensor >= 10) ){
  digitalWrite(motorRight, LOW);digitalWrite(motorLeft, LOW);
}


//resetFunc();
digitalWrite(motorLeft, LOW);digitalWrite(motorRight, LOW);digitalWrite(smsPin, LOW);
}

bool checkInactivity() {
  // Logic to check if the user has remained still for 2 seconds
  // You can compare previous acceleration to current acceleration
  return true; // Simplified for this snippet
}

void triggerEmergency() {
  Serial.println("FALL CONFIRMED: Sending SMS via ESP8266...");
  digitalWrite(13, HIGH); // Light up LED
  digitalWrite(smsPin,HIGH);
  delay(1000);
  digitalWrite(smsPin,LOW);
  delay(200);
  
  // Here you would send a signal to your ESP8266 
}

void SonarSensor(int trigPin,int echoPin)
{
  // Calculates distance for each Ultrasonic sensor
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance = (duration/2) / 29.1;
}
