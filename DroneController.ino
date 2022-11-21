#include <ESP8266WiFi.h>

//libraries for the Mpu 6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//Wifi password and SSid
const char* ssid =  "ZENILION 7302";     
const char* pass =  "683Z*6j0";

//AWS 
#define AWS_IOT_PUBLISH = "";
#define AwS_IOT_SUBSCRIBE = "";

//MPU class
Adafruit_MPU6050 mpu;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//Gyro Correction ---- needed to be calibrated //File>Examples>ADAfruit 6050>Basic_Reading  ------
float gyroXerror = 0.04;
float gyroYerror = 0.00;
float gyroZerror = -0.02;

//sonsor events


//declaring Motorpins

uint8_t D5=12;// left front
uint8_t D6=13;//left back
uint8_t D7=14;//right front
uint8_t D8=15;//right back

//4leds ---- covered by 1 pin in parallel 
//unit8_t D9;

//input parameters
float Thrust = 20;

//pid gains
float pid_p_gain = 1f;
float pid_i_gain = 1f;
float pid_d_gain = 1f;

float actuallPID = pid_p_gain+pid_d_gain+pid_i_gain;



//Target 
float targetX;
float targetY;
float targetZ;



//runs at startup
void setup() 
{   
    pinMode(LED_BUILTIN, OUTPUT);  // initialising Inbuilt Led for Indication
    Serial.begin(9600);// band of instruction for the board

    //initialising motorpins to output mode
    pinMode(D5,OUTPUT);
    pinMode(D6,OUTPUT);
    pinMode(D7,OUTPUT);
    pinMode(D8,OUTPUT);
    //WiFi.setTransportToUDP(); // using udp for low latency   ----- change if data not recieved properly

    initMPU();
    CaliberateGyroSensors();
    
    //calling function at at startup to connect to wifi
    connecttoWifi();
}


//runs per clockCycle
void loop() 
{          
  // getReadings(); 
  // delay(1000);

  delay(10000);
  Serial.print("OK");
  throttle();

}

//initializing MPU
void initMPU()
{
  if(!mpu.begin())
  {
    Serial.println("MPU not Booted");
  }
  Serial.println("MPU is running");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);  
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void throttle()
{
  analogWrite(D5,Thrust);
  analogWrite(D6,Thrust);
  analogWrite(D7,Thrust);
  analogWrite(D8,Thrust);
}


//connection with Wifi
void connecttoWifi()
{ 

   WiFi.begin(ssid, pass); 
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    
    
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);

      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
  }
          
    Serial.println(ssid); 
    digitalWrite(LED_BUILTIN, LOW);
  
}

void AWSconnect()
{

}


void getReadings()
{ 
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a,&g,&temp);

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  gyroX=(float)g.gyro.x-gyroXerror;
  gyroY=(float)g.gyro.y-gyroYerror;
  gyroZ=(float)g.gyro.z-gyroZerror;

  temperature = temp.temperature;
  
  Serial.println(gyroX);
  Serial.println(gyroY);
  Serial.println(gyroZ);
  Serial.print("");
  Serial.println(temperature);
}




void landSeq()//on connection lost slowly decreases altitude
{ 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a,&g,&temp);
  if(g.gyro.z == 0)
  {
    return;
  }
  float accreq = -0.4f;
  if (g.gyro.z>accreq) 
  {
    analogWrite(D5, analogRead(D5)-0.3);
    analogWrite(D6, analogRead(D6)-0.3);
    analogWrite(D7, analogRead(D7)-0.3);
    analogWrite(D8, analogRead(D8)-0.3);
  }
  landSeq();
}

void CaliberateGyroSensors()
{ 

  // to be done by levelling the gyro Sensor
  sensors_event_t a, g, temp;
  mpu.getEvent(&a,&g,&temp);

  gyroXerror= (float)g.gyro.x;
  gyroYerror= (float)g.gyro.y;
  gyroZerror= (float)g.gyro.z;


}