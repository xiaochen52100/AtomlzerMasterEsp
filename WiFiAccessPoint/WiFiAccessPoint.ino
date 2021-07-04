#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "AsyncUDP.h"
#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp
#include <Ticker.h>

#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY!)
#error Select ESP32 board.
#endif

#define LED_BUILTIN 2   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED
//adc参数
#define ADC1 34//ADC引脚

// Set these to your desired credentials.
const char *ssid = "ESPAP";
const char *password = "12345678";
AsyncUDP udp;
//温湿度传感器参数
DHTesp dht;
void tempTask(void *pvParameters);
bool getTemperature();
void triggerGetTemp();
/** Task handle for the light value read task */
TaskHandle_t tempTaskHandle = NULL;
/** Ticker for temperature reading */
Ticker tempTicker;
/** Comfort profile */
ComfortState cf;
/** Flag if task should run */
bool tasksEnabled = false;
/** Pin number for DHT11 data pin */
int dhtPin = 23;
//pwm参数 频率8k  占空比180-250
int freq = 8000;    // 频率
int channel1 = 0;    // 通道1
int channel2 = 1;    //
int channel3 = 2;    // 通道1
int channel4 = 3;    //
int resolution = 8;   // 分辨率
const int motor1 = 19;
const int motor2 = 32;
const int motor3 = 18;
const int motor4 = 33;
int pwmLow = 0;//180;
int pwmMiddle= 0;//= 210;
int pwmHigh = 0;//250;
int direction1=1;
int direction2=1;
#define MOTOR1(x) ledcWrite(channel1, x);
#define MOTOR2(x) ledcWrite(channel2, x);
//继电器初始化
const int DO1 = 14;
const int DO2 = 12;
const int DO1LED = 26;
const int DO2LED = 27;
#define DOUT1(x) digitalWrite(DO1,x)
#define DOUT2(x) digitalWrite(DO2,x)
#define DOUT1LED(x) digitalWrite(DO1LED,x)
#define DOUT2LED(x) digitalWrite(DO2LED,x)
void setPWM(int switch,int direction,int speed)
{
    if(switch==1)
    {
      if(direction==1)
      {
        ledcWrite(channel1, speed);
        ledcWrite(channel3, 0);
      }
      if(direction==2)
      {
        ledcWrite(channel3, speed);
        ledcWrite(channel1, 0);
      }
      
    }
    if(switch==2)
    {
      if(direction==1)
      {
        ledcWrite(channel2, speed);
        ledcWrite(channel4, 0);
      }
      if(direction==2)
      {
        ledcWrite(channel4, speed);
        ledcWrite(channel2, 0);
      }
      
    }
}
/**
   initTemp
   Setup DHT library
   Setup task and timer for repeated measurement
   @return bool
      true if task and timer are started
      false if task or timer couldn't be started
*/
bool initTemp() {
  byte resultValue = 0;
  // Initialize temperature sensor
  dht.setup(dhtPin, DHTesp::DHT22);
  Serial.println("DHT initiated");

  // Start task to get temperature
  xTaskCreatePinnedToCore(
    tempTask,                       /* Function to implement the task */
    "tempTask ",                    /* Name of the task */
    4000,                           /* Stack size in words */
    NULL,                           /* Task input parameter */
    5,                              /* Priority of the task */
    &tempTaskHandle,                /* Task handle. */
    1);                             /* Core where the task should run */

  if (tempTaskHandle == NULL) {
    Serial.println("Failed to start task for temperature update");
    return false;
  } else {
    // Start update of environment data every 20 seconds
    tempTicker.attach(2, triggerGetTemp);
  }
  return true;
}
/**
   triggerGetTemp
   Sets flag dhtUpdated to true for handling in loop()
   called by Ticker getTempTimer
*/
void triggerGetTemp() {
  if (tempTaskHandle != NULL) {
    xTaskResumeFromISR(tempTaskHandle);
  }
}
/**
   Task to reads temperature from DHT11 sensor
   @param pvParameters
      pointer to task parameters
*/
void tempTask(void *pvParameters) {
  Serial.println("tempTask loop started");
  while (1) // tempTask loop
  {
    if (tasksEnabled) {
      // Get temperature values
      getTemperature();
    }
    // Got sleep again
    vTaskSuspend(NULL);
  }
}
/**
   getTemperature
   Reads temperature from DHT11 sensor
   @return bool
      true if temperature could be aquired
      false if aquisition failed
*/
bool getTemperature() {
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0) {
    Serial.println("DHT11 error status: " + String(dht.getStatusString()));
    return false;
  }

  float heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
  float cr = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);

  String comfortStatus;
  switch (cf) {
    case Comfort_OK:
      comfortStatus = "Comfort_OK";
      break;
    case Comfort_TooHot:
      comfortStatus = "Comfort_TooHot";
      break;
    case Comfort_TooCold:
      comfortStatus = "Comfort_TooCold";
      break;
    case Comfort_TooDry:
      comfortStatus = "Comfort_TooDry";
      break;
    case Comfort_TooHumid:
      comfortStatus = "Comfort_TooHumid";
      break;
    case Comfort_HotAndHumid:
      comfortStatus = "Comfort_HotAndHumid";
      break;
    case Comfort_HotAndDry:
      comfortStatus = "Comfort_HotAndDry";
      break;
    case Comfort_ColdAndHumid:
      comfortStatus = "Comfort_ColdAndHumid";
      break;
    case Comfort_ColdAndDry:
      comfortStatus = "Comfort_ColdAndDry";
      break;
    default:
      comfortStatus = "Unknown:";
      break;
  };

  //Serial.println(" T:" + String(newValues.temperature) + " H:" + String(newValues.humidity) + " I:" + String(heatIndex) + " D:" + String(dewPoint) + " " + comfortStatus);

  char* tmpStr = (char*)malloc(32);
  memset(tmpStr, 0, 32);
  tmpStr[0] = 0x7f;
  int temperature = (int)(newValues.temperature*10);
  int humidity = (int)(newValues.humidity*10);
  uint32_t adc = analogRead(ADC1);// * 3500 / 4096;
  Serial.println("analog:" + String(adc));
  memcpy(tmpStr + 3, &temperature, 4);
  memcpy(tmpStr + 7, &humidity, 4);
  memcpy(tmpStr + 11, &adc, 4);
  udp.write((uint8_t*)tmpStr, 32);
  return true;
}
void setup() {
  //初始化继电器
  pinMode(DO1, OUTPUT);
  pinMode(DO2, OUTPUT);
  DOUT1(LOW);
  DOUT2(LOW);
  pinMode(DO1LED, OUTPUT);
  pinMode(DO2LED, OUTPUT);
  DOUT1LED(LOW);
  DOUT2LED(LOW);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  delay(1000);
  if (udp.listenMulticast(IPAddress(232, 11, 12, 13), 7000)) {
    Serial.print("UDP Listening on IP: ");
    //Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {
      Serial.print("UDP Packet Type: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
      Serial.print(", From: ");
      Serial.print(packet.remoteIP());
      Serial.print(":");
      Serial.print(packet.remotePort());
      Serial.print(", To: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Length: ");
      Serial.print(packet.length());
      Serial.print(", Data: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();
      char* tmpStr = (char*)malloc(packet.length() + 1);
      memcpy(tmpStr, packet.data(), packet.length());
      //tmpStr[packet.length()] = '\0';
      Serial.println(tmpStr[0]);
      free(tmpStr);
      if (tmpStr[0] == 0x7f)
      {
        pwmLow=tmpStr[5]+128;
        pwmMiddle=tmpStr[6]+128;
        pwmHigh=tmpStr[7]+128;
        direction1=tmpStr[8];
        direction2=tmpStr[9];
        if (tmpStr[1] == 0) //继电器1关
        {
          DOUT1(LOW);
          DOUT1LED(LOW);
          //MOTOR1(0);
          setPWM(1,direction1,0);
          
        }
        else if (tmpStr[1] == 1) //继电器1开
        {
          DOUT1(HIGH);
          DOUT1LED(HIGH);
          if (tmpStr[2] == 0) //电机1
          {
            //MOTOR1(0);
            setPWM(1,direction1,0);
          }
          else if (tmpStr[2] == 1) //电机1
          {
            //MOTOR1(pwmLow);
            setPWM(1,direction1,pwmLow);
          }
          else if (tmpStr[2] == 2) //电机1
          {
            //MOTOR1(pwmHigh);
            setPWM(1,direction1,pwmHigh);
          }
                  //              else if(tmpStr[3]==3)//电机1
        //              {
        //                MOTOR1(pwmHigh);
        //              }

        }
        if (tmpStr[3] == 0) //继电器1关
        {
          DOUT2(LOW);
          DOUT2LED(LOW);
          //MOTOR2(0);
          setPWM(2,direction2,0);
        }
        else if (tmpStr[3] == 1) //继电器1开
        {
          DOUT2(HIGH);
          DOUT2LED(HIGH);

          if (tmpStr[4] == 0) //电机1
          {
            //MOTOR2(0);
            setPWM(2,direction2,0);
          }
          else if (tmpStr[4] == 1) //电机1
          {
            //MOTOR2(pwmLow);
            setPWM(2,direction2,pwmLow);
          }
          else if (tmpStr[4] == 2) //电机1
          {
            //MOTOR2(pwmHigh);
            setPWM(2,direction2,pwmHigh);
          }
          //              else if(tmpStr[4]==3)//电机1
          //              {
          //                MOTOR2(pwmHigh);
          //              }
        }
        
      }
    });

  }
  //初始化传感器
  initTemp();
  tasksEnabled = true;
  //初始化ADC
  pinMode(ADC1, INPUT);
  analogReadResolution(12);
  //analogReadResolution(16);//设置aliogRead返回值的分辨率
  //初始化电机
  ledcSetup(channel1, freq, resolution); // 设置通道
  ledcAttachPin(motor1, channel1);  // 将通道与对应的引脚连接
  ledcSetup(channel2, freq, resolution); // 设置通道
  ledcAttachPin(motor2, channel2);  // 将通道与对应的引脚连接

  ledcSetup(channel3, freq, resolution); // 设置通道
  ledcAttachPin(motor3, channel3);  // 将通道与对应的引脚连接
  ledcSetup(channel4, freq, resolution); // 设置通道
  ledcAttachPin(motor4, channel4);  // 将通道与对应的引脚连接
  //MOTOR1(0);
  //MOTOR2(0);
  setPWM(1,direction1,0);
  setPWM(2,direction2,0);
}

void loop() {
  delay(5000);
  //Send multicast
  //udp.print("Anyone here?");
}
