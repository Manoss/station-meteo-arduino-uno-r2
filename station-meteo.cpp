#include <Bme280BoschWrapper.h>
#include "Wire.h"
#include "Adafruit_Sensor.h"

//Particules sensor
#include "SDS011.h"

/****
//MQ135
#include "MQ135.h"
****/

//MH-Z19
#include "MHZ19.h"

//WIFI
#include "WiFiNINA.h"

//MQTT
#include <ArduinoMqttClient.h>

//Secrets
#include "secrets.h"

//SDS011
#define SDS011_RXD 4 //TxD du module
#define SDS011_TXD 7 //RxD du module

//MH-Z19 CO2
#define MH_Z19_PWM 5 //PWM 

/**********************
 *  Variables
 **********************/
   
//Wifi
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;
IPAddress ip(192,168,1,101);

//MQTT
const char  broker[]  = MQTT_HOST;
int         port      = 1883;
const char  topic[]   = MQTT_TOPIC_NAME;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

/***
//MQ135
const int mq135Pin = 3;
MQ135 gasSensor = MQ135(mq135Pin);
***/

//Adafruit_BME280 bme;
Bme280BoschWrapper bme280(true);

//SDS011
float p10,p25;
SDS011 my_sds;

//MH-Z19
unsigned long duration;

void setup() {
  
  Serial.begin(9600);
  while(!Serial); //Wait until connection
  
  Serial.println("Broker MQTT OK");
  
  /**********************
  *  BME280 Sensor
  **********************/
  while(!bme280.beginI2C(0x76))
  {
    Serial.println("BME not found");
    delay(1000);
  }

  /**********************
  *  CO2 Sensor MH-Z19
  **********************/
  pinMode(MH_Z19_PWM, INPUT);
  
  /**********************
  *  SDS011 Particules Sensor
  **********************/
  my_sds.begin(SDS011_RXD,SDS011_TXD);

  /**********************
  *  WIFI Config
  **********************/
  WiFi.config(ip);
  
}

void loop() {

  Serial.println("---- Start Loop ----");

   /**********************
   *  WIFI
   **********************/
  status = WiFi.begin(ssid, pass);
  
  if (status != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(2000);
  }
  Serial.println(WiFi.localIP());
  
  /**********************
   *  MQTT Broker
   **********************/
  while (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
  }
  
  my_sds.wakeup();
  delay(30000); //warm
  my_sds.read(&p25,&p10);
  
  //BME0280
  bme280.measure();
  float humidity    = (bme280.getHumidity()/1024.0);
  float temperature = (bme280.getTemperature()/100.0);
  float pressure    = (bme280.getPressure()/100.0);
  /*****
  //MQ135
  float rzero = gasSensor.getCorrectedRZero(temperature,humidity);
  float ppm = gasSensor.getCorrectedPPM(temperature,humidity);   
  ******/
  //MH-Z19
  duration = pulseIn(MH_Z19_PWM, HIGH, 2008000);
  float ppm = getPPM(duration);
  
  mqttClient.poll();
  
  mqttClient.beginMessage(MQTT_TOPIC_NAME);
  mqttClient.print("{\"PM25\":");
  mqttClient.print(p25);
  mqttClient.print(",");
  mqttClient.print("\"PM10\":");
  mqttClient.print(p10);
  mqttClient.print(",");
  mqttClient.print("\"T\":");
  mqttClient.print(temperature);
  mqttClient.print(",");
  mqttClient.print("\"H\":");
  mqttClient.print(humidity);
  mqttClient.print(",");
  mqttClient.print("\"P\":");
  mqttClient.print(pressure);
  mqttClient.print(",");
  /**
  mqttClient.print("\"R0\":");
  mqttClient.print(rzero);
  mqttClient.print(",");
  **/
  mqttClient.print("\"CO2\":");
  mqttClient.print(ppm);
  mqttClient.print("}");

  mqttClient.endMessage();

  //debug
  Serial.print("PM25: ");
  Serial.println(p25);
  Serial.print("PM10: ");
  Serial.println(p10);
  Serial.print("Température: ");
  Serial.println(temperature);
  Serial.print("Humidité: ");
  Serial.println(humidity);
  Serial.print("Pression: ");
  Serial.println(pressure);
  Serial.print("CO2: ");
  Serial.println(ppm);
  Serial.print("Duration MH-Z19: ");
  Serial.println(duration);
 
  //SDS011 Protection
  my_sds.sleep();
  Serial.println("SDS Sleep");
  
  //Sleep WIFI
  WiFi.disconnect();
  Serial.println("Wifi Deconnected");
  
  Serial.println("----------------------");
  //delay(300000);//5 min 
  delay(1770000);//30 min
}
