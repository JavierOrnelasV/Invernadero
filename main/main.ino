//    Includes
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <Wire.h>
#include <OneWire.h>                
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

//    Credenciales WiFi
//#define WLAN_SSID       "Spider 2.4Ghz"
//#define WLAN_PASS       "033aa8e7"

//    Servidor Adafruit
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "naitomea351"
#define AIO_KEY         "85afa34c6cde4896bc5945fdb92daa33"

//    Conexion al servidor
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//    Feeds
Adafruit_MQTT_Publish bombaP = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bomba");
Adafruit_MQTT_Publish humedadP = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humedad");
Adafruit_MQTT_Publish luzP = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/luz");
Adafruit_MQTT_Subscribe bomba = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/bomba");
Adafruit_MQTT_Subscribe humedadS = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/humedad");

//    Declaraciones
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
#define relePin      15
#define led1Pin      0
#define led2Pin      2
#define soilPin      A0
#define soilPower    16
#define baudSpeed    115200
OneWire ourWire(13);
DallasTemperature temperature(&ourWire);

//    Codigo
bool regadera = false;
int tiempoRiego = 1; //En segundos
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 30000;
int j = 0;
void MQTT_connect();

//    Sensor de Luz - Display
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:          "); Serial.println(sensor.name);
  Serial.print  ("Valor maximo:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Valor minimo:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolucion:      "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

//    Sensor de Luz - Config
void configureSensor(void)
{
  //tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  //tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}

void setup() {
  Serial.begin(baudSpeed);

  pinMode(relePin, OUTPUT);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(soilPower, OUTPUT);
  digitalWrite(relePin, HIGH);
  digitalWrite(led1Pin, HIGH);
  digitalWrite(led2Pin, HIGH);
  digitalWrite(soilPower, LOW);

  /*
  Serial.println(); Serial.println();
  Serial.print("Conectando a:  ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  */

  WiFiManager wifiManager;
  //wifiManager.resetSettings();
  wifiManager.setAPStaticIPConfig(IPAddress(192,168,0,1), IPAddress(192,168,0,1), IPAddress(255,255,255,0));
  digitalWrite(led2Pin, LOW);
  wifiManager.autoConnect("Micros 2", "password");

  Serial.println("WiFi conectado");
  digitalWrite(led2Pin, HIGH);
  Serial.println("Direccion IP: "); Serial.println(WiFi.localIP());

  if(!tsl.begin())
  {
    Serial.print("Error con sensor de luz");
    while(1);
  }

  displaySensorDetails();
  configureSensor();
  temperature.begin();

  mqtt.subscribe(&bomba);
  mqtt.subscribe(&humedadS);
  startMillis = millis();
}

int readSoil()
{
    int soilValTemp = 0;
    digitalWrite(soilPower, HIGH);
    delay(10);
    soilValTemp = analogRead(soilPin); 
    digitalWrite(soilPower, LOW);
    return soilValTemp;
}

void bombaF(Adafruit_MQTT_Subscribe *subscription){
  if (subscription == &bomba) {
    Serial.print(F("Bomba: "));
    Serial.println((char *)bomba.lastread);
    if (strcmp((char *)bomba.lastread, "Encendida") == 0){
      digitalWrite(relePin, LOW);
      digitalWrite(led2Pin, LOW);
      regadera = true;
    }
    else if (strcmp((char *)bomba.lastread, "Apagada") == 0){
      digitalWrite(relePin, HIGH);
      digitalWrite(led2Pin, HIGH);
    }
  }
}

void humedadF(Adafruit_MQTT_Subscribe *subscription){
  if (subscription == &humedadS) {
    Serial.print(F("Humedad: "));
    Serial.println((char *)humedadS.lastread);
    uint16_t agh = atoi((char *)humedadS.lastread);
    if (agh < 20){
      encender();
    }
  }
}

void leer(){
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    bombaF(subscription);
    humedadF(subscription);
  }
}

void encender(){
  char val[] = "Encendida";
  Serial.println(F("Enviando valor"));
  Serial.print("...");
  if (!bombaP.publish(val)) {
    Serial.println(F("Fallo"));
  } else {
    Serial.println(F("Enviado!"));
  }
}

void apagar(){
  char val[] = "Apagada";
  Serial.println(F("Enviando valor"));
  Serial.print("...");
  if (!bombaP.publish(val)) {
    Serial.println(F("Fallo"));
  } else {
    Serial.println(F("Enviado!"));
    regadera = false;
  }
}

void humedad(int soil){
  Serial.println(F("Enviando valor"));
  Serial.print("...");
  if (!humedadP.publish(soil)) {
    Serial.println(F("Fallo"));
  } else {
    Serial.println(F("Enviado!"));
  }
}

void luz(int luz){
  luz = luz * 20;
  Serial.println(F("Enviando valor"));
  Serial.print("...");
  if (!luzP.publish(luz)) {
    Serial.println(F("Fallo"));
  } else {
    Serial.println(F("Enviado!"));
  }
}

void temperatura(int tempu){
  Serial.println(F("Enviando valor"));
  Serial.print("...");
  if (!luzP.publish(tempu)) {
    Serial.println(F("Fallo"));
  } else {
    Serial.println(F("Enviado!"));
  }
}

void loop() {
  MQTT_connect();
  currentMillis = millis();
  if((currentMillis - startMillis) >= period){
    startMillis = currentMillis;
    humedad(readSoil());
    
    sensors_event_t event;
    tsl.getEvent(&event);
    if (event.light){
      luz(event.light);
      Serial.println(event.light);
    }
    else{
      Serial.println("Sobrecarga de sensor de luz");
    }
    delay(250);
    temperature.requestTemperatures();
    float tempu = temperature.getTempCByIndex(0);
    temperatura(tempu);
  }
  leer();
  
  /*
  if(!mqtt.ping()) {
    mqtt.disconnect();
  }
  */

  if(regadera){
    delay(tiempoRiego * 1000);
    apagar();
  }
}

void MQTT_connect() {
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Conectando a servidor... ");
  int8_t ret;
  int8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Reintentando conexion al servidor en 5 segundos...");
       mqtt.disconnect();
       delay(5000);
       retries--;
       if (retries == 0) {
         while (1);
       }
  }
  Serial.println("Servidor conectado");
  digitalWrite(led1Pin, LOW);
}
