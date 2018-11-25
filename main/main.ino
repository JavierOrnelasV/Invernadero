//    Includes
#include <ESP8266WiFi.h>
#include <DNSServer.h>
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
Adafruit_MQTT_Subscribe bomba = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/bomba");

//    Declaraciones
#define relePin      4
#define led1Pin      0
#define led2Pin      2
#define baudSpeed    115200

//    Codigo
bool regadera = false;
int tiempoRiego = 5; //En segundos
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 60000;
void MQTT_connect();

void setup() {
  Serial.begin(baudSpeed);

  pinMode(relePin, OUTPUT);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  digitalWrite(relePin, HIGH);
  digitalWrite(led1Pin, HIGH);
  digitalWrite(led2Pin, HIGH);

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

  mqtt.subscribe(&bomba);
  startMillis = millis();
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

void leer(){
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    bombaF(subscription);
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

void loop() {
  MQTT_connect();
  currentMillis = millis();
  if(currentMillis - startMillis >= period){
    //Leer sensores
    startMillis = currentMillis;
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
