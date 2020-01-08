

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define SEC_TO_MILISEC(x) ((x)*1000) 




char * ssid ="WIFI Pier";
char * pass ="pagle736pagle";
unsigned int mqttPort=1883;

const char MqttUser[]="powerQuality";
const char MqttPassword[]="1234";
const char MqttClientID[]="powerQuality";
bool sendDataE=false;
IPAddress mqttServer(192,168,0,116);

WiFiClient wclient;
PubSubClient mqtt_client(wclient);

void callback(char* topic, byte* payload, unsigned int length);
void  debug_message (char * string, bool newLine)
{
#ifdef DEBUGG
  if(string !=NULL)
  {
    if (!newLine)
    {
      Serial.print(string);
    }else
    {
      Serial.println(string);
    }
  }
  #endif
}

void setUpWifi(char * ssid, char * pass)
{
  String ip;
  debug_message(" ",true);
  debug_message(" ",true);
  debug_message("Conectandose a: ",false);
  debug_message(ssid,true);

  WiFi.begin(ssid,pass);

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    debug_message(".",false);
  }
  debug_message(" ",true);
  debug_message("Coneccion realizada",true);
  debug_message("La ip es: ",false);
  ip=WiFi.localIP().toString();
  debug_message((char *)ip.c_str(),true);
}

void setUpMqtt(void)
{
  mqtt_client.setServer(mqttServer,mqttPort);
  mqtt_client.setCallback(callback);
}


void callback(char* topic, byte* payload, unsigned int length)
{
  int tiempo=0;
  payload[length]='\n';
  String message((char *)payload);
  debug_message("Llego un mensage, topic:",false);
  debug_message(topic,false);
  debug_message(", payload : ",false);
  debug_message((char *)payload,true);

  if(!strcmp(topic,"power/medir"))
  {
    Serial.println("med");
   
  }else if(!strcmp(topic,"power/get")){
     sendDataE=true;
    // Serial.println("getF");
  }
 
}


void setup() {

  setUpWifi(ssid,pass);
  setUpMqtt();

  // Initialize Serial port
  Serial.begin(9600);

}

void reconnect()
{
  while(!mqtt_client.connected())
  {
    debug_message("Intentando conectar al servidor MQTT",true);
    if (mqtt_client.connect(MqttClientID,MqttUser,MqttPassword))
      {
            debug_message("conectado",true);
  
  
            // ...suscrivirse a topicos
            mqtt_client.subscribe("power/medir");
            mqtt_client.subscribe("power/get");

           // mqtt_client.subscribe("timbre/off");
            


      }
      else
      {
        debug_message("intentando conetarse al broker",true);
        delay(3000);
      }
  }
}

void loop() {
  String temData;

  if (!mqtt_client.connected()) 
  {
      reconnect();
 }
 mqtt_client.loop(); 
 if(sendDataE){
  Serial.println("i");
  temData=waitData();
  mqtt_client.publish("power/irms",temData.c_str());
  Serial.println("v");
  temData=waitData();
 
  mqtt_client.publish("power/vrms",temData.c_str());
  
  Serial.println("p");
  temData=waitData();
  mqtt_client.publish("power/p",temData.c_str());
  Serial.println("f");
  temData=waitData();
  mqtt_client.publish("power/pf",temData.c_str());
  
  sendDataE=false;
 }

  
 // delay(100);
}

String waitData(void){
  while(!(Serial.available()>0));
  return Serial.readString();
}
