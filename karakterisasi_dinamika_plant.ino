#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_PH_ADS.h"
#include "DFRobot_EC_ADS.h"
#include "DFRobot_DO_ADS.h"
#include "esp_wpa2.h"
#define WIFI_SSID "YOUR_WIFI"
#define EAP_ID "YOUR_USERNAME"
#define EAP_PW "YOUR_PASSWORD"
#define MQTT_TOPIC "TA/Sinta"
WiFiClient espC; PubSubClient mq(espC);
Adafruit_ADS1115 ads;
OneWire ow(4); DallasTemperature ds(&ow);
DFRobot_PH_ADS ph; DFRobot_EC_ADS ec; DFRobot_DO_ADS doS;
float T=25, phv=0, ecv=0, dov=0, pH=7, EC=0, DO=0;
const unsigned long READ_INT=2000;
void connectWifi(){
  WiFi.disconnect(true); WiFi.mode(WIFI_STA);
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)EAP_ID,strlen(EAP_ID));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t*)EAP_ID,strlen(EAP_ID));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t*)EAP_PW,strlen(EAP_PW));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(WIFI_SSID);
}
float readADS_mV(uint8_t ch){
  long sum=0; for(int i=0;i<20;i++){ sum+=ads.readADC_SingleEnded(ch); delay(5); }
  return max((sum/20)*0.125f, 0.0f);
}
void setup(){
  Wire.begin(21,22); ads.begin(0x48); ads.setGain(GAIN_ONE);
  ds.begin(); ds.setResolution(12);
  ph.begin(); ec.begin(); doS.begin();
  connectWifi();
  mq.setServer("10.124.38.31",1883);
  mq.setKeepAlive(60); mq.setSocketTimeout(10);
}
void loop(){
  static unsigned long lastRead=0;
  if(WiFi.status()!=WL_CONNECTED) connectWifi();
  if(!mq.connected()) mq.connect("TA_SINTA_NFT_ESP32");
  mq.loop();
  if(millis()-lastRead<READ_INT) return;
  lastRead=millis();
  ds.requestTemperatures();
  float t=ds.getTempCByIndex(0);
  if(t!=DEVICE_DISCONNECTED_C && t>-10 && t<60) T=t;
  phv=readADS_mV(0); ecv=readADS_mV(1); dov=readADS_mV(2);
  pH=ph.readPH(phv,T); EC=ec.readEC(ecv,T); DO=doS.readDO(dov,T);
  if(WiFi.status()==WL_CONNECTED && mq.connected()){
    char buf[220];  snprintf(buf,sizeof(buf),"{\"suhu\":%.2f,\"ph\":%.2f,\"ph_mv\":%.2f,\"ec\":%.3f,\"ec_mv\":%.2f,\"donft\":%.2f,\"do_mv\":%.2f}",
      T,pH,phv,EC,ecv,DO,dov);
    mq.publish(MQTT_TOPIC,buf,true);
  }
  ph.calibration(phv,T);
  ec.calibration(ecv,T);
  doS.calibration(dov,T);
}

