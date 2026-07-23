•	KODINGAN AMBIL DATA UNTUK TRAINING ANFIS
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_PH_ADS.h"
#include "DFRobot_EC_ADS.h"
#include "DFRobot_DO_ADS.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_wpa2.h"
#define WIFI_SSID "YOUR_WIFI"
#define EAP_ID "YOUR_USERNAME"
#define EAP_PW "YOUR_PASSWORD"
#define MQTT_TOPIC "TA/Sinta"
#define RELAY_PH_UP 18
#define RELAY_PH_DOWN 23
#define RELAY_NUT_A 19
#define RELAY_NUT_B 5
#define PIN_KIPAS 33
#define PIN_AERATOR 32
const float SP_SUHU=27.5f, SP_PH=6.0f, SP_EC=2.0f, SP_DO=6.0f;
const unsigned long CTRL_INT=20000, MQTT_INT=5000;
const unsigned long PH_COOLDOWN=90000, EC_COOLDOWN=20000;
WiFiClient espC; PubSubClient mq(espC);
Adafruit_ADS1115 ads;
OneWire ow(4); DallasTemperature ds(&ow);
DFRobot_PH_ADS ph; DFRobot_EC_ADS ec; DFRobot_DO_ADS doS;
float T=25,pH=7,EC=0,DO=0,phv=0,ecv=0,dov=0;
float e_T=0,e_ph=0,e_ec=0,e_do=0,de_T=0,de_ph=0,de_ec=0,de_do=0,pe_T=0,pe_ph=0,pe_ec=0,pe_do=0;
uint8_t pwmK=0, pwmA=0; uint32_t dlyPH=0, dlyEC=0;
enum {IDLE,PH_ON,PH_SET,EC_ON,EC_SET} actPh=IDLE;
int actRelay=-1; unsigned long actT0=0, actDur=0, phCoolT=0, ecCoolT=0;
float mapf(float x,float i0,float i1,float o0,float o1){
  x=constrain(x,min(i0,i1),max(i0,i1));
  return o0+(o1-o0)*(x-i0)/(i1-i0);
}
uint8_t ctrlKipas(float e){ if(e>-0.10f) return 0; return (uint8_t)constrain(mapf(e,-0.10f,-8.0f,100,255),100,255); }
uint8_t ctrlAerator(float e){ if(e<=0.05f) return 0; return (uint8_t)constrain(mapf(e,0.05f,1.0f,60,255),0,255); }
uint32_t ctrlPH(float e){ float ae=fabs(e); if(ae<0.05f) return 0; return (uint32_t)constrain(mapf(ae,0.05f,2.0f,2000,12000),2000,12000); }
uint32_t ctrlEC(float e){ if(e<0.05f) return 0; return (uint32_t)constrain(mapf(e,0.05f,2.0f,3000,40000),3000,40000); }
void connectWifi(){
  WiFi.disconnect(true); WiFi.mode(WIFI_STA);
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)EAP_ID,strlen(EAP_ID));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t*)EAP_ID,strlen(EAP_ID));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t*)EAP_PW,strlen(EAP_PW));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(WIFI_SSID);
}
void readSensors(){
  ds.requestTemperatures();
  float t=ds.getTempCByIndex(0);
  if(t!=DEVICE_DISCONNECTED_C && t>-10 && t<60) T=t;
  long s0=0,s1=0,s2=0;
  for(int i=0;i<20;i++){ s0+=ads.readADC_SingleEnded(0); s1+=ads.readADC_SingleEnded(1); s2+=ads.readADC_SingleEnded(2); delay(5); }
  phv=max(s0/20*0.125f,0.0f); ecv=max(s1/20*0.125f,0.0f); dov=max(s2/20*0.125f,0.0f);
  pH=ph.readPH(phv,T); EC=ec.readEC(ecv,T); DO=doS.readDO(dov,T);
}
void runActuators(){
  ledcWrite(PIN_KIPAS,pwmK); ledcWrite(PIN_AERATOR,pwmA);
  if(actPh!=IDLE) return;
  if(dlyEC>0){
    digitalWrite(RELAY_NUT_A,LOW); digitalWrite(RELAY_NUT_B,LOW);
    actDur=dlyEC; actT0=millis(); actPh=EC_ON;
  } else if(dlyPH>0){
    if(e_ph<0){ digitalWrite(RELAY_PH_DOWN,LOW); actRelay=RELAY_PH_DOWN; }
    else{ digitalWrite(RELAY_PH_UP,HIGH); actRelay=RELAY_PH_UP; }
    actDur=dlyPH; actT0=millis(); actPh=PH_ON;
  }
}
void handleActuator(){
  if(actPh==IDLE) return;
  unsigned long el=millis()-actT0;
  if(actPh==PH_ON && el>=actDur){
    if(actRelay==RELAY_PH_UP) digitalWrite(RELAY_PH_UP,LOW); else digitalWrite(RELAY_PH_DOWN,HIGH);
    actT0=millis(); actDur=3000; actPh=PH_SET;
  } else if(actPh==PH_SET && el>=actDur){ actPh=IDLE; phCoolT=millis()+PH_COOLDOWN; }
  else if(actPh==EC_ON && el>=actDur){
    digitalWrite(RELAY_NUT_A,HIGH); digitalWrite(RELAY_NUT_B,HIGH);
    actT0=millis(); actDur=3000; actPh=EC_SET;
  } else if(actPh==EC_SET && el>=actDur){ actPh=IDLE; ecCoolT=millis()+EC_COOLDOWN; }
}
void setup(){
  Wire.begin(21,22); ads.begin(0x48); ads.setGain(GAIN_ONE);
  ds.begin(); ds.setResolution(12);
  ph.begin(); ec.begin(); doS.begin();
  ledcAttach(PIN_KIPAS,5000,8); ledcAttach(PIN_AERATOR,5000,8);
  int noPins[]={RELAY_PH_DOWN,RELAY_NUT_A,RELAY_NUT_B};
  for(int i=0;i<3;i++){ pinMode(noPins[i],OUTPUT); digitalWrite(noPins[i],HIGH); }
  pinMode(RELAY_PH_UP,OUTPUT); digitalWrite(RELAY_PH_UP,LOW);
  connectWifi();
  mq.setServer("10.124.38.31",1883);
  mq.setBufferSize(512);
}
void loop(){
  static unsigned long lc=0, lm=0;
  unsigned long now=millis();
  if(WiFi.status()!=WL_CONNECTED) connectWifi();
  if(!mq.connected()) mq.connect("TA_SINTA_NFT_ESP32");
  mq.loop();
  handleActuator();
  readSensors();
  if(now-lc>=CTRL_INT){
    lc=now;
    e_T=SP_SUHU-T; e_ph=SP_PH-pH; e_ec=SP_EC-EC; e_do=SP_DO-DO;
    de_T=e_T-pe_T; de_ph=e_ph-pe_ph; de_ec=e_ec-pe_ec; de_do=e_do-pe_do;
    pe_T=e_T; pe_ph=e_ph; pe_ec=e_ec; pe_do=e_do;
    pwmK=ctrlKipas(e_T); pwmA=ctrlAerator(e_do);
    bool ecOk = (now-ecCoolT)>=EC_COOLDOWN || ecCoolT==0;
    bool phOk = (now-phCoolT)>=PH_COOLDOWN || phCoolT==0;
    uint32_t ecCand = ecOk ? ctrlEC(e_ec) : 0;
    if(ecCand>0){ dlyEC=ecCand; dlyPH=0; }
    else{ dlyEC=0; dlyPH=phOk ? ctrlPH(e_ph) : 0; }
    runActuators();
  }
  if(now-lm>=MQTT_INT){
    lm=now;
    char buf[300];
snprintf(buf,sizeof(buf),"{\"suhu\":%.2f,\"ph\":%.2f,\"ec\":%.3f,\"do\":%.2f,\"pwm_kipas\":%u,\"pwm_aerator\":%u,\"delay_ph_ms\":%lu,\"delay_ec_ms\":%lu,\"e_suhu\":%.3f,\"e_ph\":%.3f,\"e_ec\":%.3f,\"e_do\":%.3f}",
      T,pH,EC,DO,pwmK,pwmA,dlyPH,dlyEC,e_T,e_ph,e_ec,e_do);
    mq.publish(MQTT_TOPIC,buf,true);
  }
}
