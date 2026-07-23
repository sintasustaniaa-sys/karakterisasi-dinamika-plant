•	KODINGAN KONTROL ANFIS Tipe-1
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
#include <math.h>
#include "ANFIS_T1_params_FIKSSS.h"
#define WIFI_SSID "YOUR_WIFI"
#define EAP_ID "YOUR_USERNAME"
#define EAP_PW "YOUR_PASSWORD"
#define MQTT_TOPIC "TA/Sinta/data"
#define RELAY_PH_UP 18
#define RELAY_PH_DOWN 23
#define RELAY_NUT_A 19
#define RELAY_NUT_B 5
#define PIN_KIPAS 33
#define PIN_AERATOR 32
#define MAX_N_MF 7
const float SP_SUHU=27.5f, SP_PH=6.0f, SP_EC=2.0f, SP_DO=6.0f;
const unsigned long CTRL_INT=20000, SENS_INT=2000, MQTT_INT=5000, PH_SETTLE=3000, PH_COOLDOWN=90000;
WiFiClient espC; PubSubClient mq(espC);
Adafruit_ADS1115 ads;
OneWire ow(4); DallasTemperature ds(&ow);
DFRobot_PH_ADS ph; DFRobot_EC_ADS ec; DFRobot_DO_ADS doS;
float T=25,pH=7,EC=0,DO=0,phv=0,ecv=0,dov=0;
float e_T=0,e_ph=0,e_ec=0,e_do=0,de_T=0,de_ph=0,de_ec=0,de_do=0,pe_T=0,pe_ph=0,pe_ec=0,pe_do=0;
uint8_t pwmK=0, pwmA=0; uint32_t dlyPH=0, dlyEC=0;
enum {IDLE,EC_ON,EC_SET,PH_ON,PH_SET} actPh=IDLE;
uint8_t actRelay=0; unsigned long actT0=0, actDur=0, cooldownT=0;
float gaussMF(float x,float c,float s){ s=fabsf(s)+1e-6f; float z=(x-c)/s; return expf(-0.5f*z*z); }
float infer(float x1,float x2,const float* c1,const float* c2,const float* s1,const float* s2,
            const float* p,float omin,float omax,uint8_t n){
  float m1[MAX_N_MF],m2[MAX_N_MF],ws=0,num=0;
  for(uint8_t i=0;i<n;i++){ m1[i]=gaussMF(x1,c1[i],s1[i]); m2[i]=gaussMF(x2,c2[i],s2[i]); }
  for(uint8_t i=0;i<n;i++) for(uint8_t j=0;j<n;j++){ float w=m1[i]*m2[j]; ws+=w; num+=w*p[i*n+j]; }
  return omin+(num/(ws+1e-10f))*(omax-omin);
}
void computeANFIS(){
  float x1,x2;
  x1=constrain(e_T,-10,10); x2=constrain(de_T,-3,3);
  pwmK=(e_T<-0.10f)?(uint8_t)constrain(infer(x1,x2,T1_Suhu_c1,T1_Suhu_c2,T1_Suhu_s1,T1_Suhu_s2,T1_Suhu_p,T1_Suhu_out_min,T1_Suhu_out_max,T1_Suhu_N_MF),0,255):0;

  x1=constrain(e_ec,-2,2); x2=constrain(de_ec,-0.5f,0.5f);
dlyEC=(e_ec>0.05f)?(uint32_t)constrain(infer(x1,x2,T1_EC_c1,T1_EC_c2,T1_EC_s1,T1_EC_s2,T1_EC_p,T1_EC_out_min,T1_EC_out_max,T1_EC_N_MF),0,40000):0;
  x1=constrain(e_ph,-2,2); x2=constrain(de_ph,-0.5f,0.5f);
  dlyPH=(uint32_t)constrain(
    e_ph>0.05f  ? infer(x1,x2,T1_pH_Up_c1,T1_pH_Up_c2,T1_pH_Up_s1,T1_pH_Up_s2,T1_pH_Up_p,T1_pH_Up_out_min,T1_pH_Up_out_max,T1_pH_Up_N_MF) :
    e_ph<-0.05f ? infer(x1,x2,T1_pH_Down_c1,T1_pH_Down_c2,T1_pH_Down_s1,T1_pH_Down_s2,T1_pH_Down_p,T1_pH_Down_out_min,T1_pH_Down_out_max,T1_pH_Down_N_MF) : 0,
    0,22000);
x1=constrain(e_do,-3,2); x2=constrain(de_do,-0.5f,0.5f);
pwmA=(fabsf(e_do)>0.05f)?(uint8_t)constrain(infer(x1,x2,T1_DO_c1,T1_DO_c2,T1_DO_s1,T1_DO_s2,T1_DO_p,T1_DO_out_min,T1_DO_out_max,T1_DO_N_MF),0,255):0;
}
void connectWifi(){
  WiFi.disconnect(true); WiFi.mode(WIFI_STA);
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)EAP_ID,strlen(EAP_ID));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t*)EAP_ID,strlen(EAP_ID));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t*)EAP_PW,strlen(EAP_PW));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(WIFI_SSID);
}
void readSensors(){
  ds.requestTemperatures(); delay(100);
  float t=ds.getTempCByIndex(0);
  if(t!=DEVICE_DISCONNECTED_C && t>-10 && t<60) T=t;
  long s0=0,s1=0,s2=0;
  for(int i=0;i<10;i++){ s0+=ads.readADC_SingleEnded(0); s1+=ads.readADC_SingleEnded(1); s2+=ads.readADC_SingleEnded(2); delay(5); }
  phv=max(s0/10*0.125f,0.0f); ecv=max(s1/10*0.125f,0.0f); dov=max(s2/10*0.125f,0.0f);
  pH=ph.readPH(phv,T); EC=ec.readEC(ecv,T); DO=doS.readDO(dov,T);
}
void runActuators(){
  ledcWrite(PIN_KIPAS,pwmK); ledcWrite(PIN_AERATOR,pwmA);
  if(actPh!=IDLE) return;
  if(dlyEC>0){
    digitalWrite(RELAY_NUT_A,LOW); digitalWrite(RELAY_NUT_B,LOW);
    actDur=dlyEC; actT0=millis(); actPh=EC_ON;
  } else if(dlyPH>0 && millis()>=cooldownT){
    if(e_ph<0){ digitalWrite(RELAY_PH_DOWN,LOW); actRelay=RELAY_PH_DOWN; }
    else{ digitalWrite(RELAY_PH_UP,HIGH); actRelay=RELAY_PH_UP; }
    actDur=dlyPH; actT0=millis(); actPh=PH_ON;
  }
}
void handleActuator(){
  if(actPh==IDLE) return;
  unsigned long el=millis()-actT0;
  if(actPh==EC_ON && el>=actDur){
    digitalWrite(RELAY_NUT_A,HIGH); digitalWrite(RELAY_NUT_B,HIGH);
    actT0=millis(); actDur=PH_SETTLE; actPh=EC_SET;
  } else if(actPh==EC_SET && el>=actDur){ actPh=IDLE; }
  else if(actPh==PH_ON && el>=actDur){
    if(actRelay==RELAY_PH_UP) digitalWrite(RELAY_PH_UP,LOW); else digitalWrite(actRelay,HIGH);
    actT0=millis(); actDur=PH_SETTLE; actPh=PH_SET;
  } else if(actPh==PH_SET && el>=actDur){ actPh=IDLE; cooldownT=millis()+PH_COOLDOWN; }
}
void setup(){
  Wire.begin(21,22); ads.begin(0x48); ads.setGain(GAIN_ONE);
  ds.begin(); ds.setResolution(9);
  ph.begin(); ec.begin(); doS.begin();
  ledcAttach(PIN_KIPAS,5000,8); ledcAttach(PIN_AERATOR,5000,8);
  int noPins[]={RELAY_PH_DOWN,RELAY_NUT_A,RELAY_NUT_B};
  for(int i=0;i<3;i++){ pinMode(noPins[i],OUTPUT); digitalWrite(noPins[i],HIGH); }
  pinMode(RELAY_PH_UP,OUTPUT); digitalWrite(RELAY_PH_UP,LOW);
  connectWifi();
  mq.setServer("10.124.38.31",1883);
}
void loop(){
  static unsigned long lc=0,ls=0,lm=0;
  unsigned long now=millis();
  if(WiFi.status()!=WL_CONNECTED) connectWifi();
  if(!mq.connected()) mq.connect("TA_SINTA_T1");
  mq.loop();
  handleActuator();
  if(now-ls>=SENS_INT){ ls=now; readSensors(); }
  if(now-lc>=CTRL_INT){
    lc=now;
    e_T=SP_SUHU-T; e_ph=SP_PH-pH; e_ec=SP_EC-EC; e_do=SP_DO-DO;
    de_T=e_T-pe_T; de_ph=e_ph-pe_ph; de_ec=e_ec-pe_ec; de_do=e_do-pe_do;
    pe_T=e_T; pe_ph=e_ph; pe_ec=e_ec; pe_do=e_do;
    computeANFIS(); runActuators();
  }
  if(now-lm>=MQTT_INT){
    lm=now;
    char buf[220];
snprintf(buf,sizeof(buf),"{\"suhu\":%.2f,\"ph\":%.3f,\"ec\":%.3f,\"do\":%.2f,\"e_suhu\":%.3f,\"e_ph\":%.3f,\"e_ec\":%.3f,\"e_do\":%.3f,\"pwm_kipas\":%u,\"pwm_aer\":%u,\"dly_ph\":%lu,\"dly_ec\":%lu}",
      T,pH,EC,DO,e_T,e_ph,e_ec,e_do,pwmK,pwmA,dlyPH,dlyEC);
    mq.publish(MQTT_TOPIC,buf);
  }
}
