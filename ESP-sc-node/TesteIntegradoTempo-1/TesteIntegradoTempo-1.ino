#include "DHT.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
//------Cliente LoRa-----------------------------
#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "SSD1306.h"
#include <CayenneLPP.h>


static const PROGMEM u1_t NWKSKEY[16] = { 0x5B, 0x0A, 0xB1, 0x5B, 0x0C, 0x5D, 0x46, 0xC1, 0x93, 0x93, 0x76, 0xE6, 0x74, 0xA9, 0xE7, 0xB9 };
static const u1_t PROGMEM APPSKEY[16] = { 0xCF, 0xD7, 0x13, 0xD7, 0xA8, 0xCE, 0x89, 0xDB, 0xC4, 0x66, 0x30, 0x05, 0x70, 0xB2, 0x30, 0x96 };
static const u4_t DEVADDR = { 0x26031845 };
static osjob_t sendjob;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
const unsigned TX_INTERVAL = 0; 

unsigned long tempo = 0; // variável para receber valores do millis()
int temporizador = 0; // variável para indicar quando o temporizador está operando

/*
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};
*/
const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    //If DIO2 is not connected use:
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ LMIC_UNUSED_PIN} 
    //If DIO2 is connected use:
    //.dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32} 
};

#define SERIAL1_RXPIN 12
#define SERIAL1_TXPIN 13
//HardwareSerial gpsSerial(2);
CayenneLPP lpp(15); 
SSD1306 display(0x3c, 4, 15); 


//---------------------------------------------------------------


Adafruit_BMP085 bmp;

//definições para o sensor DHT
#define DHTPIN 17     // define a porta 8 que o DHT está conectado
#define DHTTYPE DHT22   // define o sensor como DHT22
DHT dht(DHTPIN, DHTTYPE); //chama a função do dht passando porta e tipo de sensor

#define PIN 17

float hum =0;
float temp =0 ;
float BarometerPress = 0;
int contador = 0;


//Constants definitions anem.
const float pi = 3.14159265;           // Numero pi
//const float pi = 1.57;           // Numero pi
int period = 5000;               // Tempo de medida(miliseconds) 5000
int radius = 147;      // Raio do anemometro(mm)

// Variable definitions

unsigned int counter = 0; // magnet counter for sensor
unsigned int RPM = 0;          // Revolutions per minute
float speedwind = 0;         // Wind speed (Km/h)

//Const def Pluviômetro

#define PIN_VL 2  // entrada D6 e + , pluviometro * 
float REEDCOUNT = 0;  //This is the variable that hold the count of switching
float rain = 0;
int vl = 0; 
int pin=39;  // A0 entrada do biruta *
float valor =0;
float Winddir =0;


void setup()
{    
  Serial.begin(115200);
  Serial.printf("Starting...\r\n");
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH);
  display.init();
  display.setFont(ArialMT_Plain_10);  

  dht.begin();//inicia sensor DHT

  if (isnan(hum) || isnan(temp))
  { 
    Serial.println("Falha ao ler dados do sensor DHT !!!");
    return;
  }
  //bmp.begin(); //Start BMP sensor
  //if (!bmp.begin()) 
  //{
  //  Serial.println("Sensor BMP nao encontrado !!");
  //  while (1) {}
 // }

 // Pluviômetro
  pinMode (15,OUTPUT); // saida para reset pluviometro *
  pinMode (2,INPUT); // entrada pluvio *
  digitalWrite(15, HIGH);  // ativa pluviom

//   Anemômetro 
  pinMode(25, INPUT);  // Entrada anemometro *
 digitalWrite(25, HIGH);  //internall pull-up active 

  os_init();
  LMIC_reset();
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  LMIC_selectSubBand(0);
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF7;
  LMIC_setDrTxpow(DR_SF7,14);
  do_send(&sendjob);

}

void windvelocity(){
  speedwind = 0;
  counter = 0;  
  attachInterrupt(25, addcount, RISING);
  unsigned long millis();       
  long startTime = millis();
 while(millis() < startTime + period)
  //while(millis() < 10)
 {
  }
}


void RPMcalc(){
  RPM=((counter)*60)/(period/1000);  // Calculate revolutions per minute (RPM)
}

void SpeedWind(){
  speedwind = (((4 * pi * radius * RPM)/60) / 1000)*3.6;  // Calculate wind speed on km/h   * RPM)/60)      1000/*3.6
 
}

void addcount(){
  counter++;
} 


void loop()
{  
    outros();
    
}

void outros(){


valor = analogRead(pin)* (5.0 / 1023.0); 
 
Winddir = valor;
if (valor <= 0.15) {
Winddir = 315;
}
else if (valor <= 0.31) { 
Winddir = 270;
}
else if (valor <= 0.46) { 
Winddir = 225;
}
else if (valor <= 0.70) { 
Winddir = 180;
}
else if (valor <= 0.95) { 
Winddir = 135;
}
else if (valor <= 1.52) { 
Winddir = 90;
}
else if (valor <= 2.42) {  
Winddir = 45;
}
else {  
Winddir = 000;
}


//Ler Pluviometro


int vl = digitalRead(PIN_VL);  // Inicia a leitura do pluviom
    if ( vl == HIGH ) {
     REEDCOUNT = REEDCOUNT + 1;
     digitalWrite(39,LOW ); // reseta pluviom   
    }
   rain = REEDCOUNT / 4;  // Cada pulso 0.25 mm!
   digitalWrite(39, HIGH);  // ativa pluviom e finaliza leitura

   hum = dht.readHumidity();
   temp = dht.readTemperature();
  // BarometerPress =  bmp.readPressure();


 windvelocity();
 RPMcalc();
 SpeedWind();
/*
  //exibe dados do sensor no serial
    Serial.println("Umidade = ");
    Serial.print(hum);
    Serial.print(" %  ");
    Serial.print("Temperatura = ");
    Serial.print(temp);
    Serial.println(" Celsius  ");
   //Serial.print("--Pressao = ");
    //Serial.print(BarometerPress);
    Serial.print("Precipitacao "); 
     Serial.print(rain);
    //Serial.print(REEDCOUNT*0.25); 
    Serial.println(" mm  ");
    Serial.print("--Direcao ");
    Serial.print(Winddir);
    Serial.print(" graus ");   
    Serial.print("--Veloc ");
  Serial.print(speedwind);
    Serial.println(" Km/h ");
  */  
    Serial.print("--Veloc ");
  Serial.print(speedwind);
    Serial.println(" Km/h ");
   os_runloop_once();
  
}

//------Cliente LoRa------------------------------------------

void onEvent (ev_t ev) 
{
  //Serial.print(os_getTime());
  //Serial.print(": ");
  switch(ev) 
  {
    case EV_TXCOMPLETE:
      Serial.printf("EV_TXCOMPLETE (includes waiting for RX windows)\r\n");
      if(contador  <= 18 && contador >= 9){
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send2);
      contador++;
      Serial.println(contador);
      }
      else{
        if(contador <= 9){
          Serial.println(contador);
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
        contador++;
        }
        else{ 
          if (contador >= 18) {
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send2);
             contador = 0;
           }
          }
        }
  
      break;  
    case EV_RXCOMPLETE:
      if (LMIC.dataLen)
      {
        Serial.printf("Received %d bytes\n", LMIC.dataLen);
      }
      break;
    default:
      Serial.printf("Unknown event\r\n");
      break;
  }
}

void do_send(osjob_t* j)
{
  if (LMIC.opmode & OP_TXRXPEND) 
  {
    Serial.printf("OP_TXRXPEND, not sending\r\n");
  } 
  else
  if (!(LMIC.opmode & OP_TXRXPEND)) 
  {  
    lpp.reset();
    lpp.addTemperature(0, temp);
    lpp.addRelativeHumidity(1, hum);
    
    Serial.println("temp");
  
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
         
    Serial.printf("Packet queued\r\n");
  }
}

void do_send2(osjob_t* j)
{
  if (LMIC.opmode & OP_TXRXPEND) 
  {
    Serial.printf("OP_TXRXPEND, not sending\r\n");
  } 
  else
  if (!(LMIC.opmode & OP_TXRXPEND)) 
  {  
    lpp.reset();
    Serial.println("analogic");
    lpp.addAnalogInput(2,rain);
    lpp.addAnalogInput(3,speedwind);
    Serial.println(speedwind);
    lpp.addAnalogInput(4,Winddir);  
  
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
         
    Serial.printf("Packet queued\r\n");
  }
}

//---------------------------------------------------------------



