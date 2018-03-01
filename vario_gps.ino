/*
Mixing of signal from GPS and Vario on same serial 

/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <phk@FreeBSD.ORG> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return Poul-Henning Kamp
 * ----------------------------------------------------------------------------
 
 
  (c) BARTHELEMY Stéphane 03/2018
  Base on  Vario by Benjamin PERRIN 2017 / Vari'Up
  Modifications by Ottaflodna commented within "Modif"
  Modif Sylk Digifly sentence and GPS

  Based on Arduino Vario by Jaros, 2012 and vario DFelix 2013

  $LK8EX1,pressure,altitude,vario,temperature,battery,*checksum

  Credits:
  (1) http://code.google.com/p/bmp085driver/                             //bmp085 library
  (2) http://mbed.org/users/tkreyche/notebook/bmp085-pressure-sensor/    //more about bmp085 and average filter
  (3) http://code.google.com/p/rogue-code/                               //helpfull tone library to make nice beeping without using delay

      MS5611 https://github.com/jarzebski/Arduino-MS5611.git

 
 */
#define MS5611_VARIO
#define SAMPLES_ARR   5 
#define HZ          (1000)
#define DELAY_VARIO (HZ /3 )  /*Every second */

 
// inckude
#include <SoftwareSerial.h>
#include <Wire.h>                      //i2c library
#ifdef MS5611_VARIO
#include "MS5611.h"
#else
#include "BMP085.h"                    //bmp085 library, download from url link (1)
#endif
#include <stdlib.h> 

#define SAMPLES_ARR   5 
#define HZ          (1000)
#define DELAY_VARIO (HZ /3 )  /*Every second */
 
#define SendLog(x) SendData(x)

/*prototypes*/
static long Averaging_Filter(long input);

/*Global Variables */
SoftwareSerial gps(12, 11); // RX | TX
#ifdef MS5611_VARIO
MS5611 ms5611;
#else
BMP085   bmp085 = BMP085();            //set up bmp085 sensor
#endif

unsigned long time_vario;

boolean CaptureGps = false ;   /*Gps tram always start by $ , pass to true when it is detected */
char GpsTram[100]  =  {0}; /*Save tram received fom GPS , max 82 characters */

long     Temperature = 0;
long     Pressure = 101325;
float    Altitude;
int      Battery_Vcc = 1100;             //variable to hold the value of Vcc from battery not used
const float p0 = 101325;              //Pressure at sea level (Pa)
unsigned long get_time1 = millis();
unsigned long get_time2 = millis();
int      my_temperature = 1;
char     altitude_arr[6];            //wee need this array to translate float to string
char     vario_arr[5];               //wee need this array to translate float to string
int      samples = 40;
int      maxsamples = 50;
float    alt[51];
float    tim[51];
static long k[SAMPLES_ARR];
long average_pressure; 
float tempo;
float vario = 0;
float N1 = 0;
float N2 = 0;
float N3 = 0;
float D1 = 0;
float D2 = 0;

void SendData (char * szData ){
  
  Serial.print (szData) ; /*Send data from GPS to Serial */
  Serial.print ("\r\n") ;  /*For Carriage return and Line Feed for end of line  , not saved on GpsData */
}


static long Averaging_Filter(long input) // moving average filter function
{
  long sum = 0;
  for (int i = 0; i < SAMPLES_ARR; i++)
  {
    k[i] = k[i + 1];
  }
  k[SAMPLES_ARR - 1] = input;
  for (int i = 0; i < SAMPLES_ARR; i++)
  {
    sum += k[i];
  }
  return ( sum / SAMPLES_ARR ) ;
}


void Getinfo(void) 
{
  tempo = millis();
  vario = 0;
  N1 = 0;
  N2 = 0;
  N3 = 0;
  D1 = 0;
  D2 = 0;
#ifdef MS5611_VARIO
  Pressure = ms5611.getPressure(); 
#else
  bmp085.calcTruePressure(&Pressure);                                   //get one sample from BMP085 in every loop
#endif
  average_pressure = Averaging_Filter(Pressure);                   //put it in filter and take average
  Altitude = (float)44330 * (1 - pow(((float)Pressure / p0), 0.190295)); //take new altitude in meters

  for (int cc = 1; cc <= maxsamples; cc++)                             //samples averaging and vario algorithm
  {
    alt[(cc - 1)] = alt[cc];
    tim[(cc - 1)] = tim[cc];
  };
  alt[maxsamples] = Altitude;
  tim[maxsamples] = tempo;
  float stime = tim[maxsamples - samples];
  for (int cc = (maxsamples - samples); cc < maxsamples; cc++)
  {
    N1 += (tim[cc] - stime) * alt[cc];
    N2 += (tim[cc] - stime);
    N3 += (alt[cc]);
    D1 += (tim[cc] - stime) * (tim[cc] - stime);
    D2 += (tim[cc] - stime);
  };

  vario = 1000 * ((samples * N1) - N2 * N3) / (samples * D1 - D2 * D2);

  if (millis() >= (get_time1 + 10000))    //every second get temperature
  {
#ifdef MS5611_VARIO
    Temperature =  (ms5611.getTemperature()) ;    
#else
    bmp085.getTemperature(&Temperature(); // get temperature in celsius from time to time, we have to divide that by 10 to get XY.Z
#endif 
    my_temperature = Temperature / 10;
    get_time1 = millis();

    Battery_Vcc = 1000 + 100 * (analogRead(A7) - 753) / 216; // "Modif: read A7 analog pin fed with half the Vin voltage...
    if (Battery_Vcc <= 1001)                         // Battery charge curve is what it is defined like that, lot of room for improvement ;-)"
    {
      Battery_Vcc = 1001;
    }
  }
}

void BuildVarioString (char * szVarioData)
{
  unsigned int checksum_end, ai, bi;                                               // Calculating checksum for data string
  
  sprintf (szVarioData,"$LK8EX1,%ld,%s,%s,%d,%d,",average_pressure,dtostrf(Altitude, 0, 0, altitude_arr),dtostrf((vario * 100), 0, 0, vario_arr),my_temperature,Battery_Vcc);
  for (checksum_end = 0, ai = 0; ai < strlen(szVarioData); ai++)
    {
      bi = (unsigned char)szVarioData[ai];
      checksum_end ^= bi;
    }
  sprintf (szVarioData,"%s*%d",szVarioData,checksum_end); 
}

void setup() {
  Serial.begin(9600);
  gps.begin(9600);
#ifdef MS5611_VARIO
 ms5611.begin(); 
#else
  bmp085.init(MODE_ULTRA_HIGHRES, p0, false);
#endif
  time_vario =millis();
  
}

void loop() {
  
    /*Check Data from GPS*/
    while (gps.available() > 0) {
      char ch = '\0'; 
      ch = gps.read();
      if (ch == '$') { /*Start record of GPS tram if started by $ */
        CaptureGps = true; 
        strcpy (GpsTram,"") ; 
      }
      if ((ch == 0x0A) || (ch == 0x0D )) {  /*Stop record of GPS if CRLF detected , send data to Serial */
        CaptureGps = false ; 
        SendData (GpsTram);
      }
      if (CaptureGps == true ) { /*Convert char received to string */
        char szTmp[3] ; 
        memset ( szTmp , '\0', 3 );     
        szTmp [0] = ch ; 
        strcat (GpsTram, szTmp) ; 
        }
  }
  /*get Sensor info*/
  Getinfo();
  
  /*Build and send Vario string every Second */
  if (millis () > (time_vario + DELAY_VARIO))  {
    char SzVario [256] = {0} ;
    BuildVarioString (SzVario) ;
    SendData (SzVario) ; 
    time_vario = millis();
  }
}
