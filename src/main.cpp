#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#define LED 13
#include "MAX30105.h"
#include <FirebaseESP32.h>
#define debug Serial

// Define signal parameters
//int Sampling_Time = 20;  //(20ms-->50 samples/sec--> 25 samples/sec for ir & 25 s/sec for red) fs=50 (#sampleRate = 400; & byte sampleAverage = 8 ;)
int samp_freq=25; //for each led
const int Num_Samples = 125;  //it stores 5 sec (In fact each led is sampled every 40ms)
int Peak_Threshold_Factor = 80;
int Minimum_Peak_Separation = 10;  // 10*40=400 ms
int Moving_Average_Num = 10;
const int points=3; 
const int points_pr=5;
int Index, i, j, k;
float Pulse_Rate_next=0, Pulse_Rate_previous=70, SpO2_next=0, SpO2_previous=98;
float Peak, ADC_Range_ir, ADC_Range_r, Peak_Threshold, Sum_Points_ir, Sum_Points_r, Num_Points;
float AC_r, DC_r, AC_ir, DC_ir, Minima_ir, Peak_Magnitude_ir, Minima_r, Peak_Magnitude_r, Mean;
float ADC_Value_r[Num_Samples];       //analog to digital converter red LED . I store the values of red LED here
float ADC_Value_ir[Num_Samples];      // infra red LED
float PR[points_pr];
float Peaks_ir[points], Peaks_r[points], Min_ir[points], Min_r[points];
int HR;
int flag=1;

int Sampling_Time = 40;  //(40ms-->25 samples/sec--> fs=25Hz
const int Num_SamplesA = 1000;   //i just put a random number there, the size of the vectors
int ii=0, jj=0, kk=0;   //i probably dont need a flag
float ADC_Value_x[Num_SamplesA];   
float ADC_Value_y[Num_SamplesA];  
float ADC_Value_z[Num_SamplesA];  
float distance[Num_SamplesA];

void ReadSamples();
void ADC_range();
float Find_Minima(int p1, int p2, float *x);
float Find_Peak(int p1, int p2, float *x);
void ZeroData(int len, float *x);
void FilterData();
void ComputeHeartRate();

void displaySensorDetails();
void displayDataRate();
void displayRange();
void setup();
void loop();

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
MAX30105 Sensor; 


void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? "); 
      break;
  }  
  Serial.println(" Hz");  
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}

void ReadSamples(){
  for (int i=0; i < Num_Samples; i++){
        ADC_Value_ir[i] = float(Sensor.getIR());
        ADC_Value_r[i] = float(Sensor.getRed());
        if (ADC_Value_ir[i]<100000){     //then u know that there isnt a finger in place
          flag=0;
        }
        Sensor.nextSample();
        
  }  
}

float Find_Minima(int p1, int p2, float *x){     //*x is a pointer that shows the 0 element of the array x
  float Minima = 170000;    
  for (int m = p1; m < p2; m++){ //p1 is peak 1 
      if(Minima > x[m]){
        Minima = x[m];
      }
  }
  return Minima;
}

float Find_Peak(int p1, int p2, float *x){
  float Peak_Magnitude = 0;
  for (int m = p1; m < p2; m++){
      if(Peak_Magnitude < x[m]){
        Peak_Magnitude = x[m];
     }
  }
  return Peak_Magnitude;
}

void ZeroData(int len, float *x){
  for (int i = 0; i < len; i++){
     x[i] = 0;
  }
}

void FilterData(){
  Num_Points = 2*Moving_Average_Num+1;  //21-point moving average filter
  for (i = Moving_Average_Num; i < Num_Samples-Moving_Average_Num; i++){
    Sum_Points_ir = 0;
    Sum_Points_r = 0;
    for(k =0; k < Num_Points; k++){   
      Sum_Points_ir = Sum_Points_ir + ADC_Value_ir[i-Moving_Average_Num+k]; 
      Sum_Points_r = Sum_Points_r + ADC_Value_r[i-Moving_Average_Num+k]; 
    }    
    ADC_Value_ir[i] = Sum_Points_ir/Num_Points; 
    ADC_Value_r[i] = Sum_Points_r/Num_Points; 
  } 
  
  Peak_Magnitude_ir = Find_Peak(Moving_Average_Num, Num_Samples-Moving_Average_Num, ADC_Value_ir);
  Minima_ir = Find_Minima(Moving_Average_Num, Num_Samples-Moving_Average_Num, ADC_Value_ir);
  ADC_Range_ir = Peak_Magnitude_ir - Minima_ir;  

  Peak_Magnitude_r = Find_Peak(Moving_Average_Num, Num_Samples-Moving_Average_Num, ADC_Value_r);
  Minima_r = Find_Minima(Moving_Average_Num,Num_Samples-Moving_Average_Num, ADC_Value_r);
  ADC_Range_r = Peak_Magnitude_r - Minima_r;  
 
}

void ComputeHeartRate(){   //here i print the HR
  
  Peak_Threshold = Peak_Magnitude_ir*Peak_Threshold_Factor;   //Peak_Threshold=80% of the maxima of all the samples
  Peak_Threshold = Peak_Threshold/100;
  
  // Now detect successive peaks 
  ZeroData(points_pr, PR);
  Peak = 0;
  Index = 0;
  int p=0;

  for (j = Moving_Average_Num; j < Num_Samples-Moving_Average_Num; j++){
      // Find first peak
      if(ADC_Value_ir[j] >= ADC_Value_ir[j-1] && ADC_Value_ir[j] > ADC_Value_ir[j+1] && 
         ADC_Value_ir[j] > Peak_Threshold && Peak == 0){
           Peak = ADC_Value_ir[j];
           Index = j; 
      }
      // Search for next peak which is at least 10 sample time far
      if(Peak > 0 && j > (Index+Minimum_Peak_Separation)){
         if(ADC_Value_ir[j] >= ADC_Value_ir[j-1] && ADC_Value_ir[j] > ADC_Value_ir[j+1] && 
         ADC_Value_ir[j] > Peak_Threshold){
            float d=j-Index;
            float pulse=(float)samp_freq*60/d; //bpm for each PEAK interval
            PR[p++]=pulse; 
            p %= points_pr; //Wrap variable
            Peak = ADC_Value_ir[j];
            Index = j;
         } 
      }
 } 

 float sum=0;
 int c=0;
 for (int i=0; i< points_pr; i++){
       if(PR[i]!=0){
         sum=sum+PR[i];
         c=c+1;
       }
 }
 
 Pulse_Rate_next=sum/c;
 
 if(Pulse_Rate_next > 0 && Pulse_Rate_next < 200){
    if (Pulse_Rate_next-Pulse_Rate_previous>5){
      Pulse_Rate_next=Pulse_Rate_previous+4;
    }
    if (Pulse_Rate_next-Pulse_Rate_previous<-5){
      Pulse_Rate_next=Pulse_Rate_previous-4;
    }
    Pulse_Rate_previous=Pulse_Rate_next;
 }else{
    Pulse_Rate_next=Pulse_Rate_previous;
 }

 HR= int(round(Pulse_Rate_next));
 Serial.print("HR: ");
 Serial.print(HR);
}


void setup() {
  // put your setup code here, to run once:
  #ifndef ESP8266
  while (!Serial); 
#endif
  Serial.begin(115200);   //baud rate. to display in serial monitor
  Serial.println("Accelerometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  // accel.setRange(ADXL345_RANGE_2_G);
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate();
  displayRange();
  Serial.println("");



  //now about the HR MAX30105
  // Initialize sensor
  if (!Sensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  Sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
 
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t event; 
  accel.getEvent(&event);
  ADC_Value_x[ii] = float(event.acceleration.x);
  ADC_Value_y[ii] = float(event.acceleration.y);
  ADC_Value_z[ii] = float(event.acceleration.z);
  Serial.print("X: "); Serial.print(ADC_Value_x[ii]); Serial.print("  ");
  distance[ii]=sqrt( pow(ADC_Value_x[ii], 2) + pow(ADC_Value_y[ii], 2) + pow(ADC_Value_z[ii], 2));

  ii++;
  delay(Sampling_Time);  //fs = 25 Hz. 1/25 = 0.04 = 40msec
//replace delay with TICKS -> more accurate

  //for MAX30105
  //put this part of the code in another loop/task/function with the same priority
   ReadSamples();      //collects samples from sensor readings
  //ADC_range();
  if (flag==0){
    Serial.println("finger unplugged");
    Pulse_Rate_previous=70;
    //SpO2_previous=98;
  } else {
    FilterData();
    ComputeHeartRate();
   // ComputeSpO2();  //this function doesnt exist in the code yet
  }
  flag=1;
}