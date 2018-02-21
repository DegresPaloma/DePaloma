# DePaloma
NodeMCU Program for Aquaponics Monitoring System

#include <FS.h>
#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <DallasTemperature.h>
#include <FirebaseArduino.h>
#include <Servo.h>
#include <DHT.h>
#include <NTPtimeESP.h>


//SynchTime
#define DEBUG_ON

//FIREBASE
//#define FIREBASE_HOST "HOST"
//#define FIREBASE_AUTH "AUTH"

//GrowLights
#define REDPIN D3
#define GREENPIN D7
#define BLUEPIN D8

// Water Temperature Sensor
#define ONEWIRE D1

// pH Sensor
#define SensorPin A0
#define samplingInterval 20
#define printInterval 200
#define ArrayLength 40

// Servo Motor
#define servoPin D0

// DHT11
#define DHTPIN D2
#define DHTTYPE DHT11

// Water Level
#define TrigPin D5
#define EchoPin D6

// Water Temperature Global Variables
OneWire oneWire(ONEWIRE);
DallasTemperature DS18B20(&oneWire);
float tempCelsius, tempFahrenheit, humidityData, celData, fehrData;

// pH Sensor Global Variables
float Offset;
float Coeff;
int pHArray[ArrayLength];
int pHArrayIndex = 0;

// Servo Global Variables
Servo myservo;
int pos = 0, repeat = 0;

// DHT22 Global Variables
DHT dht(DHTPIN, DHTTYPE);

// Water Level Global Variables
float ConHeight;
long duration = 0;
float distance, distancecm = 0, distancein = 0;

// NTP
NTPtime NTPch("ph.pool.ntp.org");
strDateTime dateTime;
byte actualHour = 0, actualMinute = 0, actualsecond = 0, actualMonth = 0, actualday = 0;
int actualyear = 0;


void setup() 
{
  Serial.begin(115200);

//Initialize Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

// WiFi Manager
  WiFiManager wifiManager;
  wifiManager.setBreakAfterConfig(true);
  wifiManager.resetSettings();
  
  if (!wifiManager.autoConnect("AquaponicsSystem", "CBELS2018!")) 
  {
    Serial.println("failed to connect, we should reset as see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  Serial.println("Connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  dht.begin();
  myservo.attach(servoPin); 
 
  //Initialize Ultrasonic Sensor
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  //Initialize Growlights
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  
}

void loop() 
{
  //Get Time
  dateTime = NTPch.getNTPtime(8, 0);
  if(dateTime.valid)
  {
    actualHour = dateTime.hour;
    actualMinute = dateTime.minute;
    actualsecond = dateTime.second;
    actualyear = dateTime.year;
    actualMonth = dateTime.month;
    actualday = dateTime.day;
  }
  Firebase.setInt("SetupOne/CurrentTime/Hour",actualHour);
  Firebase.setInt("SetupOne/CurrentTime/Minute",actualMinute);
  
  Offset = Firebase.getFloat("SetupOne/Calibrate/pH/Offset");
  Coeff = Firebase.getFloat("SetupOne/Calibrate/pH/Coeff");
  ConHeight = Firebase.getFloat("SetupOne/Calibrate/WaterLvl/Height");
  
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;

  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  duration = pulseIn(EchoPin, HIGH);
  distance = (duration/58.00);
  distancecm = ConHeight - distance;
  //distancein = distancecm*0.393701;

  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLength)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLength)*5.0/1024;
      pHValue = Coeff*voltage-Offset;

      DS18B20.requestTemperatures();
      tempCelsius = DS18B20.getTempCByIndex(0);
      tempFahrenheit = DS18B20.getTempFByIndex(0);

      humidityData = dht.readHumidity();
      celData = dht.readTemperature();
      fehrData = dht.readTemperature(true);

      samplingTime=millis();
  }
  
  if(millis() - printTime > printInterval)
  {
    Serial.println();

    Serial.print("Current Data & Time: ");
    Serial.print(actualHour);
    Serial.print(":");
    Serial.print(actualMinute);
    Serial.print(":");
    Serial.print(actualsecond);
    Serial.print("  ");
    Serial.print(actualMonth);
    Serial.print("/");
    Serial.print(actualday);
    Serial.print("/");
    Serial.println(actualyear);
    
    Serial.print("Voltage ");
    Serial.print("pH Value  ");
    Serial.print("Water TempC ");
    Serial.print("Water TempF ");
    Serial.print("Air TempC ");
    Serial.print("Air TempF ");
    Serial.print(" Humidity ");
    Serial.println("Water Level");
    Serial.print("  ");
    Serial.print(voltage,2);
    Serial.print("    ");
    Serial.print(pHValue,2);
    Serial.print("       ");
    Serial.print(tempCelsius);
    Serial.print("       ");
    Serial.print(tempFahrenheit);
    
    if (isnan(humidityData) || isnan(celData) || isnan(fehrData))
      {
        Serial.print("        ");
        Serial.print("DHT Sensor Failed!");
        Serial.print("     ");
      }
    else 
      {  
        Serial.print("      ");
        Serial.print(celData);
        Serial.print("     ");
        Serial.print(fehrData);
        Serial.print("     ");
        Serial.print(humidityData);
        Serial.print("        ");
      }

    Serial.print(distancecm);
    Serial.println();
 
    printTime = millis();
  }


  static unsigned long resetvar = millis();
  if(millis()- resetvar > 60000){
    repeat = 0;
    resetvar = millis();
  }

  //Send Data to Firebase
  int DHA = Firebase.getInt("SetupOne/Controls/SendData/Times/Time1/Hour");
  int DMA = Firebase.getInt("SetupOne/Controls/SendData/Times/Time1/Minute");
  int DHB = Firebase.getInt("SetupOne/Controls/SendData/Times/Time2/Hour");
  int DMB = Firebase.getInt("SetupOne/Controls/SendData/Times/Time2/Minute");
  int DHC = Firebase.getInt("SetupOne/Controls/SendData/Times/Time3/Hour");
  int DMC = Firebase.getInt("SetupOne/Controls/SendData/Times/Time3/Minute");
  int DHD = Firebase.getInt("SetupOne/Controls/SendData/Times/Time4/Hour");
  int DMD = Firebase.getInt("SetupOne/Controls/SendData/Times/Time4/Minute");
  int DHE = Firebase.getInt("SetupOne/Controls/SendData/Times/Time5/Hour");
  int DME = Firebase.getInt("SetupOne/Controls/SendData/Times/Time5/Minute");
  int DHF = Firebase.getInt("SetupOne/Controls/SendData/Times/Time6/Hour");
  int DMF = Firebase.getInt("SetupOne/Controls/SendData/Times/Time6/Minute");
  int DM = Firebase.getInt("SetupOne/Controls/SendData/Duration");
  int DataOver = Firebase.getInt("SetupOne/Controls/SendData/Override");
  int DataSwitch = Firebase.getInt("SetupOne/Controls/SendData/Switch");
  if(DataOver == 0){
    if((actualHour == DHA) && (actualMinute == DMA) && (actualMinute <= DMA+DM) 
    || (actualHour == DHB) && (actualMinute == DMB) && (actualMinute <= DMB+DM)
    || (actualHour == DHC) && (actualMinute == DMC) && (actualMinute <= DMC+DM)
    || (actualHour == DHD) && (actualMinute == DMD) && (actualMinute <= DMD+DM)
    || (actualHour == DHE) && (actualMinute == DME) && (actualMinute <= DME+DM)
    || (actualHour == DHF) && (actualMinute == DMF) && (actualMinute <= DMF+DM)){    
      Serial.println("Sending To Firebase");
      Firebase.setString("SetupOne/Controls/SendData/Status","Sending"); 
      Firebase.push("SetupOne/Data/pH Value",pHValue);
      Firebase.push("SetupOne/Data/Water Temperature (C)",tempCelsius);
      Firebase.push("SetupOne/Data/Air Temperature (C)",celData);
      Firebase.push("SetupOne/Data/Humidity",humidityData);
      Firebase.push("SetupOne/Data/Water Level",distancecm);    
    }
    else{
      Firebase.setString("SetupOne/Controls/SendData/Status","Not Sending");
    }
  }
  if(DataOver == 1){
    if(DataSwitch == 1){
      Serial.println("Sending To Firebase");
      Firebase.setString("SetupOne/Controls/SendData/Status","Sending"); 
      Firebase.push("SetupOne/Data/pH Value",pHValue);
      Firebase.push("SetupOne/Data/Water Temperature (C)",tempCelsius);
      Firebase.push("SetupOne/Data/Air Temperature (C)",celData);
      Firebase.push("SetupOne/Data/Humidity",humidityData);
      Firebase.push("SetupOne/Data/Water Level",distancecm);
    }
    if(DataSwitch == 0){
      Firebase.setString("SetupOne/Controls/SendData/Status","Not Sending"); 
    }
  }

  //Feeder
  int THA = Firebase.getInt("SetupOne/Controls/Feeder/Times/Time1/Hour");
  int TMA = Firebase.getInt("SetupOne/Controls/Feeder/Times/Time1/Minute");
  int THB = Firebase.getInt("SetupOne/Controls/Feeder/Times/Time2/Hour");
  int TMB = Firebase.getInt("SetupOne/Controls/Feeder/Times/Time2/Minute");
  int THC = Firebase.getInt("SetupOne/Controls/Feeder/Times/Time3/Hour");
  int TMC = Firebase.getInt("SetupOne/Controls/Feeder/Times/Time3/Minute");
  int Rotation = Firebase.getInt("SetupOne/Controls/Feeder/Rotation");
  int OverFeed = Firebase.getInt("SetupOne/Controls/Feeder/Override");
  int FeedSwitch = Firebase.getInt("SetupOne/Controls/Feeder/Switch");
  if(OverFeed == 0){
    if(((actualHour == THA) && (actualMinute == TMA)) 
    || ((actualHour == THB) && (actualMinute == TMB)) 
    || ((actualHour == THC) && (actualMinute == TMC))){
      while(repeat < Rotation){
        Serial.println("Feeding");
        Firebase.setString("SetupOne/Controls/Feeder/Status","Feeding");    
        RunServo(); 
        repeat++;
      }
    }
    else{
      Firebase.setString("SetupOne/Controls/Feeder/Status","Not Feeding");
      Serial.println("Not Feeding");
    }
  }
  if(OverFeed == 1){
    if(FeedSwitch == 1){
      Firebase.setString("SetupOne/Controls/Feeder/Status","Feeding");
      Serial.println("Feeding");
      RunServo();
    }
    if(FeedSwitch == 0){
      Firebase.setString("SetupOne/Controls/Feeder/Status","Not Feeding");
      Serial.println("Not Feeding");
    }
  }

  //GrowLights
  int Override = Firebase.getInt("SetupOne/Controls/Growlights/Override");
  int Switch = Firebase.getInt("SetupOne/Controls/Growlights/Switch");
  int FHour = Firebase.getInt("SetupOne/Controls/Growlights/Timefrom/Hour");
  int FMin = Firebase.getInt("SetupOne/Controls/Growlights/Timefrom/Minute");
  int THour = Firebase.getInt("SetupOne/Controls/Growlights/Timeto/Hour");
  int TMin = Firebase.getInt("SetupOne/Controls/Growlights/Timeto/Minute");
  if(Override == 0){
    if(((actualHour == FHour) && (actualMinute >= FMin)) 
    || ((actualHour == THour) && (actualMinute <= TMin)) 
    || ((actualHour > FHour) && (actualHour < THour)) 
    || ((actualHour < FHour) && (actualHour > THour))){
      analogWrite(REDPIN,Firebase.getInt("SetupOne/Controls/Growlights/LEDS/LEDRed"));
      analogWrite(GREENPIN,Firebase.getInt("SetupOne/Controls/Growlights/LEDS/LEDGreen"));
      analogWrite(BLUEPIN,Firebase.getInt("SetupOne/Controls/Growlights/LEDS/LEDBlue"));
      Firebase.setString("SetupOne/Controls/Growlights/Status","ON");
      Serial.println("Growlights Status: ON");
    }
    else{
      analogWrite(REDPIN,0);
      analogWrite(GREENPIN,0);
      analogWrite(BLUEPIN,0);
      Firebase.setString("SetupOne/Controls/Growlights/Status","OFF");
      Serial.println("Growlights Status: OFF");
    }
  }
  if(Override == 1){
    if(Switch == 1){
      analogWrite(REDPIN,Firebase.getInt("SetupOne/Controls/Growlights/LEDS/LEDRed"));
      analogWrite(GREENPIN,Firebase.getInt("SetupOne/Controls/Growlights/LEDS/LEDGreen"));
      analogWrite(BLUEPIN,Firebase.getInt("SetupOne/Controls/Growlights/LEDS/LEDBlue"));
      Firebase.setString("SetupOne/Controls/Growlights/Status","ON");
      Serial.println("Growlights Status: ON");
    }
    if(Switch == 0){
      analogWrite(REDPIN,0);
      analogWrite(GREENPIN,0);
      analogWrite(BLUEPIN,0);
      Firebase.setString("SetupOne/Controls/Growlights/Status","OFF");
      Serial.println("Growlights Status: OFF");
    }
  }
  
}

// Functions

void RunServo()
{
      for (pos = 90; pos <= 180; pos += 5) { 
        myservo.write(pos);             
        delay(20);                      
      }
      for (pos = 180; pos >= 90; pos -= 5) { 
        myservo.write(pos);              
        delay(20);                       
      }
      for (pos = 90; pos >= 0; pos -= 5) { 
        myservo.write(pos);              
        delay(20);                      
      }
      for (pos = 0; pos <= 90; pos += 5) { 
        myservo.write(pos);              
        delay(20);                       
      }
    delay(1000);
}

double avergearray(int* arr, int number)
{
  int i;
  int max,min;
  double avg;
  long amount=0;
  
  if(number<=0)
  {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  
  if(number<5)
  {   //less than 5, calculated directly statistics
    for(i=0;i<number;i++)
    {
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }
  
  else
  {
    if(arr[0]<arr[1])
    {
      min = arr[0];max = arr[1];
    }
    
    else
    {
      min=arr[1];max=arr[0];
    }
    
    for(i=2;i<number;i++)
    {
      if(arr[i]<min)
      {
        amount+=min;        //arr<min
        min=arr[i];
      }
      else 
      {
        if(arr[i]>max)
        {
          amount+=max;    //arr>max
          max=arr[i];
        }
        else
        {
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}
