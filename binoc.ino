
//#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//SoftwareSerial mySerial(10, 11); // RX, TX

int pinClk = 3;
int pinData1 = 4;
int pinData2 = 5;

int pinData1Mask = 1<<pinData1;
int pinData2Mask = 1<<pinData2;

int pinRele1=9;
int pinRele2=10;

int pinFan1Int = 11; //PB3;
int pinFan2Ext = 12; //PD6;

int pinSharp = A0;
int pinHeat = A2;

int pinMute = 2;


// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS PD7
#define TEMPERATURE_PRECISION 9 // Lower resolution
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
int numberOfDevices = 0; // Number of temperature devices found
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address


uint16_t fanRpm[3] = {0, 0, 0};
int lastXpos1=0, lastXpos2=0;
int xPos1 = 0, xPos2 = 0;

int valArr[13];
char str[50], lastStr[50];
int lastAndrCpuTemp = 0, andrCpuTemp=0;

unsigned long lastTempContrTime = 0;
unsigned long lastFanContrTime = 0;
unsigned long lastDistContrTime = 0;

unsigned long lastSendReportTime = 0;

uint64_t lastPhoneMsgRecvTime = 0;
unsigned long maninCntr = 0;

String inString = "";  

int sharpVal = 0;
int dallasTemp = -99;
//int filteredDist =0;

boolean bFanOn = false;
boolean bHeatOn = false;


void setup() {
  // initialize digital pin 13 as an output.
  //pinMode(13, OUTPUT);
  Serial.begin(115200);
   
  pinMode(pinClk, OUTPUT);           // set pin to input
  digitalWrite(pinClk, LOW);   

  pinMode(pinData1, INPUT);      
  pinMode(pinData2, INPUT); 

  pinMode(pinRele1, OUTPUT); 
  pinMode(pinRele2, OUTPUT); 
    
  pinMode(pinFan1Int, OUTPUT);    
  pinMode(pinFan2Ext, OUTPUT);    
    
  pinMode(pinMute, OUTPUT);  
  digitalWrite(pinMute, HIGH);   
     
  digitalWrite(pinFan1Int, LOW);    
  digitalWrite(pinFan2Ext, LOW);     

  pinMode(pinHeat, OUTPUT); 
  digitalWrite(pinHeat, LOW);  

  pinMode(pinSharp, INPUT); 
    
  Serial.setTimeout(5);

  sensors.begin();
   
  numberOfDevices = sensors.getDeviceCount(); // Grab a count of devices on the wire
  
  // locate devices on the bus
  Serial.print("Locating Dallas devices...");
  
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // report parasite power requirements
//  Serial.print("Parasite power is: "); 
//  if (sensors.isParasitePowerMode()) Serial.println("ON");
//  else Serial.println("OFF");

    // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
    {
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
      
      //Serial.print("Setting resolution to ");
      //Serial.println(TEMPERATURE_PRECISION, DEC);
      
      // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
      
//       Serial.print("Resolution actually set to: ");
//      Serial.print(sensors.getResolution(tempDeviceAddress), DEC); 
//      Serial.println();
    }
    else{
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }
  
  formatData();   
  
  resetPhone();
}


void loop() 
{
  maninCntr++;
  if(readSerial() == true){
    if(inString == "reset\n"){
      resetPhone();          
      lastPhoneMsgRecvTime = millis();
    }
    else if(inString == "son\n"){
      soundOn();      
      lastPhoneMsgRecvTime = millis();
    }
    else if(inString == "soff\n"){
      soundOff();      
      lastPhoneMsgRecvTime = millis();
    }
    else if(inString.startsWith("t=") == true){
      inString.remove(0, 2);

      andrCpuTemp = inString.toInt();
      //sprintf(&(str[20]),"%04d", andrCpuTemp);            
      lastPhoneMsgRecvTime = millis();
    }
    
    inString = "";
  }
  
  getPos();    
  
  unsigned long curTime = millis();

  if((curTime - lastDistContrTime) > 50 ){    
    lastDistContrTime = curTime;    
    getDistance();
    //sprintf(&(str[15]),"%04d", sharpVal);        
    //print04d(&(str[15]), sharpVal);
  }

  //formatData(); 
  
  if((curTime - lastFanContrTime) > 1000){
      lastFanContrTime = curTime;

    if(isSoundEnabled() == false){      
      dallasTemp = getTemp();  
      controlHeat(dallasTemp); 
      //sprintf(str, "%04X %04X %04d %04d %04d    000 000 000", xPos1, xPos2, dallasTemp, sharpVal, andrCpuTemp);        
    }
    controlFan(); 

    if(isSoundEnabled() == false){
      //formatData();      
    }

  }
  
  sprintf(str, "%04X %04X %04d %04d %04d    000 000 000", xPos1, xPos2, dallasTemp, sharpVal, andrCpuTemp);
  str[25] = bFanOn? 'E':'D';
  str[26] = bHeatOn? 'E':'D';
  str[4] = str[9] = str[14] = str[19] = str[24] = str[27] = str[31] = str[35] = ' ';      
  str[39] = 0;
  
  if((curTime - lastSendReportTime) > 10){       
    lastSendReportTime = curTime;
    if(strcmp(str, lastStr) != 0){
      strcpy(lastStr, str);
      Serial.println(str);        
    }
  }
  
  if( ((millis() - lastPhoneMsgRecvTime)/1000) > 300){
    resetPhone();
    lastPhoneMsgRecvTime = millis();
  }
}


// function to print the temperature for a device
//void printTemperature(DeviceAddress deviceAddress)
//{
//  // method 1 - slower
//  //Serial.print("Temp C: ");
//  //Serial.print(sensors.getTempC(deviceAddress));
//  //Serial.print(" Temp F: ");
//  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit
//
//  // method 2 - faster
//  tempC = sensors.getTempC(deviceAddress);
//  //Serial.print("Temp C: ");
//  //Serial.println(tempC);
//  //Serial.print(" Temp F: ");
//  //Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
//}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
int getTemp()
{   
  int t = -990; 
  if(numberOfDevices > 0){
    if(sensors.requestTemperaturesByIndex(0) == true){
      t = (int)(sensors.getTempC(tempDeviceAddress)*10);
    }
    else{
      numberOfDevices--;      
    }
    
//    for(int i=0;i<numberOfDevices; i++){
//    // Search the wire for address
//      if(sensors.getAddress(tempDeviceAddress, i)){
//      // Output the device ID
//      //Serial.print("Temperature for device: ");
//      //Serial.println(i,DEC);
//         
//      // It responds almost immediately. Let's print out the data
//      //printTemperature(tempDeviceAddress); // Use a simple function to print out the data
//        tempC = (int)(sensors.getTempC(tempDeviceAddress)*10);
//      } 
//        //else ghost device! Check your power requirements and cabling        
//    }
  }
  else{
    t = -990;
    numberOfDevices = sensors.getDeviceCount();    
    for(int i=0;i<numberOfDevices; i++){
      if(sensors.getAddress(tempDeviceAddress, i)){
        sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
      }
    }
  }
  return t;
}

int calcPoly(int mV, int mV1, int mV2, int cm1, int cm2)
{
  float d = abs(mV1-mV2) / (float)abs(cm1-cm2);
  int dist = 60;
  if(d > 0){
    dist = ((mV1-mV)/d) + cm1;
  }
  return dist;
}

int recalcMvToCm(int mV)
{
  int dist = 0;
  if(mV < 500){              
    dist = calcPoly(mV, 500, 250, 25, 40); //500to250 mv = 25to40; 
  }
  else if(mV < 1000){              
    dist = calcPoly(mV, 1000, 500, 12, 25); //1000to500 mv = 12to25;
  }
  else{          
    dist = calcPoly(mV, 3000, 1000, 3,12);   //3000to1000 mv = 3to12
  }
  return dist;
  
}
void getDistance()
{
  int mV = fir(analogRead(pinSharp))*4.9; //in mV  

  //sharpVal = recalcMvToCm(mV);
  sharpVal = recalcMvToCm(mV);

  //filteredDist = recalcMvToCm((int)filter(mV));  
}

void getPos()
{
//  int pinClkSet = PIND|(1<<pinClk);
//  int pinClkClr = PIND&~(1<<pinClk);
//
//  noInterrupts();
//  PORTD = pinClkClr;
//  PORTD = pinClkClr;
//delayMicroseconds(50);     //!!!
//  PORTD = pinClkClr;
//  for(int i=0; i<13; i++){         
//    PORTD = pinClkSet; 
//    PORTD = pinClkSet; 
//    PORTD = pinClkSet; 
//    valArr[i] = PIND;
//    PORTD = pinClkClr;    
//    PORTD = pinClkClr;    
//    PORTD = pinClkClr;    
//  }  
//  PORTD = pinClkSet;
//  PORTD = pinClkSet;
//  PORTD = pinClkSet;
//  interrupts();
  
  noInterrupts();
  digitalWrite(pinClk, LOW);
  for(int i=0; i<13; i++){         
    digitalWrite(pinClk, HIGH);  //for OK connection
    valArr[i] = PIND;
    digitalWrite(pinClk, LOW); 
  }  
  digitalWrite(pinClk, HIGH);
  interrupts();
  xPos1 = 0; xPos2 = 0;
  for(int i=0; i<13; i++){        
    xPos1 |= (((valArr[i]&pinData1Mask)>>pinData1)<<(12-i));
    xPos2 |= (((valArr[i]&pinData2Mask)>>pinData2)<<(12-i));    
  } 
}


bool readSerial()
{  
  bool ret = false;
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    //if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    //}

    if (inChar == '\n') {
      //andrCpuTemp = inString.toInt();
      //inString = "";
      ret = true;
      break;              
    }
  }
  return ret;
}

void controlFan()
{    
  if(andrCpuTemp > 30){
    bFanOn = true;
    digitalWrite(pinFan1Int, LOW); 
    digitalWrite(pinFan2Ext, LOW); 
  }
  else{
    bFanOn = false;
    digitalWrite(pinFan1Int, HIGH); 
    digitalWrite(pinFan2Ext, HIGH); 
  }      
}


const unsigned long heatMaximumEnableTimeSec = 7; //включаем нагрев максимум на это время
const unsigned long heatEnablePeriodTimeSec = 60; //не чаще чем один раз в это время
unsigned long lastHeatEnableTime = 0;
enum heatState_t {off, on};
heatState_t heatState = off;
void controlHeat(int t)
{
  unsigned long curTime = millis()/1000;
  //Serial.print(curTime);
  //Serial.print(" ");
  if(t > -99){
    if(t < 50){
      //Serial.print("less15 ");
      switch(heatState){
        case off: 
         //Serial.print("coff ");
         //Serial.print((curTime - lastHeatEnableTime));
         //Serial.print(" ");
          if((curTime - lastHeatEnableTime) > heatEnablePeriodTimeSec){
            //Serial.print("gp ");
            lastHeatEnableTime = curTime;
            heatState = on;
            digitalWrite(pinHeat, HIGH);
            bHeatOn = true;
          }
          else{
            //Serial.print("lp ");
            digitalWrite(pinHeat, LOW);
            bHeatOn = false;
          }
          break;
        case on:
        //Serial.print("con ");        
          if((curTime - lastHeatEnableTime) > heatMaximumEnableTimeSec){
            //Serial.print("ge ");
            digitalWrite(pinHeat, LOW);
            heatState = off;
            bHeatOn = false;
          }    
          else{
            //Serial.print("le ");
            digitalWrite(pinHeat, HIGH);
            bHeatOn = true;            
          }
          break;
      }
      //Serial.println(" ");
    }
    else{
      digitalWrite(pinHeat, LOW);
      heatState = off;        
    }
  }
  else{
      digitalWrite(pinHeat, LOW);
      heatState = off;            
  }
}

double filter(int d)
{
  static double acc = 0;
  acc = 0.2*(double)d + 0.8 *acc;
  return acc;
}

#define Ntap 8

    float FIRCoef[Ntap] = { 
        0.08997650465060308400,
        0.13443834128434992000,
        0.16588587705222707000,
        0.17723405264792491000,
        0.16588587705222707000,
        0.13443834128434992000,
        0.08997650465060308400,
        0.04216450137771470000
    };
float  x[Ntap]; //input samples
float  fir(float  NewSample) {
    float y=0;            //output sample
    int n;

    //shift the old samples
    for(n=Ntap-1; n>0; n--)
       x[n] = x[n-1];

    //Calculate the new output
    x[0] = NewSample;
    for(n=0; n<Ntap; n++)
        y += FIRCoef[n] * x[n];
    
    return y;
}

void resetPhone()
{
  digitalWrite(pinRele2, LOW); 
  delay(2000); 
  digitalWrite(pinRele1, HIGH); 
  digitalWrite(pinRele2, HIGH);  
  delay(5000); 
  digitalWrite(pinRele1, LOW); 
}

bool bSoundEnable = false;
bool isSoundEnabled()
{
  return bSoundEnable;  
}

void soundOn()
{
  digitalWrite(pinMute, LOW);              
  bSoundEnable = true;
}

void soundOff()
{
  digitalWrite(pinMute, HIGH);  
  bSoundEnable = false;
}

void formatData()
{
  sprintf(&(str[10]), "%04d %04d %04d    %03x %03x %03x", 
                      dallasTemp, sharpVal, andrCpuTemp, 
                      fanRpm[0], fanRpm[1], fanRpm[2]);
                          
  str[25] = bFanOn? 'E':'D';
  str[26] = bHeatOn? 'E':'D';
}

