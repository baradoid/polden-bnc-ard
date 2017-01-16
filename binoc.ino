
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

int sharpVal = 0;
int lastSharpVal = 0;

boolean bFan1On = false, bFan1OnLast = false;
boolean bFan2On = false, bFan2OnLast = false;
boolean bHeatOn = false, bHeatOnLast = false;
char  fanHeatStateString[10] = "DDD";

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS PD7
#define TEMPERATURE_PRECISION 9 // Lower resolution
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
int numberOfDevices = 0; // Number of temperature devices found
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address


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
    
  digitalWrite(pinRele1, HIGH); 
  digitalWrite(pinRele2, HIGH);  
  delay(5000); 
  digitalWrite(pinRele1, LOW);  

  pinMode(pinHeat, OUTPUT); 
  digitalWrite(pinHeat, LOW);  
    
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
}

int lastXpos1=0, lastXpos2=0;
int xPos1 = 0, xPos2 = 0;

int valArr[13];
char str[50];
int lastAndrCpuTemp = 0, andrCpuTemp=0;
// the loop function runs over and over again forever
unsigned long lastTempContrTime = 0;
unsigned long lastFanContrTime = 0;
unsigned long lastDistContrTime = 0;

int lastTempC = 0;
int tempC = -990;

unsigned long lastSendReportTime = 0;
unsigned long maninCntr = 0;

void loop() 
{

  maninCntr++;
  readSerial();
  getPos();
  
  unsigned long curTime = millis();

  if((curTime - lastDistContrTime) > 200 ){    
    lastDistContrTime = curTime;    
    getDistance();
  }
  
  if(sharpVal > 30){
    if((curTime - lastTempContrTime) > 1000){    
      lastTempContrTime = curTime;
      getTemp();  
      controlHeat();
    }
    digitalWrite(pinMute, HIGH);  
  }
  else{
    digitalWrite(pinMute, LOW);      
  }
  
  if((curTime - lastFanContrTime) > 1000){
      lastFanContrTime = curTime;
      controlFan();   
      sprintf(fanHeatStateString,"%c%c%c", bFan1On? 'E':'D', 
                                           bFan2On? 'E':'D', 
                                           bHeatOn? 'E':'D');
  }

  if((curTime - lastSendReportTime) > 10){
    lastSendReportTime = curTime;
    if( (xPos1 != lastXpos1) || (xPos2 != lastXpos2) || (lastSharpVal != sharpVal) || 
     (lastTempC != tempC) || (lastAndrCpuTemp != andrCpuTemp) ||
     (bFan1On != bFan1OnLast) || (bFan2On != bFan2OnLast) ||(bHeatOn != bHeatOnLast)){
      
      sprintf(str,"%04X %04X %04d %04d %04d %s", xPos1, xPos2,
                                              tempC, sharpVal, 
                                              andrCpuTemp, fanHeatStateString);
      Serial.println(str);  
      
      lastXpos1 = xPos1;
      lastXpos2 = xPos2;
      lastSharpVal = sharpVal;
      
      lastTempC = tempC;   
      lastAndrCpuTemp = andrCpuTemp;

      bFan1OnLast = bFan1On;
      bFan2OnLast = bFan2On;
      bHeatOnLast = bHeatOn;
            
    } 
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
void getTemp()
{    
  if(numberOfDevices > 0){
    if(sensors.requestTemperaturesByIndex(0) == true){
      tempC = (int)(sensors.getTempC(tempDeviceAddress)*10);
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
    tempC = -990;
      numberOfDevices = sensors.getDeviceCount();    
      for(int i=0;i<numberOfDevices; i++)
      {
        if(sensors.getAddress(tempDeviceAddress, i))
        {
          sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
        }
      }
  }
  
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

void getDistance()
{
  int mV = analogRead(pinSharp)*4.9; //in mV  
  if(mV < 500){              
    sharpVal = calcPoly(mV, 500, 250, 25, 40); //500to250 mv = 25to40;    
  }
  else if(mV < 1000){              
    sharpVal = calcPoly(mV, 1000, 500, 12, 25); //1000to500 mv = 12to25;    
  }
  else{          
    sharpVal = calcPoly(mV, 3000, 1000, 3,12);   //3000to1000 mv = 3to12     
  }
  //sharpVal = mV;
  
}

void getPos()
{
//  int pinClkSet = PIND|(1<<pinClk);
//  int pinClkClr = PIND&~(1<<pinClk);
//
//  noInterrupts();
//  PORTD = pinClkClr;
//  PORTD = pinClkClr;
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

String inString = "";  
void readSerial()
{
    while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }

    if (inChar == '\n') {
      andrCpuTemp = inString.toInt();
      inString = "";
               
    }
  }
}

void controlFan()
{    
  if(andrCpuTemp > 30){
    bFan1On = true;
    bFan2On = true;
    digitalWrite(pinFan1Int, LOW); 
    digitalWrite(pinFan2Ext, LOW); 
  }
  else{
    bFan1On = false;
    bFan2On = false;

    digitalWrite(pinFan1Int, HIGH); 
    digitalWrite(pinFan2Ext, HIGH); 
  }      
}


const unsigned long heatMaximumEnableTimeSec = 7; //включаем нагрев максимум на это время
const unsigned long heatEnablePeriodTimeSec = 60; //не чаще чем один раз в это время
unsigned long lastHeatEnableTime = 0;
enum heatState_t {off, on};
heatState_t heatState = off;
void controlHeat()
{
  unsigned long curTime = millis()/1000;
  //Serial.print(curTime);
  //Serial.print(" ");
  if(tempC > -99){
    if(tempC < 50){
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

