/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://www.arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */
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

int pinFan1Int = PB3;
int pinFan2Ext = PD6;

int pinSharp = A0;

int pinHeat = A2;

int sharpVal = 0;
int lastSharpVal = 0;

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
  Serial.print("Locating devices...");
  
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

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
      
      Serial.print("Setting resolution to ");
      Serial.println(TEMPERATURE_PRECISION, DEC);
      
      // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
      
       Serial.print("Resolution actually set to: ");
      Serial.print(sensors.getResolution(tempDeviceAddress), DEC); 
      Serial.println();
    }
    else{
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }
}

int lastXpos1=0, lastXpos2=0;
int valArr[13];
char str[20];
int andrCpuTemp=0;
// the loop function runs over and over again forever
unsigned long lastTempContrTime = 0;
unsigned long lastDistContrTime = 0;

int xPos1 = 0, xPos2 = 0;

int lastTempC = 0;
int tempC = -990;
void loop() 
{
//  controlFan1();
//
//  unsigned long curTempContrTime = millis();
//  if((curTempContrTime - lastTempContrTime) > 1000){    
//    lastTempContrTime = curTempContrTime;
//    getTemp();
//    controlHeat();
//  }
//
//  if((curTempContrTime - lastDistContrTime) > 100 ){    
//    lastDistContrTime = curTempContrTime;    
//    getDistance();
//  }
    
  getPos();
 
  if((xPos1 != lastXpos1) || (xPos2 != lastXpos2) ||
      (lastTempC != tempC) || 
      (lastSharpVal != sharpVal) ){
    sprintf(str,"%04X %04X %04d %d", xPos1, xPos2, tempC, sharpVal);
    Serial.println(str);    
    lastXpos1 = xPos1;
    lastXpos2 = xPos2;
    lastTempC = tempC;
    lastSharpVal = sharpVal;    
  } 
}


// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  tempC = sensors.getTempC(deviceAddress);
  //Serial.print("Temp C: ");
  //Serial.println(tempC);
  //Serial.print(" Temp F: ");
  //Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}

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
  return ((mV1-mV)/d) + cm1;
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
  digitalWrite(pinClk, LOW);
  digitalWrite(pinClk, HIGH);      
  //digitalWrite(pinClk, LOW);
  for(int i=0; i<13; i++){         
    digitalWrite(pinClk, HIGH);  //for OK connection
    //digitalWrite(pinClk, HIGH);  //for OK connection
    valArr[i] = PIND;
    digitalWrite(pinClk, LOW);    
  }  
  digitalWrite(pinClk, HIGH);

  for(int i=0; i<13; i++){        
    xPos1 |= (((valArr[i]&pinData1Mask)>>pinData1)<<(12-i));
    xPos2 |= (((valArr[i]&pinData2Mask)>>pinData2)<<(12-i));    
  } 
}

void controlFan1()
{
  String inStr = Serial.readString();
  if(inStr.length() > 0){
    andrCpuTemp = inStr.toInt();
    if(andrCpuTemp > 55){
      digitalWrite(pinFan1Int, HIGH); 
      digitalWrite(pinFan2Ext, HIGH); 
    }
    else{
      digitalWrite(pinFan1Int, LOW); 
      digitalWrite(pinFan2Ext, LOW); 
    }
  }
}


void controlHeat()
{
  if(tempC > -99){
    if(tempC > 200){
      digitalWrite(pinHeat, LOW);   
    }
    else if(tempC < 150){
      digitalWrite(pinHeat, HIGH);       
    }
  }

}

