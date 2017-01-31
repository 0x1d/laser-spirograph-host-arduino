//////////////////////////////////////////////////////////////
////// Bluetooth Laser Spyrograph ////////////////////////////
//////////////////////////////////////////////////////////////
//
//  Version:      1.3.3 14/06/2012
//  
//  Hardware:     Custom-built Board w/:
//		    - ATmega328 16 MHz 
//		    - Voltage Regulation 9V (Motors) 5V (uC Vcc) 3.3V (Laser Driver)
//		    - N-Channel FETs for Motors and Laser
//		    - Serial connector footprint compatible to 
//	                FTDI-Breakoutboard, RN41/42 Bluetoothmate, etc
//
//  MessageSet:   (SetMotorSpeed) (MotorNr) (Speed)
//                AT SMS 1  1 
//                        -  -
//                        3  128
//
//                (SetLaserValue) (Value)
//                AT SLV 1        Set Laser ON
//                        2        10Hz 50% Duty Cyle
//                        -        -
//                        127      2Hz  50% Duty Cyle
//                        128      Set Laser OFF
//
//                (SetDebugLevel) (Level)
//                AT SDL 1        1 = Debug OFF
//                        2        2 = Errors Only
//                        3        3 = All Returns active 
//                        Debug Level not persistent!
//
//		  (SetPresetValues) (tempPresetNr) (M1) (M2) (M3) (LaserValue)
//		  AT SPV 1   1   1   1   1
//			  -   -   -   -   -
//  			  200 128 128 128 128
//
//		  (RunPresetMode) (Cycle/Random) (Time*1s)
//		  AT RPM 1  1      1 = Cycle Presets  || 1 = 1sek
//		          2  120    2 = Random Presets || 120 = 2 min
//  				  	   
//		  (ReadPresetNr) 
//		  AT RPN 1-200
//
//		  (LoadPresetNr) 
//		  AT LPN 1-200
// 
//		  (StopPresetMode) 
//		  AT SPM 
//		  
//		  (SetMotorCorrection) 
//		  AT SMC 0-255 
//
//                (ReadEEProm)-> Return the full EEProm
//                AT REE
//
//                (writeEE)-> init/reset the EEProm 
//		  AT initEEProm
//  				  	   
//
////////////////////////////////////////////////////////////////////////
#include <Messenger.h>
#include <TimerOne.h>
#include <EEPROM.h>

////////////////////////////////////////////////////////////////////////
#define liveLED 10   // Live LED 
#define presetModeLED 14   //14 PresetMode Active LED
#define presetModeButton 15    //15 PresetMode Start Button
#define OUTMotor1 3  // Motor 1 Pin 
#define OUTMotor2 5 // Motor 2 Pin  
#define OUTMotor3 6 // Motor 3 Pin  
#define OUTLaser 9 // Laser Pin 

Messenger message = Messenger(); // Instantiate Messenger object 

////////////////////////////////////////////////////////////////////////
long x,y,z;
byte motor1Value = 0; 
byte motor2Value = 0; 
byte motor3Value = 0; 
byte laserValue = 0; 
byte presetMode = 0;
byte presetCycleTime = 0;
byte presetNr = 0;
byte MotorNr = 0;
byte tempMotor1Value = 0; 
byte tempMotor2Value = 0; 
byte tempMotor3Value = 0; 
byte tempLaserValue = 0; 
byte tempPresetMode = 0;
byte tempPresetCycleTime = 0;
byte tempPresetNr = 0;
byte lastPresetNr = 0;
byte tempMotorNr = 0;
byte liveLedStatus = 0;
byte debugLevel = 3;
byte tempDebugLevel = 3;
byte errorStatus = 0;
byte eeAdd = 0;
byte presetModeActive = 0;
byte motorCorrection = 0;
byte tempMotorCorrection = 0;
byte presetButtonState = 0;

unsigned long slowLoopTime = 0; 
unsigned long lastPresetTime = 0; 

////////////////////////////////////////////////////////////////////////
void setup()
{
  // Define Pins as Output or Input
  pinMode(liveLED, OUTPUT);
  pinMode(presetModeLED, OUTPUT);
  pinMode(presetModeButton, INPUT);
  pinMode(OUTMotor1, OUTPUT);
  pinMode(OUTMotor2, OUTPUT);
  pinMode(OUTMotor3, OUTPUT);
  
  // Initiate Serial Communication and attach interrupt
  Serial.begin(9600); 
  message.attach(parseMessage);

  // initialize timer1 and set Laserperiod to 0
  Timer1.initialize(500000);
  Timer1.pwm(OUTLaser, 0); 

  // read motorCorrection from EEProm
  motorCorrection = 60; //EEPROM.read(1024);
}

////////////////////////////////////////////////////////////////////////
void loop()
{  
  // The following line is the most effective way of 
  // feeding the serial data to Messenger
  while ( Serial.available() ) message.process(Serial.read() );

  // Slow Loop runs every 250ms
  if(millis() - slowLoopTime >= 250)
  {
    blinkLED();
    checkPresetButton();    
    slowLoopTime = millis(); 
  }

  // PresetMode 
  if(presetModeActive == 1)
  {
    activePresetMode();
  }
}

////////////////////////////////////////////////////////////////////////
void blinkLED()
{
  if(liveLedStatus == 0)
  {
    digitalWrite(liveLED, HIGH);
    liveLedStatus = 1;
  }
  else
  {
    digitalWrite(liveLED, LOW);
    liveLedStatus = 0;
  }
}

////////////////////////////////////////////////////////////////////////
void parseMessage()
{
  if ( message.checkString("AT") ) 
  {
    if ( message.checkString("SMS") && presetModeActive == 0 )
    {
      setMotorSpeed();
    } 
    else if ( message.checkString("SLV") && presetModeActive == 0 )
    {
      setLaserValue();
    } 
    else if ( message.checkString("SPV") )
    {
      setPresetValues();
    }
    else if ( message.checkString("LPN") )
    {
      loadPresetNr();
    }
    else if ( message.checkString("RPN") )
    {
      readPresetNr();
    }
    else if ( message.checkString("RPM") )
    {
      runPresetMode();
    }
    else if ( message.checkString("SPM") )
    {
      stopPresetMode();
    }
    else if ( message.checkString("SDL") )
    {
      setDebugLevel();
    }
    else if ( message.checkString("REE") )
    {
      readEE();
    }
    else if ( message.checkString("SMC") )
    {
      setMotorCorrection();
    }   
    else if ( message.checkString("initEEProm") )
    {
      writeEE();
    }
    else
    {
      returnMessageError();
    }
  } 
  else 
  {
    returnMessageError();
  }
}

////////////////////////////////////////////////////////////////////////
void returnMessageError()
{
  if(debugLevel >= 2)
  { 
    Serial.println("Message Error"); 
  }   
}


////////////////////////////////////////////////////////////////////////
void activePresetMode()
{
    if(millis() - lastPresetTime >= presetCycleTime * 1000)
    {
      if(presetMode == 1)	
      {
        // Cylce Mode
        tempPresetNr = lastPresetNr + 1;
        lastPresetNr = tempPresetNr;
      }
      else if(presetMode == 2)
      {
        // Random Mode
        tempPresetNr = random(1, 201);
      }
      

      eeAdd = (tempPresetNr - 1) * 5;

      tempMotor1Value = EEPROM.read(eeAdd);
      tempMotor2Value = EEPROM.read(eeAdd + 1); 
      tempMotor3Value = EEPROM.read(eeAdd + 2); 
      tempLaserValue  = EEPROM.read(eeAdd + 3); 		  

      motor1Value = tempMotor1Value - 1;
      if(motor1Value == 0)
      {
        analogWrite(OUTMotor1, 0);
      }
      else
      {
        analogWrite(OUTMotor1, motor1Value + motorCorrection);
      }

      motor2Value = tempMotor2Value - 1;
      if(motor2Value == 0)
      {
        analogWrite(OUTMotor2, 0);
      }
      else
      {
        analogWrite(OUTMotor2, motor2Value + motorCorrection);
      }

      motor3Value = tempMotor3Value - 1;
      if(motor3Value == 0)
      {
        analogWrite(OUTMotor3, 0);
      }
      else
      {
        analogWrite(OUTMotor3, motor3Value + motorCorrection);
      }
	  
	  laserValue = tempLaserValue - 1;

      if(laserValue == 0)
      {
        Timer1.setPwmDuty(OUTLaser, 1023); // set Laser always on
      }
      else if(laserValue == 127)
      {
        Timer1.setPwmDuty(OUTLaser, 0); // set Laser always off
      }
      else
      {
        Timer1.setPeriod( map(laserValue, 1, 126, 20000, 1000000) );              
        Timer1.setPwmDuty(OUTLaser, 512);  // set PWM with 50% Duty Cycle
      }
  
        if(debugLevel == 3)
        { 
          Serial.print(tempPresetNr, DEC);
          Serial.print(" ");
          Serial.print(motor1Value, DEC);
          Serial.print(" ");
          Serial.print(motor2Value, DEC);
          Serial.print(" ");
          Serial.print(motor3Value, DEC);
          Serial.print(" ");
          Serial.println(tempLaserValue, DEC);
        }
  
        lastPresetTime = millis(); 
        
        presetButtonState = digitalRead(presetModeButton);
        if(presetButtonState)
        {
          delay(30);
          if(presetButtonState)
          {
            presetModeActive = 0;  
            lastPresetNr = 0;
            digitalWrite(presetModeLED, LOW);
            Serial.println("Preset Mode Stopped...");
            delay(200);
          }
        }
  
  
    }
}

////////////////////////////////////////////////////////////////////////
void setMotorSpeed()
{
  tempMotorNr = message.readInt(); // Read first Value (Motor Number 1-3)      
  tempMotor1Value = message.readInt(); // Read second Value (Speed 1-128) 

  if(tempMotorNr >= 1 && tempMotorNr <= 3)
  {
    if(tempMotor1Value >= 1 && tempMotor1Value <= 128) 
    {
      // Set Motor Speed
      if(tempMotorNr == 1)
      {
        motor1Value = tempMotor1Value - 1;
        if(motor1Value == 0)
        {
          analogWrite(OUTMotor1, 0);
        }
        else
        {
          analogWrite(OUTMotor1, motor1Value + motorCorrection);
        }            
      }
      else if(tempMotorNr == 2)
      {
        motor1Value = tempMotor1Value - 1;
        if(motor1Value == 0)
        {
          analogWrite(OUTMotor2, 0);
        }
        else
        {
          analogWrite(OUTMotor2, motor1Value + motorCorrection );
        }
      }
      else if(tempMotorNr == 3)
      {
        motor1Value = tempMotor1Value - 1;
        if(motor1Value == 0)
        {
          analogWrite(OUTMotor3, 0);
        }
        else
        {
          analogWrite(OUTMotor3, motor1Value + motorCorrection );
        }
      }
      
      if(debugLevel == 3)
      {         
        Serial.print("OK ");  
        Serial.print("MotorNr:");
        Serial.println(tempMotorNr, DEC); 
        Serial.print("Speed:");  
        Serial.println(tempMotor1Value, DEC);     
      }
    }
    else 
    {
      if(debugLevel >= 2)
      {  
        Serial.println("Value Error"); 
        Serial.println("Speed must be between 1-128");
        Serial.print("Your Speed:");
        Serial.println(tempMotor1Value, DEC);
      }
    }
  }
  else 
  {
    if(debugLevel >= 2)
    {  
      Serial.println("Value Error"); 
      Serial.println("MotorNr must be between 1-3");
      Serial.print("Your Value:");
      Serial.println(tempMotorNr, DEC);
    }
  }
}

////////////////////////////////////////////////////////////////////////
void setLaserValue()
{

  tempLaserValue = message.readInt();

  if(tempLaserValue >= 1 && tempLaserValue <= 128) 
  {

    laserValue = tempLaserValue - 1;

    if(laserValue == 0)
    {
      Timer1.setPwmDuty(OUTLaser, 1023); // set Laser always on
    }
    else if(laserValue == 127)
    {
      Timer1.setPwmDuty(OUTLaser, 0); // set Laser always off
    }
    else
    {
      Timer1.setPeriod( map(laserValue, 1, 126, 100000, 1000000) );              
      Timer1.setPwmDuty(OUTLaser, 512);  // set PWM with 50% Duty Cycle
    }

    if(debugLevel == 3)
    {         
      Serial.print("OK"); 
      Serial.print(" "); 
      Serial.print(":laserValue:"); 
      Serial.println(laserValue, DEC); 
    }
  }
  else 
  {
    if(debugLevel >= 2)
    {  
      Serial.println("Value Error"); 
      Serial.println("laserValue must be between 1-128");
      Serial.print("laserValue:"); 
      Serial.println(tempLaserValue + 1, DEC); 
    }
  }
}

////////////////////////////////////////////////////////////////////////
void setPresetValues()
{   
  tempPresetNr = message.readInt(); // (1-200)
  tempMotor1Value = message.readInt(); // (1-128) 
  tempMotor2Value = message.readInt(); // (1-128)
  tempMotor3Value = message.readInt(); // (1-128)
  tempLaserValue = message.readInt(); // (1-128)

  // Never trust input Data...
  if(tempPresetNr == 0 || tempPresetNr > 200)
    errorStatus = 1; 
  if(tempMotor1Value == 0 || tempMotor1Value > 128) 
    errorStatus = 1;
  if(tempMotor2Value == 0 || tempMotor2Value > 128) 
    errorStatus = 1;
  if(tempMotor3Value == 0 || tempMotor2Value > 128) 
    errorStatus = 1;
  if(tempLaserValue == 0 || tempLaserValue > 128) 
    errorStatus = 1;

  // Write Values to EEPROM 
  if(errorStatus == 0)
  {
    eeAdd = (tempPresetNr -1) * 5;

    EEPROM.write(eeAdd, tempMotor1Value);
    EEPROM.write(eeAdd + 1, tempMotor2Value);
    EEPROM.write(eeAdd + 2, tempMotor3Value);
    EEPROM.write(eeAdd + 3, tempLaserValue);

    if(debugLevel == 3)
    {         
      Serial.println("OK"); 
      Serial.print(tempPresetNr, DEC); 
      Serial.print(" "); 
      Serial.print(tempMotor1Value, DEC); 
      Serial.print(" "); 
      Serial.print(tempMotor2Value, DEC); 
      Serial.print(" "); 
      Serial.print(tempMotor3Value, DEC); 
      Serial.print(" "); 
      Serial.println(tempLaserValue, DEC); 
    }
  }
  else 
  {
    if(debugLevel >= 2)
    {  
      Serial.println("Error"); 
      Serial.print("Your Values:");
      Serial.print(tempPresetNr, DEC);
      Serial.print(" "); 
      Serial.print(tempMotor1Value, DEC);
      Serial.print(" "); 
      Serial.print(tempMotor2Value, DEC);
      Serial.print(" "); 
      Serial.print(tempMotor3Value, DEC);
      Serial.print(" "); 
      Serial.println(tempLaserValue, DEC);
    }
    errorStatus = 0;
  }
}

////////////////////////////////////////////////////////////////////////
void runPresetMode()
{
  presetMode = message.readInt(); // Read presetMode  
  presetCycleTime = message.readInt(); // Read presetCycleTime

  presetModeActive = 1;
  digitalWrite(presetModeLED, HIGH);
  Serial.println("Preset Mode Active...");
}

////////////////////////////////////////////////////////////////////////
void checkPresetButton()
{
  presetButtonState = digitalRead(presetModeButton);
  if(presetButtonState)
  {
    delay(200);
    if(presetButtonState)
    {
      presetMode = 1; // set cycle
      presetCycleTime = 2; // Read presetCycleTime
  
      presetModeActive = 1;
      digitalWrite(presetModeLED, HIGH);
      Serial.println("Preset Mode Active...");
      delay(200);
    }
  }
}

////////////////////////////////////////////////////////////////////////
void stopPresetMode()
{
  presetModeActive = 0;  
  lastPresetNr = 0;
  digitalWrite(presetModeLED, LOW);
  Serial.println("Preset Mode Stopped...");
}

////////////////////////////////////////////////////////////////////////
void readPresetNr()
{   
  presetModeActive = 0;    
  tempPresetNr = message.readInt(); // Read presetCycleTime

  if(tempPresetNr > 0 && tempPresetNr < 200)
  {
    eeAdd = (tempPresetNr - 1) * 5;
  
    tempMotor1Value = EEPROM.read(eeAdd);
    tempMotor2Value = EEPROM.read(eeAdd + 1); 
    tempMotor3Value = EEPROM.read(eeAdd + 2); 
    tempLaserValue  = EEPROM.read(eeAdd + 3); 		  
  
      Serial.print("PrNr:");
      Serial.print(tempPresetNr, DEC);
      Serial.print("! M1:");
      Serial.print(tempMotor1Value, DEC);
      Serial.print("! M2:");
      Serial.print(tempMotor2Value, DEC);
      Serial.print("! M3:");
      Serial.print(tempMotor3Value, DEC);
      Serial.print("! LASER:");
      Serial.print(tempLaserValue, DEC);
      Serial.println("!");
     
  }
}

////////////////////////////////////////////////////////////////////////
void loadPresetNr()
{   
  presetModeActive = 0;    
  tempPresetNr = message.readInt(); // Read presetCycleTime

  if(tempPresetNr > 0 && tempPresetNr < 200)
  {
    eeAdd = (tempPresetNr - 1) * 5;
  
    tempMotor1Value = EEPROM.read(eeAdd);
    tempMotor2Value = EEPROM.read(eeAdd + 1); 
    tempMotor3Value = EEPROM.read(eeAdd + 2); 
    tempLaserValue  = EEPROM.read(eeAdd + 3); 		  
  
    motor1Value = tempMotor1Value - 1;
    if(motor1Value == 0)
    {
      analogWrite(OUTMotor1, 0);
    }
    else
    {
      analogWrite(OUTMotor1, motor1Value + motorCorrection);
    }
  
    motor2Value = tempMotor2Value - 1;
    if(motor2Value == 0)
    {
      analogWrite(OUTMotor2, 0);
    }
    else
    {
      analogWrite(OUTMotor2, motor2Value + motorCorrection);
    }
  
    motor3Value = tempMotor3Value - 1;
    if(motor3Value == 0)
    {
      analogWrite(OUTMotor3, 0);
    }
    else
    {
      analogWrite(OUTMotor3, motor3Value + motorCorrection);
    }
    
     laserValue = tempLaserValue - 1;
  
      if(laserValue == 0)
      {
        Timer1.setPwmDuty(OUTLaser, 1023); // set Laser always on
      }
      else if(laserValue == 127)
      {
        Timer1.setPwmDuty(OUTLaser, 0); // set Laser always off
      }
      else
      {
        Timer1.setPeriod( map(laserValue, 1, 126, 100000, 1000000) );              
        Timer1.setPwmDuty(OUTLaser, 512);  // set PWM with 50% Duty Cycle
      }
  
    if(debugLevel == 3)
    { 
      Serial.print("PrNr:");
      Serial.print(tempPresetNr, DEC);
      Serial.print("! M1:");
      Serial.print(motor1Value, DEC);
      Serial.print("! M2:");
      Serial.print(motor2Value, DEC);
      Serial.print("! M3:");
      Serial.print(motor3Value, DEC);
      Serial.print("! LASER:");
      Serial.println(tempLaserValue, DEC);
    }  
  }


}

////////////////////////////////////////////////////////////////////////
void setDebugLevel()
{
  tempDebugLevel = message.readInt(); // Read Debug Level
  if(tempDebugLevel >= 1 && tempDebugLevel <= 3)
  {
    debugLevel = tempDebugLevel;
    Serial.print("new Debug Level:"); 
    Serial.println(debugLevel, DEC );
  }
  else 
  {
    if(debugLevel >= 2)
    {  
      Serial.println("Value Error"); 
      Serial.println("Debug Level must be between 1-3");
      Serial.print("Your Value:");
      Serial.println(tempDebugLevel, DEC);
    }
  }
}

////////////////////////////////////////////////////////////////////////
void readEE()
{
  x = 0; 
  for(int address = 0; address <= 1024; address++)
  {
    int value = EEPROM.read(address);

    Serial.print(address);
    Serial.print(":");
    Serial.print(value, DEC);
    Serial.print("!");

    if(x == 4)
    {
      x = 0;
      Serial.println();
    }
    else
    {
      x++;
    }
    delay(5);
  }
}

////////////////////////////////////////////////////////////////////////
void setMotorCorrection()
{
  tempMotorCorrection = message.readInt(); // Read new MotorCorrection
  if(tempMotorCorrection >= 0 && tempMotorCorrection <= 255)
  {
    motorCorrection = tempMotorCorrection;
    EEPROM.write(1024, tempMotorCorrection);
    Serial.print("new MotorCorrection:"); 
    Serial.println(motorCorrection, DEC );
  }
  else 
  {
    if(debugLevel >= 2)
    {  
      Serial.println("Value Error"); 
      Serial.println("Motor Correction must be between 0-255");
      Serial.print("Your Value:");
      Serial.println(tempMotorCorrection, DEC);
    }
  }
}

////////////////////////////////////////////////////////////////////////
void writeEE()
{
  x = 0; 
  y = 1;
  for(int address = 0; address <= 1023; address++)
  {
    EEPROM.write(address, random(1, 127));

    if(x == 4)
    {
      x = 0;
      y++;
    }
    else if(y == 128)
    {
      y = 0;
    }
    else 
    {
      x++;
    }
    delay(5);
  }
  delay(10);
  EEPROM.write(1024, 140);
  Serial.println("***done***");
}








