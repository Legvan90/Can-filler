/*
Project: NinaAutomation 3
Created: Primoz Flander 9.6.2018

Arduino pin   /      I/O:
DI2           ->     Water flow meter (INT0)
DO3           ->     LCD D7
DO4           ->     LCD D6
DO5           ->     LCD D5
DO6           ->     LCD D4
DI7           ->     IR receiver
DO8           ->     Relay
DO9           ->     US Reserved
DO10          ->     US Reserved
DO11          ->     LCD Enable
DO12          ->     LCD RS
*/

/*=======================================================================================
                                    Includes
========================================================================================*/

#include <LiquidCrystal.h>
//#include <RCSwitch.h>
#include <EEPROM.h>
#include <IRremote.h>

/*=======================================================================================
                                    Definitions
========================================================================================*/

#define FLOWSENSORPIN 2
#define relay         8

/*=======================================================================================
                                User Configurarations
========================================================================================*/

LiquidCrystal lcd(12, 11, 6, 5, 4, 3);  // initialize the library with the numbers of the interface pins
//RCSwitch mySwitch = RCSwitch();
IRrecv irrecv(7);
decode_results results;

int address = 1;                        // EEPROM address
int pMode = 0;
int timerVal = 0;
int flowVol = 0;
int heightHigh = 0;
int heightLow = 0;
int timeHigh = 0;
int timeLow = 0;
int loopTimeVal = 0;                        // delay loop
int loopFlowVal = 0;                        // delay loop
bool socketStat = false;                    // socket status ON/OFF
volatile uint16_t pulses = 0;               // count how many pulses
volatile uint8_t lastflowpinstate;          // track the state of the pulse pin
volatile uint32_t lastflowratetimer = 0;    // keep time of how long it is between pulses
volatile float flowrate;                    // to calculate a flow rate

SIGNAL(TIMER0_COMPA_vect) {
  uint8_t x = digitalRead(FLOWSENSORPIN);
  if (x == lastflowpinstate) {
    lastflowratetimer++;
    return; // nothing changed!
  } 
  if (x == HIGH) {
    pulses++; //low to high transition!
  }
  lastflowpinstate = x;
  flowrate = 1000.0;
  flowrate /= lastflowratetimer;  // in hertz
  lastflowratetimer = 0;
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
  }
}

/*=======================================================================================
                                   Setup function
========================================================================================*/
  
void setup() {

  Serial.begin(115200);                     // enable serial
  Serial.println("Init starting...");
  //mySwitch.enableTransmit(8);               // transmitter is connected to Arduino Pin #8
  //mySwitch.enableReceive(0);              // receiver on interrupt 0 => that is pin #2
  irrecv.enableIRIn();                      // start the receiver
  pinMode(LED_BUILTIN, OUTPUT);             // onboard LED as output
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(FLOWSENSORPIN, INPUT);
  pinMode(relay, OUTPUT);                       // relay
  digitalWrite(relay, HIGH);
  digitalWrite(FLOWSENSORPIN, HIGH);
  lastflowpinstate = digitalRead(FLOWSENSORPIN);
  useInterrupt(true);
  
  lcd.begin(16, 2);  //enable LCD display
  Serial.println("LCD display OK");
  versionDelay();
  lcd.clear();

  timerVal = EEPROM.read(address);           // load timerVal from EEPROM
  heightHigh = EEPROM.read(address+1);
  heightLow = EEPROM.read(address+2);
  timeHigh = EEPROM.read(address+3);
  timeLow = EEPROM.read(address+4);
  pMode = EEPROM.read(address+5);
  flowVol = (EEPROM.read(address+6) * 10);
  
  Serial.print("EEPROM timerVal=:");
  Serial.println(timerVal);
  Serial.print("EEPROM flowVol=:");
  Serial.println(flowVol);
  Serial.print("EEPROM heightHigh=:");
  Serial.println(heightHigh);
  Serial.print("EEPROM heightLow=:");
  Serial.println(heightHigh);
  Serial.print("EEPROM timeHigh=:");
  Serial.println(heightHigh);
  Serial.print("EEPROM timeLow=:");
  Serial.println(heightHigh);
  Serial.print("EEPROM mode=:");
  Serial.println(pMode);

  switch (pMode) {
    case 2:
      mode2();
      break;
    case 1:
      mode1();
      break;
    default:
      mode0();
    break;
  }

  // mySwitch.send("110111001000111001000011");  // RF socket OFF
  socketStat = false;
  lcd.setCursor(0,1);
  lcd.print("Socket OFF      ");
  
  Serial.println("Init complete");
  
}

/*=======================================================================================
                                            Loop
========================================================================================*/

void loop() {

  if (irrecv.decode(&results)) {

      Serial.print("Received:");
      Serial.println(results.value);

      //Remote button next (ON)
      if (results.value == 1120 || results.value == 3168 || results.value == 1312 || results.value == 3360)  {

        if (pMode == 0) {
          //mySwitch.send("110111001000111001001011");
          digitalWrite(relay, LOW);
          digitalWrite(LED_BUILTIN, HIGH);
          Serial.println("Socket ON");
          lcd.setCursor(0,1);
          lcd.print("Socket ON       ");
          loopTimeVal = timerVal;
          socketStat = true;
        }
        
        else if (pMode == 1) {
          
        }
        
        else if (pMode == 2) {
          //mySwitch.send("110111001000111001001011");
          digitalWrite(relay, LOW);
          digitalWrite(LED_BUILTIN, HIGH);
          Serial.println("Socket ON");
          lcd.setCursor(0,1);
          //lcd.print("Socket ON       ");
          loopFlowVal = 1;
          pulses = 0;
          socketStat = true;
        }
        
      }

      //Remote button prev. (OFF)
      else if (results.value == 1121 || results.value == 3169 || results.value == 3361 || results.value == 1313)  {

        if (pMode == 0 || pMode == 1 || pMode == 2) {
          //mySwitch.send("110111001000111001000011");
          digitalWrite(relay, HIGH);
          digitalWrite(LED_BUILTIN, LOW);
          Serial.println("Socket OFF");
          lcd.setCursor(0,1);
          lcd.println("Socket OFF      ");
          loopTimeVal = 0;
          loopFlowVal = 0;
          socketStat = false;
        }
      }
        
      //Remote button volume up
      else if (results.value == 1040 || results.value == 3088)  {
      
        if (pMode == 0) {
          timerVal++;

          if (timerVal >= 255)  {
            timerVal = 255;
            Serial.println("Timer value +limit");
          }
          
          lcd.setCursor(6,0);
          lcd.print(timerVal/10.0,1);
          lcd.print("s   ");
          EEPROM.update(address, timerVal);
          Serial.println("Timer value increased and saved");
        }

        else if (pMode == 1) {
          
        }
        
        else if (pMode == 2) {
          flowVol += 10;

          if (flowVol >= 2550)  {
            flowVol = 2550;
            Serial.println("Flow value +limit");
          }
          
          lcd.setCursor(4,0);
          lcd.print(flowVol);
          lcd.print("ml   ");
          EEPROM.update(address+6, flowVol / 10);
          Serial.println("Flow volume value increased and saved");
        }
        
      }
        
      //Remote button volume down
      else if (results.value == 1041 || results.value == 3089)  {

        if (pMode == 0) {
          timerVal--;

          if (timerVal < 0)  {
            timerVal = 0;
            Serial.println("Timer value -limit");
          }
          
          lcd.setCursor(6,0);
          lcd.print(timerVal/10.0,1);
          lcd.print("s   ");
          EEPROM.update(address, timerVal);
          Serial.println("Timer value decreased and saved");
        }

        else if (pMode == 1) {
          
        }
        
        else if (pMode == 2) {
          flowVol -= 10;

          if (flowVol < 0)  {
            flowVol = 0;
            Serial.println("Timer value -limit");
          }
          
          lcd.setCursor(4,0);
          lcd.print(flowVol);
          lcd.print("ml   ");
          EEPROM.update(address+6, flowVol / 10);
          Serial.println("Flow volume value decreased and saved");
        }
        
      }
        
      //Remote button 1
      else if (results.value == 1335 || results.value == 3383)  {
        mode0();
      }

      //Remote button 2
      else if (results.value == 1336 || results.value == 3384)  {
        mode1();
      }

      //Remote button 3
      else if (results.value == 1337 || results.value == 3385)  {
        mode2();
      }
        
    irrecv.resume();  //receive the next value
  }
  
  outputSet();
  
}

/*=======================================================================================
                                         Functions
========================================================================================*/

void versionDelay() {

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Odori d.o.o.  v3");  //display version
  lcd.setCursor(0,1);
  
  for(int i=0; i<16; i++)  {   
    lcd.print(".");
    delay(75);
  }
}

void outputSet()  {

  if (loopTimeVal > 0) {
    Serial.print("delayVal:");
    Serial.println(loopTimeVal);
    lcd.setCursor(9,1);
    lcd.print("(");
    //lcd.setCursor(10,1);
    lcd.print(loopTimeVal/10.0,1);
    //lcd.setCursor(14,1);
    lcd.print(") ");
    loopTimeVal--;
    delay(100);      
  }
       
  else if (loopFlowVal > 0) {
    
    float liters = pulses / 617.0;
   
    Serial.print("Flow volume:");
    Serial.println(liters);
    lcd.setCursor(0,1);

    Serial.print("Flow rate:");
    lcd.print("FR:");
    if (flowrate < 200)  {
      Serial.println(0.0);
      lcd.print(0.0,1);
    }
    else  {
    Serial.println(flowrate);
    lcd.print(flowrate,1);
    }
    
    lcd.print(" FV:");
    lcd.print(liters * 1000,1);

    if (liters * 1000 >= flowVol) {
      loopFlowVal = 0;
    }
    delay(100);
  }

  else if  (socketStat == true) {
    //turn off socket
    //mySwitch.send("110111001000111001000011");
    digitalWrite(relay, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Socket OFF");
    lcd.setCursor(0,1);
    lcd.println("Socket OFF      ");
    socketStat = false;
  }
  
}

void mode0() {

    Serial.println("Mode 0");
    pMode = 0;
    EEPROM.update(address+5, pMode);
    loopFlowVal = 0;
    loopTimeVal = 0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Timer:");
    lcd.print(timerVal/10.0,1);
    lcd.print("s   ");
    lcd.setCursor(14,0);
    lcd.print("M1");
        
}

void mode1() {
  
    Serial.println("Mode 1");
    pMode = 1;
    EEPROM.update(address+5, pMode);
    loopFlowVal = 0;
    loopTimeVal = 0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("HH:    HL:      ");
    lcd.setCursor(14,0);
    lcd.print("M2");
    lcd.setCursor(0,1);
    lcd.print("TH:    TL:      ");
    
}

void mode2() {
  
    Serial.println("Mode 2");
    pMode = 2;
    EEPROM.update(address+5, pMode);
    loopFlowVal = 0;
    loopTimeVal = 0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Vol:");
    lcd.print(flowVol);
    lcd.print("ml   ");
    lcd.setCursor(14,0);
    lcd.print("M3");
    
}

