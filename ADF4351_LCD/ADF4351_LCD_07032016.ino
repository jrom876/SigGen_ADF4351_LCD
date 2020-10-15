//
//   Jacob Romero
//   Creative Engineering Solutions
//   Translation and Modification of Alain Fort's ADF4251 Driver

//   This excellent code by Alain Fort was originally written in French,
//   and needed significant translation and debug before it could actually
//   run in my custom frequency generator and amplifier circuit.
//
//   Contained herein is my rewrite of Alain's code, with all of his comments intact.
//
//   CHEERS!
//
//  ************************************************** ORIGINAL **********************************************************

//   ADF4251 and Arduino
//   By Alain Fort F1CJN feb 2,2016
//   update march 7, 2016 (ROBOT V1.1 and V1.0)
//
//
//  *************************************************** ENGLISH ***********************************************************
//
//  This sketch uses and Arduino uno (5€), a standard "LCD buttons shield" from ROBOT (5€), with buttons and an ADF4351 chinese
//  card found at EBAY (40€). The frequency can be programmed between 34.5 and 4400 MHz.
//  Twenty frequencies can be memorized into the Arduino EEPROM.
//  If one or more frequencies are memorized, then at power on, the memory zero is always selected.
//
//   The cursor can move with LEFT and RIGHT buttons. Then the underlined digit can be modified with the UP and DOWN buttons,
//    for the frequency, the memories and the frequency reference (10 or 25 MHz):
//   - to change the frequency, move the cursor to the digit to be modified, then use the UP and DOWN buttons,
//   - to modify the memory number,move the cursor to the number to be modified, then use the UP and DOWN buttons,
//   - to select the refrence frequence,move the cursor on 10 or 25 and select with UP and DOWN.
//   - to read or write the frequency in memory, place the cursor on the more left/more down position and select REE (for Reading EEprom)
//    or WEE (for Writing EEprom).
//    The cursor dissapears after few seconds and is reactivated if a button is pressed.
//
//   MEMORIZATION
//    - For the frequency, select WEE, then select the memory number, then push the SELECT button for a second. The word MEMORISATION
//    appears on the screen. This memorization works then the cursor is anywhere except on the reference 10 or 25 position.
//    - For the reference frequency, move the cursor to 10 or 25, the press the SELECT for one second.

//  ******************************************** HARDWARE IMPORTANT********************************************************
//  With an Arduino un0 : uses a resistive divider to reduce the voltage, MOSI (pin 11) to
//  ADF DATA, SCK (pin13) to ADF CLK, Select (PIN 3) to ADF LE
//  Resistive divider 560 Ohm with 1000 Ohm to ground on Arduino pins 11, 13 et 3 to adapt from 5V
//  to 3.3V the digital signals DATA, CLK and LE send by the Arduino.
//  Arduino pin 2 (for lock detection) directly connected to ADF4351 card MUXOUT.
//  The ADF card is 5V powered by the ARDUINO (PINs +5V and GND are closed to the Arduino LED).

//************************************************* MANUAL*****************************************************************
//touch LEFT    cursor to the left, cursor to the left
//touch RIGHT   cursor to the right, cursor to the right
//touch UP      frequence or memory increment, increase frequency
//touch DOWN    frequence or memory decrement, decrease frequency
//touch SELECT  long push = frequency memorization into the EE number EEPROM / or reference memorization
//*************************************************************************************************************************
// Warning : if you are using a ROBOT Shied version 1.1, it is necessary to modify the read_lcd_buttons sub routine
// you need to uncomment the 1.1 version and to comment the 1.0 version. See below

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SPI.h>

#define ADF4351_LE 3

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

byte poscursor = 0; //position cursor current 0 to the 15
byte line = 0; // line LCD display being 0 or 1
byte memory,RWtemp; // EEPROM memory number

uint32_t registers[6] =  {0x4580A8, 0x80080C9, 0x4E42, 0x4B3, 0xBC803C, 0x580005} ; // 437 MHz with ref to the 25 MHz
//uint32_t registers[6] =  {0, 0, 0, 0, 0xBC803C, 0x580005} ; // 437 MHz with ref to the 25 MHz
int address,modif=0,WEE=0;
int lcd_key = 0;
int adc_key_in  = 0;
int timer = 0,timer2=0; // used to measureee the contact time on a touch
unsigned int i = 0;


double RFout, REFin, INT, PFDRFout, OutputChannelSpacing, FRACF;
double RFoutMin = 35, RFoutMax = 4400, REFinMax = 250, PDFMax = 32;
unsigned int long RFint,RFintold,INTA,RFcalc,PDRFout, MOD, FRAC;
byte OutputDivider;byte lock=2;
unsigned int long reg0, reg1;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNun    5

//**************************** SP Reading buttons ********************************************
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the buttons
  if (adc_key_in < 790)lcd.blink();

  if (adc_key_in < 50)return btnRIGHT;  // for display ROBOT V1.0
  if (adc_key_in < 195)return btnUP;
  if (adc_key_in < 380)return btnDOWN;
  if (adc_key_in < 555)return btnLEFT;
  if (adc_key_in < 790)return btnSELECT; // end display ROBOT1.0

  //if (adc_key_in < 50)return btnRIGHT; // for display ROBOT 1.1
  //if (adc_key_in < 250)return btnUP;
  //if (adc_key_in < 450)return btnDOWN;
  //if (adc_key_in < 650)return btnLEFT;
  //if (adc_key_in < 850)return btnSELECT; // end display ROBOT 1.1

  return btnNun;  // touch not supported
}

//***************************** SP display frequency on LCD ********************************
void printAll ()
{
  //RFout=1001.10 // test
  lcd.setCursor(0, 0);
  lcd.print("RF = ");
  if (RFint < 100000) lcd.print(" ");
  if (RFint < 10000)  lcd.print(" ");
  lcd.print(RFint/100);lcd.print(".");
  RFcalc=RFint-((RFint/100)*100);
  if (RFcalc<10)lcd.print("0");
  lcd.print(RFcalc);
  lcd.print(" MHz");
  lcd.setCursor(0,1);
  if (WEE==0) {lcd.print("REE=");}
  else {lcd.print("WEE=");}
  if (memory<10)lcd.print(" ");
  lcd.print(memory,DEC);
  if  ((digitalRead(2)==1))lcd.print(" LOCKED ");
  else lcd.print(" NOLOCK ");
  lcd.print(PFDRFout,DEC);
  lcd.setCursor(poscursor,line);
}

void WriteRegister32(const uint32_t value)   //Program records 32bits
{
  digitalWrite(ADF4351_LE, LOW);
  for (int i = 3; i >= 0; i--)          // loop 4 x 8bits
  SPI.transfer((value >> 8 * i) & 0xFF); // shift, masking the byte and sending via SPI
  digitalWrite(ADF4351_LE, HIGH);
  digitalWrite(ADF4351_LE, LOW);
}

void SetADF4351()  // Program all registers of the ADF4351
{ for (int i = 5; i >= 0; i--)  // Programming ADF4351 beginning with R5
    WriteRegister32(registers[i]);
}

// *************** SP writing Mot long (32bits) en EEPROM  entre address et address+3 **************
void EEPROMWritelong(int address, long value)
      {
      // Decompose a long (32 bits) into 4 bytes
      //three = MSB -> four = lsb
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Writing the 4 bytes into the EEPROM memory
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      }

// *************** SP reading long word (32bits) in EEPROM between address and address+3 **************
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Returns the length (32bits) using the shift of 0, 8, 16 and 24 bits and masks
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }
//************************************ Setup ****************************************
void setup() {
  lcd.begin(16, 2); // two 16 characters lines
  lcd.display();
  analogWrite(10,255); //LCD Brightness

  Serial.begin (19200); //  Serial to the PC via Arduino "Serial Monitor"  at 9600
  lcd.print("    GENERATOR   ");
  lcd.setCursor(0, 1);
  lcd.print("    ADF4351     ");
  poscursor = 7; line = 0;
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("   par F1CJN    ");
   delay(1000);

  pinMode(2, INPUT);  // PIN 2 as input for lock
  pinMode(ADF4351_LE, OUTPUT);          // Setup pins
  digitalWrite(ADF4351_LE, HIGH);
  SPI.begin();                          // Init SPI bus
  SPI.setDataMode(SPI_MODE0);           // CPHA = 0 et Clock positive
  SPI.setBitOrder(MSBFIRST);            // strong weight in head

  if (EEPROM.read(100)==55){PFDRFout=EEPROM.read(20*4);} // if the template is written in EEPROM, it is read
  else {PFDRFout=25;}

  if (EEPROM.read(101)==55){RFint=EEPROMReadlong(memory*4);} // if the frequency is written in EEPROM, it is read
  else {RFint=7000;}

  RFintold=1234;//RFintold that is different from RFout when init
  RFout = RFint/100 ; // output frequency
  OutputChannelSpacing = 0.01; // no frequency = 10kHz

  WEE=0;  address=0;
  lcd.blink();
  printAll(); delay(500);


} // End setup

//*************************************Loop***********************************
void loop()
{
  RFout=RFint;
  RFout=RFout/100;
  if ((RFint != RFintold)|| (modif==1)) {
    //Serial.print(RFout,DEC);Serial.print("\r\n");
    if (RFout >= 2200) {
      OutputDivider = 1;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
      bitWrite (registers[4], 3, 1);
      bitWrite (registers[4], 4, 1);
    }
    if (RFout < 2200) {
      OutputDivider = 2;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
      bitWrite (registers[4], 3, 1);
      bitWrite (registers[4], 4, 1);
    }
    if (RFout < 1100) {
      OutputDivider = 4;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
      bitWrite (registers[4], 3, 1);
      bitWrite (registers[4], 4, 1);
    }
    if (RFout < 550)  {
      OutputDivider = 8;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
      bitWrite (registers[4], 3, 1);
      bitWrite (registers[4], 4, 1);
    }
    if (RFout < 275)  {
      OutputDivider = 16;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
      bitWrite (registers[4], 3, 1);
      bitWrite (registers[4], 4, 1);
    }
    if (RFout < 137.5) {
      OutputDivider = 32;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
      bitWrite (registers[4], 3, 1);
      bitWrite (registers[4], 4, 1);
    }
    if (RFout < 68.75) {
      OutputDivider = 64;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
      bitWrite (registers[4], 3, 1);
      bitWrite (registers[4], 4, 1);
    }

    INTA = (RFout * OutputDivider) / PFDRFout;
    MOD = (PFDRFout / OutputChannelSpacing);
    FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
    FRAC = round(FRACF); // It rounds the result

    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;

    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; // add the address "001"
    bitSet (registers[1], 27); // Prescaler sure 8/9

    bitSet (registers[2], 28); // Digital lock == "110" sure b28 b27 b26
    bitSet (registers[2], 27); // digital lock
    bitClear (registers[2], 26); // digital lock

    SetADF4351();  // Program all the records of the ADF4351
    RFintold=RFint;modif=0;
    printAll();  // LCD display
  }

  lcd_key = read_LCD_buttons();  // read the buttons

  switch (lcd_key)               // Select action
  {
    case btnRIGHT: //Straight
      poscursor++; // cursor to the right
      if (line == 0) {
        if (poscursor == 9 ) {
          poscursor = 10;
          line = 0; } //if the cursor
        if (poscursor == 12 ) {
          poscursor = 0; line = 1; }; //si cursor to the right
      }
     if (line == 1) {
        if (poscursor == 1 ) {poscursor = 5; line = 1; } //if cursor is on the number memory
        if (poscursor == 6 ) {poscursor = 15; line = 1; } //if cursor is on the number memory
        if (poscursor==16) {poscursor=5; line=0;};
      }
      //Serial.print (" RIGHT Button\r\n");
      lcd.setCursor(poscursor, line);
      break;

    case btnLEFT: //left
      poscursor--; // horizontal cursor
      if (line == 0) {
        if (poscursor == 4) {poscursor = 15; line = 1;  };
        if (poscursor == 9) {   poscursor = 8; line=0;}
      }
       if(line==1){
          if (poscursor==255) {poscursor=11; line=0;};
          if (poscursor==4) {poscursor=0; line=1;};
          if (poscursor==14) {poscursor=5; line=1;};
      }
      //Serial.print(poscursor,DEC);
      lcd.setCursor(poscursor, line);
      break;

    case btnUP: //Top
      if (line == 0)
      { // RFoutfrequency
        //Serial.print(oldRFint,DEC);
        if (poscursor == 5) RFint = RFint + 100000 ;
        if (poscursor == 6) RFint = RFint + 10000 ;
        if (poscursor == 7) RFint = RFint + 1000 ;
        if (poscursor == 8) RFint = RFint + 100 ;
        if (poscursor == 10) RFint = RFint + 10 ;
        if (poscursor == 11) RFint = RFint + 1 ;
        if (RFint > 440000)RFint = RFintold;
        //Serial.print(RFint,DEC);
        //Serial.print("  \r\n");
      }
      if (line == 1)
      {
        if (poscursor == 5){ memory++;
        if (memory==20)memory=0;
        if (WEE==0){RFint=EEPROMReadlong(memory*4); // read EEPROM and display
           if (RFint>440000) RFint=440000;
           }
        }
        if (poscursor==15){
        if( PFDRFout==10){PFDRFout=25;} //controls FREF
        else if ( PFDRFout==25){PFDRFout=10;}
        else PFDRFout=25;// in case PFDRF is different from 10 and 25
        modif=1;  }

      if( (poscursor==0) && (WEE==1))WEE=0;
      else if ((poscursor==0) && (WEE==0))WEE=1;
      }
        printAll();
      break; // fin button up

    case btnDOWN: //Bottom
      if (line == 0) {
        if (poscursor == 5) RFint = RFint - 100000 ;
        if (poscursor == 6) RFint = RFint - 10000 ;
        if (poscursor == 7) RFint = RFint - 1000 ;
        if (poscursor == 8) RFint = RFint - 100 ;
        if (poscursor == 10) RFint = RFint - 10 ;
        if (poscursor == 11) RFint = RFint - 1 ;
        if (RFint < 3450) RFint = RFintold;
        if (RFint > 440000)  RFint = RFintold;
        break;
      }

     if (line == 1)
      {
        if (poscursor == 5){memory--;
          if (memory==255)memory=19;
          if (WEE==0){RFint=EEPROMReadlong(memory*4); // read EEPROM and display
            if (RFint>440000) RFint=440000;
          // Serial.print(RFint,DEC);
           }
        } // fin poscursor =5

       if (poscursor==15){
       if( PFDRFout==10){PFDRFout=25;} //controls FREF
       else if ( PFDRFout==25){PFDRFout=10;}
       else PFDRFout=25;// in case PFDRF is different from 10 and 25
       modif=1;
       }

       if( (poscursor==0) && (WEE==1))WEE=0;
       else if ((poscursor==0)&&(WEE==0))WEE=1;

       printAll();
      // Serial.print (" DOWN Button  \r\n");
      break; // fin button bas
      }

    case btnSELECT:
      do {
        adc_key_in = analogRead(0);      // Test release button
        delay(1); timer2++;        // timer increment every one ms
        if (timer2 > 600) { //waiting 600 millisecondes
         if (WEE==1 || poscursor==15){
         if (line==1 && poscursor==15){ EEPROMWritelong(20*4,PFDRFout);EEPROM.write(100,55);} // writing FREF
         else if (WEE==1) {EEPROMWritelong(memory*4,RFint);EEPROM.write(101,55);}// writing RF to EEPROM at address (memory*4)
          lcd.setCursor(0,1); lcd.print("  MEMORISATION  ");}
          lcd.setCursor(poscursor,line);
          delay(500);timer2=0;
          printAll();
        }; // mes

        }
      while (adc_key_in < 900); // waiting loosening
      break;  // Fin button Select

     case btnNun: {
        break;
      };
      break;
  }// Fin LCD keys

   do { adc_key_in = analogRead(0); delay(1);} while (adc_key_in < 900); // waiting loosening touch
   delay (10);timer++; // inc timer
   //Serial.print(timer,DEC);
   if (timer>1000){lcd.noBlink();timer=0;} // cursor off

}   // fin loop
