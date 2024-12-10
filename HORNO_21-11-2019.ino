/* Electric kiln controller v1.0
 *   by Steve Turner (arduinokiln@gmail.com)
 *   See ReadMe.txt
 *   
 *   OJO, con alimentacion de 5 volts en la entrada jack de alimentacion no reconoce la memoria SD
 */

#include <max6675.h>        // libreria placa termocoupla
#include <PID_v1.h>         // libreria control temperatura PID
#include <SPI.h>            // Serial Peripheral Interface library
#include <SD.h>             // libreria memoria SD (se requiere SPI)
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>             


// Setup user variables (CHANGE THESE TO MATCH YOUR SETUP)
const int lcdRefresh = 1930;           // Frecuencia de actualización de pantalla cuando se ejecuta (ms)
const int maxTemp = 1600;              // Maxima temperatura, si se alcanza se apagará!
const int numZones = 1;                // Numero de zonas (maximo 3)
const int pidCycle = 2500;             // Time for a complete PID on/off cycle for the heating elements (ms)
double pidInput[numZones];             // Input array for PID loop (actual temp reading from thermocouple).  NO CAMBIAR!
double pidOutput[numZones];            // Output array for PID loop (relay for heater).                      NO CAMBIAR!
double pidSetPoint[numZones];          // Setpoint array for PID loop (temp you are trying to reach).        NO CAMBIAR!
PID pidCont[numZones] = {PID(&pidInput[0], &pidOutput[0], &pidSetPoint[0], 800, 47.37, 4.93, DIRECT)};  // PID controller array for each zone.  Set arguments 4/5/6 to the Kp, Ki, Kd values after tuning.
const long saveCycle = 5000;          // Con que frecuencia se guardan las temperaturas (ms)    eran 15000
const int tempOffset[numZones] = {0};  // Array to add a temp offset for each zone (degrees).  Use if you have a cold zone in your kiln or if your thermocouple reading is off.  This gets added to the setpoint.
const int tempRange = 2;               // This is how close the temp reading needs to be to the set point to shift to the hold phase (degrees).  Set to zero or a positive integer.
const char tempScale = 'C';            //  F = Fahrenheit.  C = Celsius
signed short minutes, secondes;
char timeline[16];
// Setup pin connections (CHANGE THESE TO MATCH YOUR SETUP)
                                                // PINES 0 y 1 son usados para el puerto serie
                                                
const int upPin =     2;                        // BOTON SUBIR
const int downPin =   3;                        // BOTON BAJAR
const int selectPin = 4;                        // BOTON SELECCIONAR/COMENZAR
MAX6675 thermo[numZones] = {MAX6675(5, 6 ,7)};  // Pines conectados a la placa de la termocupla. Esta es una matriz para cada termopar (zona).
                                                //  5   SCK
                                                //  6   CS
                                                //  7   SO
                                                
const int heaterPin[numZones] = {9};            // PIN QUE VA AL RELE

// PINES 10 al 13 son para la placa lectora de SD 
                                                //  10  CS    (CS)
                                                //  11  MOSI  (D1)
                                                //  12  MISO  (D0)
                                                //  13  SCK   (CLK)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
LiquidCrystal_I2C lcd(0x27,20,4);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


 
// OTRAS VARIABLES   NO CAMBIAR!!!!
double calcSetPoint;        // Calculated set point (degrees)
unsigned long holdStart;    // Exact time the hold phase of the segment started (ms).  Based on millis().
int i = 0;                  // Simple loop counter
int lastSeg = 0;            // Last segment number in firing schedule
int lastTemp;               // Last setpoint temperature (degrees)
unsigned long lcdStart;     // Exact time you refreshed the lcd screen (ms).  Based on millis().
int optionNum = 1;          // Option selected from screen #3
unsigned long pidStart;     // Exact time you started the new PID cycle (ms).  Based on millis().
double rampHours;           // Time it has spent in ramp (hours)
unsigned long rampStart;    // Exact time the ramp phase of the segment started (ms).  Based on millis().
unsigned long saveStart;    // Exact time you saved the temps (ms).  Based on millis().
File saveFile;              // File to save temps / set point.
char schedDesc1[21];        // Schedule description #1 (first line of text file)
int schedNum = 1;           // Current firing schedule number.  This ties to the file name (ex: 1.txt, 2.txt).
boolean schedOK = false;    // Is the schedule you loaded OK?
unsigned long schedStart;   // Exact time you started running the schedule (ms).  Based on millis().
int screenNum = 1;          // Screen number displayed during firing (1 = temps / 2 = schedule info / 3 = tools / 4 = done
int segHold[20];            // Hold time for each segment (min).  This starts after it reaches target temp.
int segNum = 0;             // Current segment number running in firing schedule.  0 means a schedule hasn't been selected yet.
boolean segPhase = 0;       // Current segment phase.  0 = ramp.  1 = hold.
int segRamp[20];            // Rate of temp change for each segment (deg/hr).
int segTemp[20];            // Target temp for each segment (degrees).

//******************************************************************************************************************************
//  CONFIGURACIÓN INICIAL (CORRE UNA VEZ DURANTE EL ARRANQUE)
//******************************************************************************************************************************
void setup() {

  // Setup all pin modes on board.  Remove INPUT_PULLUP if you have resistors in your wiring.
  pinMode(upPin,     INPUT_PULLUP);
  pinMode(downPin,   INPUT_PULLUP);
  pinMode(selectPin, INPUT_PULLUP); 
  for (i = 0; i < numZones; i++) {
    pinMode(heaterPin[i], OUTPUT);
  }
  for (i = 14; i <= 19; i++) {  // Change analog inputs to digital outputs for LCD screen
    pinMode(i, OUTPUT);
  }
  lcd.init();                      // initialize the lcd 
  lcd.init(); 
  lcd.backlight();
  // Setup lcd display (20 columns x 4 rows)


  if (SD.begin() == false) {
    lcd.print(F("       ERROR:"));
    lcd.setCursor(0, 2);
    lcd.print(F("No se encuentra la "));
    lcd.setCursor(0, 3);
    lcd.print(F("memoria microSD     "));
    shutDown();
  }

  // Delete old save file
  SD.remove("temps.txt");

  // Intro screens (i'm bored / delete if you want)
  lcd.print(F(" HORNO DE CERAMICA  "));
  lcd.setCursor(0, 1);
  lcd.print(F("----- V: 1.9 -------"));
  lcd.setCursor(0, 3);  
  lcd.print(F("    21/11/2019      "));
  delay(4000);


  // Open firing shedule # 1
  openSched();

}

//******************************************************************************************************************************
//  LOOP: MAIN LOOP (CONTINUOUS)
//******************************************************************************************************************************
void loop() {

  //******************************
  // Shutdown if too hot
  for (i = 0; i < numZones; i++) {
    if (pidInput[i] >= maxTemp) {
      lcd.clear();
      lcd.print(F("       ERROR:"));
      lcd.setCursor(2, 2);
 //   lcd.print(F("Max temp reached"));
      lcd.print(F("Max temp alcanzada"));
      lcd.setCursor(0, 3);
 //   lcd.print(F("System was shut down"));    
      lcd.print(F("Sistema apagado     "));  
      shutDown();
    }
  }

  //******************************
  // Select a firing schedule
  if (segNum == 0) {
    
    // Up arrow button
    if (digitalRead(upPin) == LOW && schedNum > 1) {
      schedNum = schedNum - 1;
      openSched();
      btnBounce(upPin);
    }
    
    // Down arrow button
    if (digitalRead(downPin) == LOW) {
      schedNum = schedNum + 1;
      openSched();
      btnBounce(downPin);          
    }
    
    // Select / Start button
    if (digitalRead(selectPin) == LOW && schedOK == true) {
      setupPIDs();
      segNum = 1;
      lcdStart = millis();
      pidStart = millis();
      rampStart = millis();
      schedStart = millis();
      updateLCD();
      btnBounce(selectPin);      
    }
  }

  //******************************
  // Running the firing schedule
  if (segNum >= 1) {

    // Up arrow button
    if (digitalRead(upPin) == LOW) {
      if (screenNum == 2 || (screenNum == 3 && optionNum == 1)) {
        screenNum = screenNum - 1;
      }
      else if (screenNum == 3 && optionNum >= 2) {
        optionNum = optionNum - 1;
      }
      updateLCD();
      btnBounce(upPin); 
    }
    
    // Down arrow button
    if (digitalRead(downPin) == LOW) {
      if (screenNum <= 2) {
        screenNum = screenNum + 1;
      }
      else if (screenNum == 3 && optionNum <= 2) {
        optionNum = optionNum + 1;
      }
      updateLCD();
      btnBounce(downPin); 
    }

    // Select / Start button
    if (digitalRead(selectPin) == LOW && screenNum == 3) {
      if (optionNum == 1) {  // Add 5 min
        segHold[segNum - 1] = segHold[segNum - 1] + 5;
        optionNum = 1;
        screenNum = 2;
      }

      if (optionNum == 2) {  // Add 5 deg
        segTemp[segNum - 1] = segTemp[segNum - 1] + 5;
        optionNum = 1;
        screenNum = 1;
      }

      if (optionNum == 3) {  // Goto next segment
        segNum = segNum + 1;
        optionNum = 1;
        screenNum = 2;
      }

      updateLCD();
      btnBounce(selectPin);           
    }
 
    // Update PID's / turn on heaters / update segment info
    if (screenNum < 4) {
      if (millis() - pidStart >= pidCycle) {
        pidStart = millis();
        updatePIDs();
      }
      htrControl();
      updateSeg();
    }

    // Refresh the LCD
    if (millis() - lcdStart >= lcdRefresh) {
      updateLCD();
      lcdStart = millis();
    }

    // Save the temps to a file on SD card
    if (millis() - saveStart >= saveCycle && pidInput[0] > 0 && screenNum < 4) {                  //////////////////////////////////////////////////CORRECCION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      saveFile = SD.open("temps.txt", FILE_WRITE);
        saveFile.print((millis() - schedStart) / 60000.0); // Save in minutes
        for (i = 0; i < numZones; i++) {
          saveFile.print(",");
          saveFile.print(pidInput[i]);
        }
        saveFile.print(",");
        saveFile.println(pidSetPoint[0]);
      saveFile.close();

      saveStart = millis();
    }  
    
  }

}

//******************************************************************************************************************************
//  BTNBOUNCE: HOLD UNTIL BUTTON IS RELEASED.  DELAY FOR ANY BOUNCE
//******************************************************************************************************************************
void btnBounce(int btnPin) {

  while (digitalRead(btnPin) == LOW);
  delay(40);
  
}

//******************************************************************************************************************************
//  HTRCONTROL: TURN HEATERS ON OR OFF
//******************************************************************************************************************************
void htrControl() {

  // Loop thru all zones
  for (i = 0; i < numZones; i++) {
    if (pidOutput[i] >= millis() - pidStart) {
      digitalWrite(heaterPin[i], HIGH);
    }
    else {
      digitalWrite(heaterPin[i], LOW);      
    }
  }
  
}

//******************************************************************************************************************************
//  INTLENGTH: GET THE LENGTH OF A INTEGER
//******************************************************************************************************************************
int intLength(int myInt) {

  myInt = abs(myInt);

  if (myInt >= 10000) {
    return 5;
  }
  else if (myInt >= 1000) {
    return 4;
  }
  else if (myInt >= 100) {
    return 3;
  }
  else if (myInt >= 10) {
    return 2;
  }
  else {
    return 1;
  }
  
}

//******************************************************************************************************************************
//  OPENSCHED: OPEN AND LOAD A FIRING SCHEDULE FILE / DISPLAY ON SCREEN
//******************************************************************************************************************************
void openSched() {

  // Setup all variables
  int col = 1;          // Column number (of text file).  First column is one.
  int row = 1;          // Row number (of text file).  First row is one.
  char tempChar;        // Temporary character holder (read one at a time from file)
  char tempLine[21];    // Temporary character array holder
  int tempLoc = 0;      // Current location of next character to place in tempLine array
  char schedDesc2[21];  // Schedule description #2 (second line of text file)
  char schedDesc3[21];  // Schedule description #3 (third line of text file)  

  // Clear the arrays
  memset(schedDesc1, 0, sizeof(schedDesc1));
  memset(segRamp, 0, sizeof(segRamp));
  memset(segTemp, 0, sizeof(segTemp)); 
  memset(segHold, 0, sizeof(segHold));
  
  // Make sure you can open the file
  sprintf(tempLine, "%d.txt", schedNum);
  File myFile = SD.open(tempLine, FILE_READ);
  
  if (myFile == false) {
    lcd.clear();
    lcd.print(F("ELEGIR PROGRAMA: "));
 // lcd.print(F("SELECT SCHEDULE: "));
    lcd.print(schedNum);
    lcd.setCursor(0, 2);
    lcd.print(F("Programas faltantes ")); 
 // lcd.print(F("Can't find/open file"));
    schedOK = false;
    return;
  }
 
   // Load the data
  while (myFile.available() > 0) {

    // Read a single character
    tempChar = myFile.read();

    if (tempChar == 13) {       // Carriage return: Read another char (it is always a line feed / 10).  Add null to end.
      myFile.read();
      tempLine[tempLoc] = '\0';
    }
    else if (tempChar == 44) {  // Comma: Add null to end.
      tempLine[tempLoc] = '\0';      
    }
    else if (tempLoc <= 19) {   // Add it to the temp line array
      tempLine[tempLoc] = tempChar;
      tempLoc = tempLoc + 1; 
    }

    if (row == 1 && tempChar == 13) {
      memcpy(schedDesc1, tempLine, 21);
    }
    else if (row == 2 && tempChar == 13) {
      memcpy(schedDesc2, tempLine, 21);      
    }
    else if (row == 3 && tempChar == 13) {
      memcpy(schedDesc3, tempLine, 21);      
    }
    else if (row >= 4 && col == 1 && tempChar == 44) {
      segRamp[row - 4] = atoi(tempLine);
    }
    else if (row >= 4 && col == 2 && tempChar == 44) {
      segTemp[row - 4] = atoi(tempLine);
    }
    else if ((row >= 4 && col == 3 && tempChar == 13) || myFile.available() == 0) {
      segHold[row - 4] = atoi(tempLine);
    }

    if (tempChar == 13) {  // End of line.  Reset everything and goto next line
      memset(tempLine, 0, 21);
      tempLoc = 0;
      row = row + 1;
      col = 1;
    }

    if (tempChar == 44) {  // Comma.  Reset everything and goto 1st column
      memset(tempLine, 0, 21);
      tempLoc = 0;
      col = col + 1;
    }
    
  }  // end of while(myFile.available ...

  // Close the file
  myFile.close();

  // Set some variables
  lastSeg = row - 3;
  schedOK = true;

  // Fix Ramp values so it will show the correct sign (+/-).  This will help to determine when to start hold.
  for (i = 0; i < lastSeg; i++) {
    segRamp[i] = abs(segRamp[i]);
    if (i >= 1) {
      if (segTemp[i] < segTemp[i - 1]) {
        segRamp[i] = -segRamp[i];
      }
    }
  } 

  // Display on the screen
  lcd.clear();
  lcd.print(F("ELEGIR PROGRAMA: "));
//lcd.print(F("SELECT SCHEDULE: "));
  lcd.print(schedNum);
  lcd.setCursor(0, 1);
  lcd.print(schedDesc1);
  lcd.setCursor(0, 2);
  lcd.print(schedDesc2);    
  lcd.setCursor(0, 3);
  lcd.print(schedDesc3);

  // Cut down schedule description so it shows better on other screen when running
  schedDesc1[14 - intLength(schedNum)] = '\0';

}

//******************************************************************************************************************************
//  READTEMPS: Read the temperatures
//******************************************************************************************************************************
void readTemps() {

  // Loop thru all zones
  for (i = 0; i < numZones; i++) {
    if (tempScale == 'C') {
      pidInput[i] = thermo[i].readCelsius();
    }
    if (tempScale == 'F') {
      pidInput[i] = thermo[i].readFahrenheit();
    }    
  }

}

//******************************************************************************************************************************
//  SETUPPIDS: INITIALIZE THE PID LOOPS
//******************************************************************************************************************************
void setupPIDs() {

  for (i = 0; i < numZones; i++) {
    pidCont[i].SetSampleTime(pidCycle);
    pidCont[i].SetOutputLimits(0, pidCycle);
    pidCont[i].SetMode(AUTOMATIC);
  }

}

//******************************************************************************************************************************
//  SHUTDOWN: SHUT DOWN SYSTEM
//******************************************************************************************************************************
void shutDown() {

  // Turn off all zones (heating element relays)
  for (i = 0; i < numZones; i++) {
    digitalWrite(heaterPin[i], LOW);
  }
  
  // Disable interrupts / Infinite loop
  cli();
  while (1);
  
}

//******************************************************************************************************************************
//  UPDATELCD: UPDATE THE LCD SCREEN WHEN RUNNING
//******************************************************************************************************************************
void updateLCD() {

  // Clear screen and set cursor to top left
  lcd.clear();


  // Temperatures
  if (screenNum == 1) {
    
     
    for (i = 0; i < numZones; i++) {
  lcd.setCursor(0,0);
  lcd.print(F("Temp. actual: "));
    lcd.setCursor(14,0);
    lcd.print((int)pidInput[i]);
    lcd.print((char)223);
    lcd.print("C");
lcd.setCursor(0,1);
    lcd.print(F("Temp. rampa: "));
    lcd.setCursor(14,1);
    lcd.print((int)pidSetPoint[i]);
    lcd.print((char)223);
    lcd.print("C");

  lcd.setCursor(3,3);
    lcd.print("Zona 1");

      

 sprintf(timeline,"%0.2d min %0.2d seg", minutes, secondes);
lcd.setCursor(0,3);
  lcd.print(timeline);
  

  secondes=secondes+2;
  
  if (secondes == 60)
  {
    secondes = 0;
    minutes ++;
  }


   
   
    }
  }

  // Schedule / segment info
  if (screenNum == 2) {
  //lcd.print(F("SCH "));
    lcd.print(F("PRG "));
    lcd.print(schedNum);
    lcd.print(F(": "));
    lcd.print(schedDesc1);

    lcd.setCursor(0, 1);
    lcd.print(F("SEG: "));
    lcd.print(segNum);
    lcd.print(F(" / "));
    lcd.print(lastSeg);
 
    if (segPhase == 0) {
      lcd.setCursor(2, 2);
  //  lcd.print(F("Ramp to "));
      lcd.print(F("Rampa a "));
      lcd.print(segTemp[segNum - 1]);
      lcd.print(F(" "));
      lcd.print((char)223);
      lcd.print(tempScale);

      lcd.setCursor(2, 3);
  //  lcd.print(F("at "));
      lcd.print(F("a: "));
      lcd.print(segRamp[segNum - 1]);
      lcd.print(F(" "));
      lcd.print((char)223);
      lcd.print(F("/hr"));
    }
    else {
      lcd.setCursor(2, 2);
      lcd.print(F("Hold at "));
      lcd.print(segTemp[segNum - 1]);
      lcd.print(F(" "));
      lcd.print((char)223);
      lcd.print(tempScale);
      
      lcd.setCursor(2, 3);
      lcd.print(F("for "));
      lcd.print((millis() - holdStart) / 60000);
      lcd.print(F(" / "));
      lcd.print(segHold[segNum - 1]);
      lcd.print(F(" min"));
    }
  }

  // Tools
  if (screenNum == 3) {
//  lcd.print(F("       TOOLS:"));
    lcd.print(F("   CONFIGURACION: "));
    lcd.setCursor(2, 1);
//  lcd.print(F("Add 5 min"));
    lcd.print(F("Mas 5 min"));
    lcd.setCursor(2, 2);
//  lcd.print(F("Increase 5 deg"));
    lcd.print(F("Sumar 5 grados"));    
    lcd.setCursor(2, 3);
//  lcd.print(F("Skip to next seg"));
    lcd.print(F("Ir al siguiente "));
    lcd.setCursor(0, optionNum);
    lcd.print(F(">"));
    lcd.setCursor(19, optionNum);
    lcd.print(F("<"));
  }
  
  // Schedule completed
  if (screenNum == 4) {
    readTemps();
    
//  lcd.print(F(" SCHEDULE COMPLETE"));
    lcd.print(F(" PROGRAMA  COMPLETO")); 
    lcd.setCursor(0, 1);
//  lcd.print(F("Wait until cool"));
    lcd.print(F("Espere a que enfrie"));
    lcd.setCursor(0, 2);
 //   lcd.print(F("before you open"));
    lcd.print(F(" luego puede abrir "));
    lcd.setCursor(2, 3);
 // lcd.print(F("Zone 1: "));
    lcd.print(F("Zona 1: "));
    lcd.print((int)pidInput[0]);
    lcd.print(F(" "));
    lcd.print((char)223);
    lcd.print(tempScale);
  }

}

//******************************************************************************************************************************
//  UPDATEPIDS: UPDATE THE PID LOOPS
//******************************************************************************************************************************
void updatePIDs() {

  // Get the last target temperature
  if (segNum == 1) {  // Set to room temperature for first segment
    if (tempScale == 'C') {
      lastTemp = 24;
    }
    if (tempScale == 'F') {
      lastTemp = 75;
    }
  }
  else {
    lastTemp = segTemp[segNum - 2];
  }

  // Calculate the new setpoint value.  Don't set above / below target temp  
  if (segPhase == 0) {
    rampHours = (millis() - rampStart) / 3600000.0;
    calcSetPoint = lastTemp + (segRamp[segNum - 1] * rampHours);  // Ramp
    if (segRamp[segNum - 1] >= 0 && calcSetPoint >= segTemp[segNum - 1]) {
      calcSetPoint = segTemp[segNum - 1];
    }
    if (segRamp[segNum - 1] < 0 && calcSetPoint <= segTemp[segNum - 1]) {
      calcSetPoint = segTemp[segNum - 1];
    }
  }
  else {
    calcSetPoint = segTemp[segNum - 1];  // Hold
  }

  // Read the temperatures
  readTemps();
  
  // Loop thru all PID controllers
  for (i = 0; i < numZones; i++) {

    // Set the target temp.  Add any offset.
    pidSetPoint[i] = calcSetPoint + tempOffset[i];

    // Update the PID based on new variables
    pidCont[i].Compute();

  }

}

//******************************************************************************************************************************
//  UPDATESEG: UPDATE THE PHASE AND SEGMENT
//******************************************************************************************************************************
void updateSeg() {

  // Start the hold phase
  if ((segPhase == 0 && segRamp[segNum - 1] < 0 && pidInput[0] <= (segTemp[segNum - 1] + tempRange)) || 
      (segPhase == 0 && segRamp[segNum - 1] >= 0 && pidInput[0] >= (segTemp[segNum - 1] - tempRange))) {
    segPhase = 1;
    holdStart = millis();
  }

  // Go to the next segment
  if (segPhase == 1 && millis() - holdStart >= segHold[segNum - 1] * 60000) {
    segNum = segNum + 1;
    segPhase = 0;
    rampStart = millis();
  }

  // Check if complete / turn off all zones
  if (segNum - 1 > lastSeg) {
    for (i = 0; i < numZones; i++) {
      digitalWrite(heaterPin[i], LOW);
    }
    screenNum = 4;
  }

}
