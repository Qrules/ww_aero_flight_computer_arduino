//--------------------------------------------------------------------
//
// This program 
//
// I/O:
// Serial(0):  Output to OpenLog logger board
// SoftwareSerial (2): GPS input
//--------------------------------------------------------------------

//--------------------------------------------------------------------
// INCLUDES
//--------------------------------------------------------------------
#include "SoftwareSerial.h"

//--------------------------------------------------------------------
// DEFINES
//--------------------------------------------------------------------
// #define SIMULATING 1
// #define DEBUG 1 

//--------------------------------------------------------------------
// CONSTANTS
//--------------------------------------------------------------------
#define GPS_RECV_PIN 2     // from Byonics GPS receiver
#define LED_PIN 13
#define LOGGER_BAUD_RATE 9600
#define LOGGER_ESCAPE_CHAR '$'
#define LOGGER_MAX_CHAR_READ_TRIES 100
#define BIG_BUFFER_SIZE 200
#define LITTLE_BUFFER_SIZE 20
#define MINIMUM_GPS_SENTENCE_LENGTH 20 // Arbitrarily chosen
#define AUTOMATIC_AIRPLANE_RELEASE_ALTITUDE_FT 91000

#ifdef SIMULATING
  #define SIMULATED_CLIMBING_ALTITUDE_DELTA 5000.0f // 1000.0f
  #define SIMULATED_GROUND_ALTITUDE_FT 900.0f
#endif

//--------------------------------------------------------------------
// VARIABLES
//--------------------------------------------------------------------
SoftwareSerial gps(GPS_RECV_PIN, 0, true); // 
long logSequenceNum = 1;
char bigBuffer[BIG_BUFFER_SIZE];
char littleBuffer[LITTLE_BUFFER_SIZE];
int gpsFixQuality = 0;
float altitudeFt = 0.0f;
boolean releaseAirplane = false;
boolean airplaneReleased = false;
float maxAltitudeFt = -999.0f;
long numGpsLinesRead = 0;
char gpsTime[7];

#ifdef SIMULATING
  long runTimeMsec = 0;
  int gpsDelayMsec = 500;
  float simulatedAltitude = SIMULATED_GROUND_ALTITUDE_FT;
  float altitudeDelta = SIMULATED_CLIMBING_ALTITUDE_DELTA;
  boolean fakeNoFix = true;
#endif


//--------------------------------------------------------------------
// MAIN PROGRAM
//--------------------------------------------------------------------
void loop()
{
#ifdef DEBUG      
   Serial.println("Waiting on next GPS sentence...");
#endif

#ifdef SIMULATING  
  if (fakeNoFix && logSequenceNum > 50)
  {
    WriteLoggerLine("Fake no fix done, simulated fixed GPS from here on out.");
    fakeNoFix = false;
  }
  
  if (fakeNoFix)
    WriteLoggerLine("A fake no GPS fix is in effect for a few reads.");
#endif

  GetNextGpsLine();
  
  // TODO:  Log more rapidly at first, then slow the rate greatly when below 2000'.
  //        This will reduce the chance of powering off the logger mid-write.
  WriteLoggerLine(bigBuffer);
  
#ifdef DEBUG      
  Serial.print("FREE RAM: ");
  Serial.println(GetAvailableMemory());
#endif

#ifdef LIGHT_DEBUG
  Serial.println(bigBuffer);
#endif
  
#ifdef DEBUG
#ifdef SIMULATING  
  Serial.println("GPS Sentence: ");
  Serial.println(bigBuffer);
#endif
#endif
 
  if (!strncmp(bigBuffer, "$GPGGA", 6))
  {
#ifdef DEBUG
#ifdef SIMULATING  
    Serial.print("In simulation mode.  Simulated altitude now = ");
    Serial.print(altitudeFt);
    Serial.println(" ft.");
#endif
#endif
    
    GetGpsFixQuality();
    GetGpsTime();
    
#ifdef DEBUG
  Serial.println("GOT A GPS FIX via $GPGGA!!");
#endif
  }
  
  if (gpsFixQuality >= 1)
    ProcessGpsSentence(); 
  
  if (releaseAirplane && !airplaneReleased)
    ReleaseAirplane();
}

//--------------------------------------------------------------------
// INITIALIZATION
//--------------------------------------------------------------------
void setup()
{
  Serial.begin(115200); // for debugging
  gps.begin(4800); // Use Soft Serial object to talk read GPS
  pinMode(LED_PIN, OUTPUT); 
  delay(3000);
  
#ifdef DEBUG
  delay(1000);  
  Serial.println("Writing header..."); 
#endif

  WriteLoggerLine("Flight computer powered up, ready to go.");
  
#ifdef SIMULATING  
  WriteLoggerLine("****** SIMULATION MODE!!! *****");
  WriteLoggerLine("****** SIMULATION MODE!!! *****");
  WriteLoggerLine("****** SIMULATION MODE!!! *****");
#endif

#ifdef DEBUG
  Serial.println("done"); 
#endif
}

//====================================================================
//====================================================================
//                         GPS METHODS
//====================================================================
//====================================================================
//--------------------------------------------------------------------
// ProcessGpsSentence - assumes it's in bigBuffer
//--------------------------------------------------------------------
void ProcessGpsSentence()
{
  int sentenceLen = strlen(bigBuffer);
  
  if  (sentenceLen < MINIMUM_GPS_SENTENCE_LENGTH)
    return;
    
  if (bigBuffer[sentenceLen - 3] != '*')
  {
#ifdef DEBUG      
     Serial.println("GPS SENTENCE DOESN'T HAVE THE ASTERISK NEAR THE END.  IGNORING");
#endif
     return;
  }
  
  if (!strncmp(bigBuffer, "$GPGGA", 6))
    ProcessGpggaSentence();
}

//--------------------------------------------------------------------
// ProcessGpggaSentence
//--------------------------------------------------------------------
void ProcessGpggaSentence()
{
  GetGpsTime();
  GetNthGpsParam(10); // Altitude in meters
  float altitudeM;
     
  if (charsAreNumeric(littleBuffer))
  {
#ifdef SIMULATING
    altitudeFt = simulatedAltitude;
    
    if (altitudeFt > 95000.0)  // TODO:  make a constant out of apogee
      altitudeDelta *= -1.0;
    
    simulatedAltitude += altitudeDelta; 
   
    if (simulatedAltitude < SIMULATED_GROUND_ALTITUDE_FT)
      simulatedAltitude = SIMULATED_GROUND_ALTITUDE_FT;
      
#else
    altitudeM = atof(littleBuffer);
    altitudeFt = altitudeM * 3.281f;
#endif
  }
  
  //////////////////////////////////////////////////////////////////////////////////////////
  // Here is where you want to perform any operations dependent upon altitude.
  // Flags set here are picked up on in the main loop.
  //////////////////////////////////////////////////////////////////////////////////////////
  if (altitudeFt > maxAltitudeFt)
    maxAltitudeFt = altitudeFt;
     
  // Check if we neede to drop the plane
  if (!airplaneReleased && altitudeFt >= AUTOMATIC_AIRPLANE_RELEASE_ALTITUDE_FT)
    releaseAirplane = true; 
     
  /*
  // TODO:  REMOVE THIS FOR RECORD BREAKER!  WE DON'T WANT TO RELEASE OUR GOOD AIRPLANE
  //        IF WE DON'T GET ABOVE THE CURRENT RECORD!
  if (!airplaneReleased && altitudeFt < (maxAltitudeFt - 20000)) // Make sure the balloon has popped for sure
    releaseAirplane = true; 
  */
  
#ifdef DEBUG      
  Serial.print("altM: ");
  Serial.println(altitudeM);
  Serial.print("altFt: ");
  Serial.println(altitudeFt);
#endif
}

//--------------------------------------------------------------------
// GetNthGpsParam - assumes 'valid' GPS sentence in bigBuffer, 
//                  returns with data field in littleBuffer
//--------------------------------------------------------------------
void GetNthGpsParam(int fieldNum)
{
  if (strlen(bigBuffer) == 0)
  {
    littleBuffer[0] = '\0';
    return; 
  }
    
  int currentFieldNum = 1;
  int currentPosn = 0;
  
  while (currentFieldNum < fieldNum)
  {
    do
    {
      currentPosn++;
    } 
    while (currentPosn < strlen(bigBuffer) && bigBuffer[currentPosn] != ',');
    currentFieldNum++; 
  }

  if (currentPosn >= strlen(bigBuffer))
  {
#ifdef DEBUG      
    Serial.print("      Terminating at -->0");
    Serial.print("   ");
    Serial.println(currentPosn);
#endif
    *littleBuffer = '\0';
    return;
  }
  
  if (bigBuffer[currentPosn] == ',')
    currentPosn++;
    
  int destPosn = 0;
  while (currentPosn < strlen(bigBuffer) && bigBuffer[currentPosn] != ',')
  {
#ifdef DEBUG      
    Serial.print("      Copying over -->");
    Serial.println(bigBuffer[currentPosn]);
#endif
    littleBuffer[destPosn] = bigBuffer[currentPosn];
    destPosn++;
    currentPosn++;
  }
  
#ifdef DEBUG      
    Serial.print("      Terminating at -->");
    Serial.println(destPosn);
    Serial.print("   ");
    Serial.println(currentPosn);
#endif
    littleBuffer[destPosn] = '\0';
}

//--------------------------------------------------------------------
// GetNextGpsLine
//--------------------------------------------------------------------
void GetNextGpsLine()
{
  
#ifdef SIMULATING
#ifdef DEBUG      
  Serial.println("Getting simulation GPS line...");
#endif
  delay(gpsDelayMsec);
  runTimeMsec += gpsDelayMsec;
  int nmeaIndex = numGpsLinesRead % 6;
#ifdef DEBUG      
  Serial.print("Getting simulation GPS line, nmeaIndex = ");
  Serial.println(nmeaIndex);
#endif
  
  switch (nmeaIndex)
  {
    case 0:
      if (fakeNoFix)
        strcpy(bigBuffer, "$GPGSA,A,1,,,,,,,,,,,,,,,*1E");
      else
        strcpy(bigBuffer, "$GPGSA,A,3,27,16,23,13,06,,,,,,,,2.3,2.1,1.0*36");
      break;
    case 1:
      if (fakeNoFix)
        strcpy(bigBuffer, "$GPGSV,3,1,11,21,48,323,,26,38,056,,05,17,052,,29,66,189,*75");
      else
        strcpy(bigBuffer, "$GPGSV,3,1,12,23,80,154,21,13,62,324,23,03,55,115,26,27,45,091,27*7B");
      break;
    case 2:
      if (fakeNoFix)
        strcpy(bigBuffer, "$GPGSV,3,2,11,15,65,107,,18,45,266,,22,12,251,,02,06,106,*75");
      else
        strcpy(bigBuffer, "$GPGSV,3,2,12,30,43,039,,16,42,036,29,06,42,076,21,19,36,145,*75");
      break;
    case 3:
      if (fakeNoFix)
        strcpy(bigBuffer, "$GPGSV,3,3,11,25,05,205,,24,05,144,,16,02,316,*4F");
      else
        strcpy(bigBuffer, "$GPGSV,3,2,12,30,43,039,,16,42,036,29,06,42,076,21,19,36,145,*75");
      break;
    case 4:
      if (fakeNoFix)
        strcpy(bigBuffer, "$GPRMC,025548.000,V,,,,,,,240114,,,N*4E");
      else
        strcpy(bigBuffer, "$GPRMC,025548.000,A,3028.9512,N,09747.6292,W,17.17,170.1,230114,,,A*6D");
      break;
    case 5:
      if (fakeNoFix)
        strcpy(bigBuffer, "$GPGGA,025549.020,,,,,0,00,,,M,0.0,M,,*59");
      else
        strcpy(bigBuffer, "$GPGGA,025549.000,3028.9512,N,09747.6292,W,1,05,2.1,272.6,M,-22.5,M,,*6A");
      break;
  }
  
  digitalWrite(LED_PIN, HIGH); 
  delay(20);
#ifdef DEBUG      
  Serial.println(bigBuffer);
#endif
  numGpsLinesRead++;
  digitalWrite(LED_PIN, LOW); 
  return;
#endif  // SIMULATING

  digitalWrite(LED_PIN, HIGH); 
  char c;

  do 
  {
    c = ReadGpsChar();
  }
  while (c != '$');

  bigBuffer[0] = '$';
  int charsCopied = 1;

  do
  {
    c = ReadGpsChar();
        
    if (c != '\n' && c != '\r' && c!= '$' && charsCopied < BIG_BUFFER_SIZE - 1)
    {
      bigBuffer[charsCopied] = c;
      charsCopied++;
    }
  }
  while (c != '\n' && c != '\r' && c != '$');

  bigBuffer[charsCopied] = 0;
  numGpsLinesRead++;
  digitalWrite(LED_PIN, LOW); 
}

//--------------------------------------------------------------------
// ReadGpsChar
//--------------------------------------------------------------------
char ReadGpsChar()
{
    while (!gps.available())
      delay(10);
    
    return gps.read();  
}

//--------------------------------------------------------------------
// GetGpsTime - if not simulating, assumes $GPGGA sentence in 
//              bigBuffer
//--------------------------------------------------------------------
void GetGpsTime()
{
#ifdef SIMULATING
  long runTimeSec = runTimeMsec / 1000;
  int timeSec = runTimeSec % 60;
  int runTimeHours = (runTimeSec / 3600) % 24;
  int runTimeMins = (runTimeSec / 60) % 60;
  int runTimeSecs = runTimeSec % 60;
  sprintf(gpsTime, "%02d%02d%02d", runTimeHours, runTimeMins, runTimeSecs);
  return;
#endif
  
  if (strlen(bigBuffer) >= 13)
  {
    strncpy(gpsTime, bigBuffer + 7, 6);
    gpsTime[6] = '\0';
  }
  
  else
    strcpy(gpsTime, "000000");  // stranger things have happened

#ifdef DEBUG
  Serial.print("Updated time: ");
  Serial.println(gpsTime);
#endif
}

//--------------------------------------------------------------------
// GetGpsFixQuality - assumes $GPGGA sentence in bigBuffer
//--------------------------------------------------------------------
void GetGpsFixQuality()
{
  gpsFixQuality = 0;
  
#ifdef DEBUG
  Serial.println("Checking 7th $GPGGA parm for fix quality...");
  Serial.print("bigBuffer: ");
  Serial.println(bigBuffer);
#endif

  GetNthGpsParam(7);

#ifdef DEBUG
  Serial.print("Field: ");
  Serial.println(littleBuffer); 
#endif

  if (strlen(littleBuffer) > 0)
  {
    if (littleBuffer[0] == '1')
      gpsFixQuality = 1;
    else if (littleBuffer[0] == '2')
      gpsFixQuality = 2;
  }
}

//====================================================================
//====================================================================
//                         UTILITY METHODS
//====================================================================
//====================================================================
//--------------------------------------------------------------------
// WriteLoggerLine
//--------------------------------------------------------------------
void WriteLoggerLine(char * line)
{
  long ms = millis();
  
  Serial.print(logSequenceNum);
  Serial.print(",");
  Serial.print(gpsTime);
  Serial.print(",");
  Serial.print(ms);
  Serial.print(",");
  Serial.println(line);
  
#ifdef DEBUG
  Serial.println("---------------------------------- LOGGER LINE -------------------------------------");
  Serial.print(logSequenceNum);
  Serial.print(",");
  Serial.print(gpsTime);
  Serial.print(",");
  Serial.print(ms);
  Serial.print(",");
  Serial.println(line);
  Serial.println("------------------------------------------------------------------------------------");
#endif

  logSequenceNum++;
}

//--------------------------------------------------------------------
// ReleaseAirplane - Let that suckah fly!
//--------------------------------------------------------------------
void ReleaseAirplane()
{
  WriteLoggerLine("GOT COMMAND TO RELEASE AIRPLANE!");

/*  
  // Turn the stepper motor
  stepper.step(1500);
  
  // Turn off the adapter
  digitalWrite(6, LOW);
  digitalWrite(5, LOW);
  digitalWrite(4, LOW);
  digitalWrite(3, LOW);
*/
  
  airplaneReleased = true; 
  releaseAirplane = false;

  WriteLoggerLine("Warrior II...AWAY!!");
}

//--------------------------------------------------------------------
// charsAreNumeric
//--------------------------------------------------------------------
boolean charsAreNumeric(char * buff)
{
  if (!buff || strlen(buff) == 0)
    return false;
    
  for (int i=0; i<strlen(buff); i++)
  {
    char c = buff[i];
    
    if (c != '.' && (c < '0' || c > '9'))
      return false;
  }
  
  return true;
}

//--------------------------------------------------------------------
// GetAvailableMemory
//--------------------------------------------------------------------
int GetAvailableMemory() 
{
extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

