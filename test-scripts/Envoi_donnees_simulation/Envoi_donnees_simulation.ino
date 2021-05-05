// ____________________________________________________________________________________
// Initialisation

/* This code simulates the 2 sensors, a lidar and a TFMini
 * It sends a range of 200 from the start to the switchTime 
 * and then sends a deacreasing range from 200 to 30 during switchPeriode.
 * It send sends a range of 30  
*/
#include "Wire.h"
//The Arduino Wire library uses the 7-bit version of the address, so the code example uses 0x70 instead of the 8‑bit 0xE0
#define SensorAddress byte(0x70)
//The Sensor ranging command has a value of 0x51
#define RangeCommand byte(0x51)
//These are the two commands that need to be sent in sequence to change the sensor address
#define ChangeAddressCommand1 byte(0xAA)
#define ChangeAddressCommand2 byte(0xA5)

float previousSonarReading = millis(); // Gives the time (in ms) at which the previous sonar reading was done
float previousLidarReading = millis(); // Gives the time (in ms) at which the previous lidar reading was done
bool rangeToRead = false; // True if a range reading as been asked to the sonar and hasn't been read yet
unsigned long startTime = millis();

// --------------------- PARAMS FOR SIMULATING THE SENSORS ----------------------------
int range = 200;
unsigned long switchTime = 200000; // Time (in ms) at which the simulated range goes from 200 to 30
unsigned long switchPeriode = 10000; // Time (in ms) during which the range will go linearly from 200 to 30
// ____________________________________________________________________________________

void setup() {
  Serial.begin(115200); // Baudrate of 115200 is fixed by the TFMini Plus (default value of the sensor)
  Wire.begin();     // Initiate Wire library for I2C communications with I2CXL‑MaxSonar‑EZ
}

void loop() {
  unsigned long ellapsedTime = millis() - startTime;
  // SETTING THE RANGE
  if ((ellapsedTime > switchTime) and (range > 30)) {
    range = 200-(30-200)*switchTime/switchPeriode + ellapsedTime * (30 - 200) / switchPeriode;
  }

  // SONAR READING
  if (millis() - previousSonarReading > 110) {
    // It's best to allow at least 100ms between sonar reading for proprer acoustic dissipation
    // So we ask a range reading from the sonar every 110 ms
    previousSonarReading = millis();
    rangeToRead = true; // We now have to read the data
  }
  if (millis() - previousSonarReading > 90 and rangeToRead == true) {
    // We can read the data only 80 ms after we requested a range reading
    Serial.print("Sonar_Range:"); Serial.println(range);   //Print to the user, the message has to start by "S" for the communication with the Raspberry (cf Rapsberry code)
    rangeToRead = false; // We no longer have data to read
  }

  // LIDAR READING
  if (millis() - previousLidarReading > 20) {
    // The default frame rate is equal to 100Hz
    // Thus we should ask for data from the Lidar every 10ms at least
    Serial.print("Lidar_Range:"); Serial.println(range); //Print to the user, the message has to start by "L" for the communication with the Raspberry (cf Rapsberry code)
    previousLidarReading = millis();
  }
}
