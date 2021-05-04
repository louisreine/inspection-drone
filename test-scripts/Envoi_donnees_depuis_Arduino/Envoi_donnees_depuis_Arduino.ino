// ____________________________________________________________________________________
// Initialisation
// https://www.raspberrypi.org/forums/viewtopic.php?f=32&t=62714&p=465137&hilit=I2C+maxsonar#p465137

/* Code for Arduino Uno R3
  This asynchrone code reads data from two sensors : a TFMini Plus and a Maxbotix Maxsonar I2C XL Z0
  It then prints the data read on the serial port.
  We can then recover the data from this sensors on a Raspberry by connecting the Arduino and the Raspberry via their serial ports (USB).
  
  Sonar Connections:
  Pin 7 (black) to GND
  Pin 6 (red) to 5V (or 3.3V)
  Pin 5 (blue) to SCL
  Pin 4 (green) to SDA
  Requires pull‑ups for SCL and SDA connected to 5V to work reliably

  TFMini Connections:
  Red cable to 5V
  Black cable to GND
  Blue cable to RX
  White cable to TX
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

// ____________________________________________________________________________________

void setup() {
  Serial.begin(115200); // Baudrate of 115200 is fixed by the TFMini Plus (default value of the sensor)
  Wire.begin();     // Initiate Wire library for I2C communications with I2CXL‑MaxSonar‑EZ
}

void loop() {
  // SONAR READING
  if (millis() - previousSonarReading > 110) {
    // It's best to allow at least 100ms between sonar reading for proprer acoustic dissipation
    // So we ask a range reading from the sonar every 110 ms
    takeRangeReading();                                  //Tell the sensor to perform a ranging cycle
    previousSonarReading = millis();
    rangeToRead = true; // We now have to read the data
  }
  if (millis() - previousSonarReading > 90 and rangeToRead == true) {
    // We can read the data only 80 ms after we requested a range reading
    word range = requestRange();                           //Get the range from the sensor
    Serial.print("Sonar_Range:"); Serial.println(range);   //Print to the user, the message has to start by "S" for the communication with the Raspberry (cf Rapsberry code)
    rangeToRead = false; // We no longer have data to read
  }

  // LIDAR READING
  if (millis() - previousLidarReading > 20) {
    // The default frame rate is equal to 100Hz
    // Thus we should ask for data from the Lidar every 10ms at least
    int distance = 0;
    int strength = 0;
    getTFminiData(&distance, &strength);
    if (distance) {
      // From experience, we can have data from the lidar only 50 ms after we asked for data
      
      Serial.print("Lidar_Range:"); Serial.println(distance); //Print to the user, the message has to start by "L" for the communication with the Raspberry (cf Rapsberry code)
      previousLidarReading = millis();
      //        Serial.print("cm ");
      //        Serial.print("strength: ");
      //        Serial.println(strength);
    }
  }
}
// _______________________________________________
// TFMini function

void getTFminiData(int* distance, int* strength) {
  /* Function that modifies the inputs distance and strenght and gives data from the TFMini
  */
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (Serial.available()) {
    rx[i] = Serial.read();
    if (rx[0] != 0x59) {
      i = 0;
    } else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if (i == 8) {
      for (j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)) {
        // The data we want is obtained by combining low and high bits
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    } else {
      i++;
    }
  }
}

// Sonar functions

void takeRangeReading() {
  /* Commands the sonar to take a range reading
    It's best to allow at least 100ms between the readings
  */
  Wire.beginTransmission(SensorAddress);             //Start addressing
  Wire.write(RangeCommand);                          //send range command
  Wire.endTransmission();                            //Stop and do something else now
}

word requestRange() {
  /* Returns the last range that the sonar determined in its last ranging cycle in centimeters. Returns 0 if there is no communication.
  */
  Wire.requestFrom(SensorAddress, byte(2));
  if (Wire.available() >= 2) {                        //Sensor responded with the two bytes
    byte HighByte = Wire.read();                      //Read the high byte back
    byte LowByte = Wire.read();                       //Read the low byte back
    word range = word(HighByte, LowByte);             //Make a 16-bit word out of the two bytes for the range
    return range;
  }
  else {
    return word(0);                                   //Else nothing was received, return 0
  }
}
