// Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
// 
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software!
     
#include <Adafruit_GPS.h> //Load the GPS Library. Make sure you have installed the library form the adafruit site above
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>  
#include <utility/imumaths.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial4
void memory();
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
     
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();
int relay=36;
int buzzer=34;
boolean trailer=false;

/*--------------------Defining SD------------------------*/
File File1;
int chipSelect = BUILTIN_SDCARD;

/*--------------------Defining BME 280------------------------*/

Adafruit_BME280 bme; // I2C

#define SEALEVELPRESSURE_HPA (915) //Fırlatma günü yerdeki basınç değeri yazılacak

/*--------------------Defining BNO055---------------------*/

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  
  pinMode(buzzer,OUTPUT);
digitalWrite(buzzer,HIGH);
delay(100);
digitalWrite(34,LOW);
delay(100);
digitalWrite(buzzer,HIGH);
delay(100);
digitalWrite(buzzer,LOW);
delay(100);
digitalWrite(buzzer,HIGH);
delay(100);
digitalWrite(buzzer,LOW);

Serial3.begin(9600);
     
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  
 //SD //
  SD.begin(chipSelect);
  if (SD.exists("accelerationx.txt") && SD.exists("accelerationy.txt") && SD.exists("accelerationz.txt") && SD.exists("temperature.txt") && SD.exists("height.txt") )
  {
SD.remove("accelerationx.txt");
SD.remove("accelerationy.txt");
SD.remove("accelerationz.txt");
SD.remove("temperature.txt");
SD.remove("height.txt");
  }

    delay(1000);
  }



void loop() // run over and over again
{
  readgps();
  printBME();
  memory();      
   
  }

void readgps() {

// read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial3.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    Serial3.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    
      Serial3.print("Location: ");
      Serial3.print(GPS.latitude, 4); Serial3.print(GPS.lat);
      Serial3.print(", ");
      Serial3.print(GPS.longitude, 4); Serial3.println(GPS.lon);
}
}



/*----------------------BME 280 FUNCTIONS-----------------------*/
void printBME()
{  
  
    delay(50);
    float height1=bme.readAltitude(SEALEVELPRESSURE_HPA);
    Serial3.print(height1,4);
    Serial3.print(" m");
    Serial3.print("     ");  

   if (height1>=1000) {

    trailer=true;
   }

 

  if (trailer==true&& height1<550) {

    digitalWrite(buzzer,HIGH);

  }  
}




/*-----------------------SD FUNCTİON--------------------*/

void memory()
{
delay(50);
File1 = SD.open("height.txt",FILE_WRITE);
if (File1) {
File1.print(timer);
File1.print("\t");
File1.println(bme.readAltitude(SEALEVELPRESSURE_HPA),4);
File1.close();
}   

File1 = SD.open("temperature.txt",FILE_WRITE);
if (File1) {
File1.print(timer);
File1.print("\t");
File1.println(bme.readTemperature(),4);
File1.close();
}   

imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

File1 = SD.open("accelerationx.txt",FILE_WRITE);
if (File1) {
File1.print(timer);
File1.print("\t");
File1.println(accel.x()); //X_ACCELERATION  (m/s^2)
File1.close();
}   


File1 = SD.open("accelerationy.txt",FILE_WRITE);
if (File1) {
File1.print(timer);
File1.print("\t");
File1.println(accel.y()); //Y_ACCELERATION  (m/s^2)
File1.close();
}   


File1 = SD.open("accelerationz.txt",FILE_WRITE);
if (File1) {
File1.print(timer);
File1.print("\t");
File1.println(accel.z()); //Z_ACCELERATION  (m/s^2)
File1.close();
} 
}























