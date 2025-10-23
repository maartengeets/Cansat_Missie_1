#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Secrets.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include "sd_utils.h"

#define FILE_NAME "/Maarten_Test.txt"
#define SD_CSPIN  25

// WiFi Client definition
WiFiClient wifiClient;

// MQTT Client definition
MqttClient mqttClient(wifiClient);

// Set the value of the sea level pressure correct at your location
// You can find it at https://www.meteo.be/nl/weer/waarnemingen/belgie 
// if you do so, the height will be calculated approx. correctly
// + or - 8m hight difference is normal, as the sensor has a deviation
#define SEALEVELPRESSURE_HPA (1007.1)

#define BMP280_ADR  0x76 // Set I2C address of BMP280 sensor
                         // Some sensors will use address 0x77      

// In this example, we'll use differnt than default pins for I2C connection
#define SDA_2   32 // Use this PIN as secondary I2C SDA connection
#define SCL_2   33 // Use this PIN as secondary I2C SCL connection
#define DELAY_TIME  1000

TwoWire I2Ctwo = TwoWire(1);  // Create I2Ctwo object on ESP32
                              // This is needed as we will be using a second
                              // I2C connection
Adafruit_BMP280 bmp( &I2Ctwo ); // declare an object bmp where data from the 
                                // bmp280 sensor can be written into the
                                // I2C object I2Ctwo previously declared.

bool bmp_connected = false; // A global var to indicate throughout the program
                            // if the bmp sensor is connected or not

/* Declare functions
void appendFile(fs::FS &fs, const char * path, const char * message);
void deleteFile(fs::FS &fs, const char * path);
int  ConnectSD();
*/
bool I2C_check(TwoWire *bus, byte adress);
void printValues();

// Define constants for error messages
const char* ERROR_BMP_NOT_FOUND = "Could not find a valid BMP280 sensor, check wiring, address, sensor ID!";
const char* ERROR_MQTT_CONNECTION_FAILED = "MQTT connection failed! Error code = ";

void setup() {
  
  char msg[120];
  
    // start serial monitor
    Serial.begin( 115200 );
    Serial.println( "Start of Program" );
    Serial.println( "----------------" );
    Serial.println("Setting BMP280 up:");
    while(!Serial) {
        delay( 10 );
    }    // time to get serial running
    Serial.println( "BMP280 test" );
    
    // Connect BMP280 sensor with non-default I2C connection
    I2Ctwo.begin(SDA_2, SCL_2);  // SDA=32, SCL=33

    // Connect the bmp280 sensor
    if (!bmp.begin( BMP280_ADR )) {
        Serial.println(ERROR_BMP_NOT_FOUND);
        return;
    }
    
    // If the bmp280 sensor is connected successfully, we continue with a few 
    // more things, as per below code
    
    bmp_connected = true; // Since the connection of the bpm280 sensor is OK
                          // We can now set this global var accordingly

    // Optional: configure oversampling and filter
    // to obtain more accurate sensor readings
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    Serial.println("-- Default Test --");
    Serial.println( "CanSat => BMP-280 successful connected (I2C-2)" );

    Serial.println();
    Serial.println();
  Serial.println( "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  Serial.println( "Sensor Output LED Stearing - Data Transmission starts!");
    
  // Connect to WiFi network
  Serial.println("Establishing WiFi connection...");
  WiFi.begin( SECRET_SSID, SECRET_PASS );
  while ( WiFi.status() != WL_CONNECTED) {
    Serial.print( "." );
    delay( 1000 );
  }
  // Connection to WiFi is success - provide some info about WiFi connection:
  sprintf( msg, "\nWiFi Connected, IP = %s\n", WiFi.localIP().toString() );
  Serial.println( msg );

  // Connect to MQTT broker & topic
  sprintf( msg, "Tracht te verbinden met MQTT broker- op hostadres: %s en topic %s", BROKER, TOPIC );
  Serial.println( msg );

  while (!mqttClient.connect( BROKER, PORT)) {
    Serial.print(ERROR_MQTT_CONNECTION_FAILED);
    Serial.println(mqttClient.connectError());
    delay( 1000 );
  }

  //Subscribe on MQTT Topic
  Serial.print ( "Subscribing on topic: " );
  Serial.println( TOPIC );

  mqttClient.subscribe( TOPIC );

  sprintf( msg, "Connection with MQTT broker %s on topic %s established!\n", BROKER, TOPIC );
  Serial.print( msg );
  Serial.println( "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  Serial.println("Starting SD Card:");
  ConnectSD();
  Serial.printf( "Deleting file %s\n", FILE_NAME );
  deleteFile( SD, FILE_NAME );
  delay( 3000 );

}

void loop() { 
    mqttClient.poll();

    bmp_connected = I2C_check(&I2Ctwo, BMP280_ADR);

    if (bmp_connected) {
        printValues();
    } else {
        Serial.println("ERROR: BMP280 sensor not found!");
    }
    
    
    delay(DELAY_TIME);
}

void printValues() {
    String csv = "";
    // Extra safety check
    if (!bmp_connected) return;

    float temperature = bmp.readTemperature(); // Read temperature
    float pressure = bmp.readPressure() / 100.0F; // Read pressure
    float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA); // Read altitude
    csv = String(temperature) + ";" + String(pressure) + ";" + String(altitude); // CSV string

    Serial.print("Temperature = ");
    Serial.print( temperature );
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print( pressure );
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print( altitude );
    Serial.println(" m");

    Serial.println(csv);
    Serial.println();
    mqttClient.beginMessage( TOPIC ); // Message beginning on TOPIC
    mqttClient.print( csv ); // Printing CSV string on mqtt broker
    mqttClient.endMessage(); // Ending message
    Serial.println( "Writing to file" );
    char msg[ 100 ] = "" ;
    String csv_2 = csv + "\n";
    csv_2.toCharArray(msg,100);
    Serial.println( msg );
    appendFile(SD, FILE_NAME, msg);
    delay( 1000 );

}


bool I2C_check(TwoWire *bus, byte address) {
    bus->beginTransmission(address);
    byte error = bus->endTransmission();
    return (error == 0);
}