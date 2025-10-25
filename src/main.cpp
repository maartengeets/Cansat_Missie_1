// including libraries
#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <secrets.h> // secrets file invoegen

// definieren van filename
#define FILE_NAME "/CanSatSend.txt"

// definieren van de sd cs pin
#define SD_CSPIN  25

// wificlient definieren
WiFiClient wifiClient;

// mqttclient definieren
MqttClient mqttClient(wifiClient);

// stel de waarde van de luchtdruk op zeeniveau in voor jouw locatie
// je kunt deze vinden op https://www.meteo.be/nl/weer/waarnemingen/belgie
// als je dat doet, wordt de hoogte ongeveer correct berekend
// een afwijking van ±8 m is normaal, omdat de sensor een afwijking heeft
#define SEALEVELPRESSURE_HPA (1002.5)

#define BMP280_ADR  0x76 // zet I2C address van de BMP280 sensor
                         // sommige sensoren gebruiken het adress 0x77 

// in deze oefening, gebruiken we andere dan de default I2C pinnen
#define SDA_2   32 // gebruik deze PIN als tweede I2C SDA connectie
#define SCL_2   33 // gebruik deze PIN als tweede I2C SCL connectie
#define DELAY_TIME  1000

TwoWire I2Ctwo = TwoWire(1);  // maak I2Ctwo object op ESP32
                              // dees is nodig als we een tweede
                              // I2C connection willen gebruiken
Adafruit_BMP280 bmp( &I2Ctwo ); // declareer een object bmp waar data van de 
                                // bmp280 sensor kan geschreven worden in de
                                // I2C object I2Ctwo eerder gedeclareerd.

bool bmp_connected = false; // een globale var om doorheen het programma te checken
                            // of de bmp sensor geconnecteerd is

// vooruitdeclaties (prototypes) zodat loop() deze functies kan aanroepen
void printValues();
String csv_string_data();
void mqtt_pub_csv(const char *topic, String str_csv);
bool I2C_check(TwoWire *bus, byte address);
void appendFile(fs::FS &fs, const char * path, const char * message);
int  ConnectSD();
bool checkSDConnection();

void setup() {
  // initialiseren van de seriele poort en wachten tot het opent
  Serial.begin(115200);
  while (!Serial) {
    ; // wacht voor de seriele poort voor te connecteren
  }

  Serial.println("Begin programma");
  Serial.println("------------------");

  // verbinden met wifi
  Serial.print("Proberen te connecteren tot het wifi netwerk SSD: ");
  Serial.println(SECRET_SSID);
  WiFi.begin( SECRET_SSID, SECRET_PASS ); // verbinden met de wifi
  while (WiFi.status() != WL_CONNECTED) { 
    // print . elke seconden zolang de wifi niet verbonden is
    Serial.print(".");
    delay(1000);
  }

  Serial.println();
  Serial.println("Verbonden met de wifi!");
  Serial.println();

  // verbinden met mqtt broker
  Serial.print("Proberen te verbinden met de MQTT broker: ");
  Serial.println(BROKER);
    
  while (!mqttClient.connect(BROKER, PORT)) {
    Serial.print("MQTT verbinding mislukt! Fout code = ");
    Serial.println(mqttClient.connectError());
  }

  Serial.println("Verbonden met de broker!");
  Serial.println();

  Serial.print("Verbinden met topic: ");
  Serial.println(TOPIC);
  Serial.println();

  // subscribe op het topic
  mqttClient.subscribe(TOPIC);

  // testen van de bmp280 sensor
  Serial.println( "BMP280 test" );

  // connecteren vab BMP280 sensor met de niet default I2C connection
  I2Ctwo.begin(SDA_2, SCL_2);  // SDA=32, SCL=33
  // connecteren van de bmp280 sensor
    if (!bmp.begin( BMP280_ADR )) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring, address, sensor ID!");
        return;
    }
  
  bmp_connected = true; // omdat de connection van de bpm280 sensor oké is
                        // kunnen we de globale var op true zetten
  
  // optioneel: stel oversampling en filter in
  // om nauwkeurigere sensormetingen te verkrijgen
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // bmp succesvol verbonden
  Serial.println();
  Serial.println( "BMP280 succesvol verbonden!" );
  Serial.println();

  // verbinden met sd kaart
  Serial.println("Verbinden met sd kaart");
  ConnectSD();
  Serial.println();
  Serial.println("Verbonden met sd kaart!");
  Serial.println();

  // versie beheer
  const char *baseFile = "/CanSatSend.txt"; // zet de basefile als CanSatSend.txt

  // Controleren of CanSatSend.txt bestaat
  if (SD.exists(baseFile)) {
    Serial.println("Bestand gevonden: CanSatSend.txt");

    // Zoek naar eerstvolgende vrije versie
    int version = 1;
    char newFileName[40];

    bool bezet = true;
    while (bezet) {
      snprintf(newFileName, sizeof(newFileName), "/CanSatSend_%04d.txt", version);
      if (!SD.exists(newFileName)) {
        bezet = false; // Deze bestandsnaam is nog vrij
      }
      version++;
    }

    // Hernoem het bestand
    if (SD.rename(baseFile, newFileName)) {
      Serial.printf("Bestand hernoemd naar: %s\n", newFileName);
    } else {
      Serial.println("Hernoemen mislukt!");
    }

  } else {
    Serial.println("Geen bestand met de naam CanSatSend.txt gevonden.");
  }
}

void loop() {
  // behouden van de verbinding met mqttClient
  mqttClient.poll();
  // var definieren waar csv in komt
  String data = ""; 

  char msg[100] = "";

  // check of de BMP280 sensor nog steeds verbonden is door I2C communicatie te testen
  bmp_connected = I2C_check(&I2Ctwo, BMP280_ADR);

  if (bmp_connected) {
      printValues();
      data = csv_string_data(); // zet de csv in de var data
      mqtt_pub_csv(TOPIC,data); // publiceert de data (csv) op mqtt broker
      data = data +"\n";
      data.toCharArray(msg,100); // zet de csv string om naar een char lijst
      if(checkSDConnection()){ // kijkt of de sd kaart verbonden is
        appendFile(SD, FILE_NAME, msg); // zet data in het bestand met de naam FILE_NAME
      }
  } else {
      Serial.println("ERROR: BMP280 sensor not found!");
  }
  
  delay(1000);
}

// functie die de waardes van bmp280 sensor print
void printValues() {
  
  // Extra safety check
  if (!bmp_connected) return;

  // waardes ophalen
  float temperature = bmp.readTemperature(); 
  float pressure = bmp.readPressure() / 100.0F; 
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  // waardes printen
  Serial.print("Temperature = ");
  Serial.print( temperature );
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print( pressure );
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
   Serial.print( altitude );
  Serial.println(" m");
}

// functie die de waarde in csv string zet
String csv_string_data(){
  // Extra safety check
  if (!bmp_connected) return String(); // returnt een lege string

  // waarde ophalen
  float temperature = bmp.readTemperature(); // Read temperature
  float pressure = bmp.readPressure() / 100.0F; // Read pressure
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA); // Read altitude

  // waarde in csv string zetten
  String csv = String(temperature) + ";" + String(pressure) + ";" + String(altitude); // CSV string

  return csv;
}

// sturen van csv string op mqtt broker
void mqtt_pub_csv(const char *topic, String str_csv){
  mqttClient.beginMessage( topic ); // bericht sturen op TOPIC
  mqttClient.print( str_csv ); // sturen van CSV string op mqtt broker
  mqttClient.endMessage(); // beëindigen van het bericht
}

// functie die checkt of een I2C apparaat bereikbaar is op het gegeven adres
bool I2C_check(TwoWire *bus, byte address) {
    bus->beginTransmission(address);    // start communicatie met I2C apparaat
    byte error = bus->endTransmission(); // beëindig communicatie en krijg foutcode
    return (error == 0);                // true als geen fout (error=0)
}

// functie die een message zet in een bestand
void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Sturen naar bestand: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Openen van het bestand mislukt!");
    return;
  }
  if(file.print(message)){
      Serial.print( message );
  } else {
    Serial.println("Sturen mislukt");
  }
  file.close();
}

// functie die verbind met de sd kaart
int ConnectSD() {
    // initialiseert de SD kaart
    Serial.print("Initializeren van SD card...");
    
    // blijft proberen te verbinden met de SD kaart totdat het lukt
    while (!SD.begin( SD_CSPIN )) {
        Serial.print(".");
        delay( 50 );
    }
    Serial.println("initializeren gelukt.");

    // Bepaalt het type SD kaart
    uint8_t cardType = SD.cardType();

    // controleert of er een SD kaart aanwezig is
    if(cardType == CARD_NONE){
        Serial.println("Geen SD card geconnecteerd!");
        return( -1);  // Geeft -1 terug als er geen kaart gevonden is
    }

    // print het type SD kaart
    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("ONBEKEND");
    }

    // Berekent en print de grootte van de SD kaart in MB
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Grootte: %lluMB\n", cardSize);

    return 0;  // Geeft 0 terug als alles succesvol is
}

// functie die de sd kaart connectie test
bool checkSDConnection() {
    uint8_t cardType = SD.cardType();
    
    if(cardType == CARD_NONE) {
        Serial.println("SD kaart niet gevonden!");
        return false;
    }
    
    // Test of we kunnen schrijven naar de kaart
    File testFile = SD.open("/test.txt", FILE_WRITE);
    if(!testFile) {
        Serial.println("SD kaart schrijf test mislukt!");
        return false;
    }
    testFile.close();
    
    return true;
}