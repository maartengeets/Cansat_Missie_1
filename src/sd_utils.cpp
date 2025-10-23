#include "sd_utils.h"
#include <FS.h>
#include <SD.h>
#include <SPI.h>

int ConnectSD() {
    if (!SD.begin(25)) {  // 25 is de SD_CSPIN
        Serial.println("SD kaart initialisatie mislukt!");
        return 0;
    }
    Serial.println("SD kaart geïnitialiseerd.");
    return 1;
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
    Serial.printf("Toevoegen aan bestand: %s\n", path);
    File file = fs.open(path, FILE_APPEND);
    if(!file) {
        Serial.println("Kan bestand niet openen om toe te voegen");
        return;
    }
    if(file.print(message)) {
        Serial.println("Bericht toegevoegd");
    } else {
        Serial.println("Toevoegen mislukt");
    }
    file.close();
}

void deleteFile(fs::FS &fs, const char *path) {
    Serial.printf("Verwijderen bestand: %s\n", path);
    if(fs.remove(path)) {
        Serial.println("Bestand verwijderd");
    } else {
        Serial.println("Verwijderen mislukt");
    }
}