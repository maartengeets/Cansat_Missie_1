#ifndef SD_UTILS_H
#define SD_UTILS_H

#include <FS.h>

void appendFile(fs::FS &fs, const char * path, const char * message);
void deleteFile(fs::FS &fs, const char * path);
int ConnectSD();

#endif