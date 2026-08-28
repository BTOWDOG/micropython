#ifndef PARAM_STORAGE_H
#define PARAM_STORAGE_H

#include <stdbool.h>
void storageInit();
bool saveConfig();
bool loadConfig();
bool getCalibrated();

#endif