#pragma once
#include <Arduino.h>
#include <SD.h>

using LogHeaderWriter = void (*)(File&);

bool logCsvOpen(const char* filename,
                int tftCsPin,
                int sdCsPin,
                LogHeaderWriter headerWriter);

void logCsvWriteCommentU32(const char* key,
                           uint32_t value,
                           int tftCsPin,
                           int sdCsPin);

void logCsvFlush(int tftCsPin, int sdCsPin);
void logCsvClose(int tftCsPin, int sdCsPin);
void logCsvWriteBytes(const uint8_t* data, size_t n, int tftCsPin, int sdCsPin);
void logCsvMaybeFlush(uint16_t* flushCtr,
                      uint16_t flushEveryN,
                      int tftCsPin,
                      int sdCsPin);

bool logCsvIsOpen();
File& logCsvFile();  // temporary bridge; we'll remove later
