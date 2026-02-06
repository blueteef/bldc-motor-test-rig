#pragma once
#include <Arduino.h>
#include <SD.h>

bool     logJsonOpen(const char* path, int tftCsPin, int sdCsPin);
void     logJsonClose(int tftCsPin, int sdCsPin);
bool     logJsonIsOpen();
fs::File logJsonFile();

void logJsonBeginObject(const char* nameOrNull, int tftCsPin, int sdCsPin);
void logJsonEndObject(bool trailingComma, int tftCsPin, int sdCsPin);

void logJsonWriteKeyValueStr (const char* key, const char* value, int tftCsPin, int sdCsPin);
void logJsonWriteKeyValueU32 (const char* key, uint32_t value, int tftCsPin, int sdCsPin);
void logJsonWriteKeyValueI32 (const char* key, int32_t value, int tftCsPin, int sdCsPin);
void logJsonWriteKeyValueF   (const char* key, float value, int decimals, int tftCsPin, int sdCsPin);
void logJsonWriteKeyValueBool(const char* key, bool value, int tftCsPin, int sdCsPin);

// ✅ This is what your main.cpp needs
void logJsonWriteColumns(const char* const* cols, size_t count, int tftCsPin, int sdCsPin);

void logJsonCloseWithStopMs(uint32_t stopMs, int tftCsPin, int sdCsPin);
