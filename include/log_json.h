#pragma once
#include <Arduino.h>
#include <SD.h>

// Open JSON and start root "{"
bool logJsonOpen(const char* filename, int tftCsPin, int sdCsPin);

// Key/Value writers (always write trailing comma)
void logJsonWriteKeyValueStr(const char* key, const char* value, int tftCsPin, int sdCsPin);
void logJsonWriteKeyValueU32(const char* key, uint32_t value, int tftCsPin, int sdCsPin);
void logJsonWriteKeyValueI32(const char* key, int32_t value, int tftCsPin, int sdCsPin);
void logJsonWriteKeyValueF(const char* key, float value, int decimals, int tftCsPin, int sdCsPin);
void logJsonWriteKeyValueBool(const char* key, bool value, int tftCsPin, int sdCsPin);

// Arrays
void logJsonWriteColumns(const char* const* cols, size_t n, int tftCsPin, int sdCsPin);

// Object helpers
// Begin:  "key": {    (keyOrNull must not be null for nested objects)
void logJsonBeginObject(const char* keyOrNull, int tftCsPin, int sdCsPin);
// End:  },  (if trailingComma=true)  OR } (if false)
void logJsonEndObject(bool trailingComma, int tftCsPin, int sdCsPin);

// Final close: write "stop_ms": <value>\n}\n and close file
void logJsonCloseWithStopMs(uint32_t stopMs, int tftCsPin, int sdCsPin);

// Plain close
void logJsonClose(int tftCsPin, int sdCsPin);

bool logJsonIsOpen();
File& logJsonFile(); // optional bridge/debug
