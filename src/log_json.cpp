#include "log_json.h"

static File s_json;

static void spiSelectSd(int tftCsPin, int sdCsPin){
  digitalWrite(tftCsPin, HIGH);
  digitalWrite(sdCsPin, LOW);
}

static void spiDeselectSd(int tftCsPin, int sdCsPin){
  digitalWrite(sdCsPin, HIGH);
  digitalWrite(tftCsPin, HIGH);
}

static void jsonWriteEscaped(File& f, const char* s){
  if(!s) { f.print(""); return; }
  for(const char* p = s; *p; ++p){
    const char c = *p;
    switch(c){
      case '\"': f.print("\\\""); break;
      case '\\': f.print("\\\\"); break;
      case '\b': f.print("\\b");  break;
      case '\f': f.print("\\f");  break;
      case '\n': f.print("\\n");  break;
      case '\r': f.print("\\r");  break;
      case '\t': f.print("\\t");  break;
      default:
        if((uint8_t)c < 0x20) f.print(' ');
        else f.print(c);
        break;
    }
  }
}

bool logJsonOpen(const char* filename, int tftCsPin, int sdCsPin){
  if(s_json) s_json.close();

  spiSelectSd(tftCsPin, sdCsPin);
  s_json = SD.open(filename, FILE_WRITE);
  spiDeselectSd(tftCsPin, sdCsPin);

  if(!s_json) return false;

  spiSelectSd(tftCsPin, sdCsPin);
  s_json.println("{");
  spiDeselectSd(tftCsPin, sdCsPin);

  return true;
}

static void writeKeyPrefix(const char* key, int tftCsPin, int sdCsPin){
  spiSelectSd(tftCsPin, sdCsPin);
  s_json.print("  \"");
  jsonWriteEscaped(s_json, key);
  s_json.print("\": ");
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteKeyValueStr(const char* key, const char* value, int tftCsPin, int sdCsPin){
  if(!s_json || !key) return;
  writeKeyPrefix(key, tftCsPin, sdCsPin);

  spiSelectSd(tftCsPin, sdCsPin);
  s_json.print("\"");
  jsonWriteEscaped(s_json, value ? value : "");
  s_json.println("\",");
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteKeyValueU32(const char* key, uint32_t value, int tftCsPin, int sdCsPin){
  if(!s_json || !key) return;
  writeKeyPrefix(key, tftCsPin, sdCsPin);

  spiSelectSd(tftCsPin, sdCsPin);
  s_json.print(value);
  s_json.println(",");
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteKeyValueI32(const char* key, int32_t value, int tftCsPin, int sdCsPin){
  if(!s_json || !key) return;
  writeKeyPrefix(key, tftCsPin, sdCsPin);

  spiSelectSd(tftCsPin, sdCsPin);
  s_json.print(value);
  s_json.println(",");
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteKeyValueF(const char* key, float value, int decimals, int tftCsPin, int sdCsPin){
  if(!s_json || !key) return;
  if(decimals < 0) decimals = 0;
  if(decimals > 9) decimals = 9;

  writeKeyPrefix(key, tftCsPin, sdCsPin);

  spiSelectSd(tftCsPin, sdCsPin);
  s_json.print(value, decimals);
  s_json.println(",");
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteKeyValueBool(const char* key, bool value, int tftCsPin, int sdCsPin){
  if(!s_json || !key) return;
  writeKeyPrefix(key, tftCsPin, sdCsPin);

  spiSelectSd(tftCsPin, sdCsPin);
  s_json.print(value ? "true" : "false");
  s_json.println(",");
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonWriteColumns(const char* const* cols, size_t n, int tftCsPin, int sdCsPin){
  if(!s_json || !cols) return;

  writeKeyPrefix("columns", tftCsPin, sdCsPin);

  spiSelectSd(tftCsPin, sdCsPin);
  s_json.print("[");
  for(size_t i=0;i<n;i++){
    if(i) s_json.print(",");
    s_json.print("\"");
    jsonWriteEscaped(s_json, cols[i] ? cols[i] : "");
    s_json.print("\"");
  }
  s_json.println("],");
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonBeginObject(const char* keyOrNull, int tftCsPin, int sdCsPin){
  if(!s_json) return;

  spiSelectSd(tftCsPin, sdCsPin);
  if(keyOrNull){
    s_json.print("  \"");
    jsonWriteEscaped(s_json, keyOrNull);
    s_json.println("\": {");
  } else {
    s_json.println("{");
  }
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonEndObject(bool trailingComma, int tftCsPin, int sdCsPin){
  if(!s_json) return;

  spiSelectSd(tftCsPin, sdCsPin);
  if(trailingComma) s_json.println("  },");
  else              s_json.println("  }");
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonCloseWithStopMs(uint32_t stopMs, int tftCsPin, int sdCsPin){
  if(!s_json) return;

  spiSelectSd(tftCsPin, sdCsPin);
  s_json.print("  \"stop_ms\": ");
  s_json.println(stopMs);
  s_json.println("}");
  s_json.flush();
  s_json.close();
  spiDeselectSd(tftCsPin, sdCsPin);
}

void logJsonClose(int tftCsPin, int sdCsPin){
  if(!s_json) return;
  spiSelectSd(tftCsPin, sdCsPin);
  s_json.flush();
  s_json.close();
  spiDeselectSd(tftCsPin, sdCsPin);
}

bool logJsonIsOpen(){ return (bool)s_json; }
File& logJsonFile(){ return s_json; }
