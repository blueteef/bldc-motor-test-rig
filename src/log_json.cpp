#include "log_json.h"

static fs::File s_json;

static inline void spiSel(int tft, int sd){
  digitalWrite(tft, HIGH);
  digitalWrite(sd, LOW);
}
static inline void spiDesel(int tft, int sd){
  digitalWrite(sd, HIGH);
  digitalWrite(tft, LOW);
}

static void writeEscaped(fs::File& f, const char* s){
  f.print("\"");
  if(s){
    for(const char* p=s; *p; ++p){
      char c=*p;
      if(c=='\\' || c=='"'){
        f.print('\\'); f.print(c);
      }else if(c=='\n'){
        f.print("\\n");
      }else{
        f.print(c);
      }
    }
  }
  f.print("\"");
}

bool logJsonOpen(const char* path, int tft, int sd){
  if(s_json) return true;

  spiSel(tft,sd);
  s_json = SD.open(path, FILE_WRITE);
  spiDesel(tft,sd);

  if(!s_json) return false;

  spiSel(tft,sd);
  s_json.println("{");
  spiDesel(tft,sd);

  return true;
}

void logJsonClose(int tft, int sd){
  if(!s_json) return;

  spiSel(tft,sd);
  s_json.println("}");
  s_json.flush();
  s_json.close();
  spiDesel(tft,sd);
}

bool logJsonIsOpen(){ return (bool)s_json; }
fs::File logJsonFile(){ return s_json; }

void logJsonBeginObject(const char* name, int tft, int sd){
  if(!s_json) return;

  spiSel(tft,sd);
  if(name){
    writeEscaped(s_json,name);
    s_json.print(": ");
  }
  s_json.println("{");
  spiDesel(tft,sd);
}

void logJsonEndObject(bool comma, int tft, int sd){
  if(!s_json) return;
  spiSel(tft,sd);
  s_json.print("}");
  if(comma) s_json.print(",");
  s_json.println();
  spiDesel(tft,sd);
}

static void keyPrefix(const char* k, int tft, int sd){
  spiSel(tft,sd);
  writeEscaped(s_json,k);
  s_json.print(": ");
  spiDesel(tft,sd);
}

void logJsonWriteKeyValueStr(const char* k,const char* v,int tft,int sd){
  if(!s_json) return;
  keyPrefix(k,tft,sd);
  spiSel(tft,sd);
  writeEscaped(s_json,v?v:"");
  s_json.println(",");
  spiDesel(tft,sd);
}

void logJsonWriteKeyValueU32(const char* k,uint32_t v,int tft,int sd){
  if(!s_json) return;
  keyPrefix(k,tft,sd);
  spiSel(tft,sd);
  s_json.print(v);
  s_json.println(",");
  spiDesel(tft,sd);
}

void logJsonWriteKeyValueI32(const char* k,int32_t v,int tft,int sd){
  if(!s_json) return;
  keyPrefix(k,tft,sd);
  spiSel(tft,sd);
  s_json.print(v);
  s_json.println(",");
  spiDesel(tft,sd);
}

void logJsonWriteKeyValueF(const char* k,float v,int d,int tft,int sd){
  if(!s_json) return;
  keyPrefix(k,tft,sd);
  spiSel(tft,sd);
  s_json.print(v,d);
  s_json.println(",");
  spiDesel(tft,sd);
}

void logJsonWriteKeyValueBool(const char* k,bool v,int tft,int sd){
  if(!s_json) return;
  keyPrefix(k,tft,sd);
  spiSel(tft,sd);
  s_json.print(v?"true":"false");
  s_json.println(",");
  spiDesel(tft,sd);
}

// ✅ COLUMN ARRAY WRITER
void logJsonWriteColumns(const char* const* cols,size_t count,int tft,int sd){
  if(!s_json || !cols || count==0) return;

  spiSel(tft,sd);

  writeEscaped(s_json,"columns");
  s_json.print(": [");

  for(size_t i=0;i<count;i++){
    writeEscaped(s_json, cols[i]);
    if(i+1<count) s_json.print(", ");
  }

  s_json.println("],");
  spiDesel(tft,sd);
}

void logJsonCloseWithStopMs(uint32_t stopMs,int tft,int sd){
  if(!s_json) return;

  spiSel(tft,sd);
  writeEscaped(s_json,"stop_ms");
  s_json.print(": ");
  s_json.println(stopMs);
  spiDesel(tft,sd);

  logJsonClose(tft,sd);
}
