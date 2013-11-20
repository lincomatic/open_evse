#ifdef RAPI
typedef int int32;
typedef unsigned int uint32;
typedef short int16;
typedef unsigned short uint16;
typedef char int8;
typedef unsigned char uint8;

#define ESRAPI_BUFLEN 40
#define ESRAPI_EOC '\n'
#define ESRAPI_MAX_ARGS 10
class EvseRapiProcessor {
  char buffer[ESRAPI_BUFLEN]; // input buffer
  int8 bufCnt; // # valid bytes in buffer
  char *tokens[ESRAPI_MAX_ARGS];
  int8 tokenCnt;

  int available() { return Serial.available(); }
  int read() { return Serial.read(); }
  void write(const char *str) { Serial.write(str); }

  void reset() {
    buffer[0] = 0;
    bufCnt = 0;
  }

  uint8 htou(const char *s);
  uint8 dtou(const char *s);
  int tokenize();
  int processCmd();

  void response(uint8 ok);
  
public:
  EvseRapiProcessor();
  int doCmd();
  
  /*
  void Init();
  void println(char *s) { 
    Serial.println(s); 
  }
  void println_P(prog_char *s);
  void print(char *s) { 
    Serial.print(s); 
  }
  void print_P(prog_char *s);
  void printlnn();
  void flush() { 
    Serial.flush(); 
  }
  uint8_t getInt();
  */
};

#endif // RAPI
