#include "open_evse.h"

#ifdef SERIALCLI
CLI g_CLI;

static const char s_psEnabled[] PROGMEM = "enabled";
static const char s_psDisabled[] PROGMEM = "disabled";

CLI::CLI()
{
  m_CLIstrCount = 0; 
  m_strBuf = g_sTmp;
  m_strBufLen = sizeof(g_sTmp);
}

void CLI::info()
{
  println_P(PSTR("OpenEVSE")); // CLI print prompt when serial is ready
  print_P(PSTR("Firmware - Open EVSE V")); //CLI info
  println_P(VERSTR);
  printlnn();
}

void CLI::Init()
{
  info();
  println_P(PSTR("type help for command list"));
  print_P(PSTR("OpenEVSE> ")); // CLI Prompt
  flush();

}

uint8_t CLI::getInt()
{
  uint8_t c;
  uint8_t num = 0;

  do {
    c = Serial.read(); // read the byte
    if ((c >= '0') && (c <= '9')) {
      num = (num * 10) + c - '0';
    }
  } while (c != 13);
  return num;
}

void CLI::printlnn()
{
  println("");
}

const char g_pson[] PROGMEM = "on";
void CLI::getInput()
{
  int currentreading;
  uint8_t amp;
  if (Serial.available()) { // if byte(s) are available to be read
    char inbyte = (char) Serial.read(); // read the byte
    Serial.print(inbyte);
    if (inbyte != 13) { // CR
      if (((inbyte >= 'a') && (inbyte <= 'z')) || ((inbyte >= '0') && (inbyte <= '@') || (inbyte == ' ')) ) { //sar - allow ?
	m_CLIinstr[m_CLIstrCount] = inbyte;
	m_CLIstrCount++;
      }
 else if (m_CLIstrCount && ((inbyte == 8) || (inbyte == 127))) {
	m_CLIstrCount--;
      }
    }

    if ((inbyte == 13) || (m_CLIstrCount == CLI_BUFLEN-1)) { // if enter was pressed or max chars reached
      m_CLIinstr[m_CLIstrCount] = '\0';
      printlnn(); // print a newline
      
      if (strcmp_P(m_CLIinstr, PSTR("show")) == 0){ //if match SHOW 
        info();
        
        println_P(PSTR("Settings"));
	print_P(PSTR("Service level: L"));
	Serial.println((int)g_EvseController.GetCurSvcLevel()); 
        print_P(PSTR("Current capacity (Amps): "));
        Serial.println((int)g_EvseController.GetCurrentCapacity()); 
        print_P(PSTR("Min Current Capacity: "));
        Serial.println(MIN_CURRENT_CAPACITY_J1772);
        print_P(PSTR("Max Current Capacity: "));
        Serial.println((g_EvseController.GetCurSvcLevel() == 2) ? MAX_CURRENT_CAPACITY_L2 : MAX_CURRENT_CAPACITY_L1);
	print_P(PSTR("Vent Required: "));
	println_P(g_EvseController.VentReqEnabled() ? s_psEnabled : s_psDisabled);
         print_P(PSTR("Diode Check: "));
	println_P(g_EvseController.DiodeCheckEnabled() ? s_psEnabled : s_psDisabled);

#ifdef ADVPWR
	print_P(PSTR("Ground Check: "));
	println_P(g_EvseController.GndChkEnabled() ? s_psEnabled : s_psDisabled);
	print_P(PSTR("Stuck Relay Check: "));
	println_P(g_EvseController.StuckRelayChkEnabled() ? s_psEnabled : s_psDisabled);
#endif // ADVPWR           
        // Start Delay Timer feature - GoldServe
#ifdef DELAYTIMER
        print_P(PSTR("Delay Timer: "));
        if (g_DelayTimer.IsTimerEnabled()){
          println_P(s_psEnabled);
        } else {
          println_P(s_psDisabled);
        }
        print_P(PSTR("Start Time: "));
        Serial.print(g_DelayTimer.GetStartTimerHour(), DEC);
        print_P(PSTR(" hour "));
        Serial.print(g_DelayTimer.GetStartTimerMin(), DEC);
        println_P(PSTR(" min"));
        print_P(PSTR("End Time: "));
        Serial.print(g_DelayTimer.GetStopTimerHour(), DEC);
        print_P(PSTR(" hour "));
        Serial.print(g_DelayTimer.GetStopTimerMin(), DEC);
        println_P(PSTR(" min"));
        print_P(PSTR("System Date/Time: "));
        g_CurrTime = g_RTC.now();
        Serial.print(g_CurrTime.year(), DEC);
        Serial.print('/');
        Serial.print(g_CurrTime.month(), DEC);
        Serial.print('/');
        Serial.print(g_CurrTime.day(), DEC);
        Serial.print(' ');
        Serial.print(g_CurrTime.hour(), DEC);
        Serial.print(':');
        Serial.print(g_CurrTime.minute(), DEC);
        Serial.print(':');
        Serial.print(g_CurrTime.second(), DEC);
        // End Delay Timer feature - GoldServe
#endif //#ifdef DELAYTIMER
      } 
 else if ((strcmp_P(m_CLIinstr, PSTR("help")) == 0) || (strcmp_P(m_CLIinstr, PSTR("?")) == 0)){ // string compare
        println_P(PSTR("Help Commands"));
        printlnn();
        println_P(PSTR("help - Display commands")); // print to the terminal
        println_P(PSTR("set  - Change settings"));
        println_P(PSTR("show - Display settings/values"));
        // Start Delay Timer feature - GoldServe
#ifdef DELAYTIMER
 println_P(PSTR("dt - Date/Time commands"));
 println_P(PSTR("timer - Delay timer commands"));
#endif //#ifdef DELAYTIMER
 // End Delay Timer feature - GoldServe
 } 
 else if (strcmp_P(m_CLIinstr, PSTR("set")) == 0) { // string compare
   println_P(PSTR("Set Commands - Usage: set amp"));
   printlnn();
   println_P(PSTR("amp - set current capacity"));
   println_P(PSTR("vntreq on/off - enable/disable vent required state"));
   println_P(PSTR("diochk on/off - enable/disable diode check"));

#ifdef ADVPWR
   println_P(PSTR("gndchk on/off - turn ground check on/off"));
   println_P(PSTR("rlychk on/off - turn stuck relay check on/off"));
#endif // ADVPWR
   println_P(PSTR("sdbg on/off - turn serial debugging on/off"));
 }
 else if (strncmp_P(m_CLIinstr, PSTR("set "),4) == 0) {
   char *p = m_CLIinstr + 4;
   if (!strncmp_P(p,PSTR("sdbg "),5)) {
     p += 5;
     print_P(PSTR("serial debugging "));
     if (!strcmp_P(p,g_pson)) {
       g_EvseController.EnableSerDbg(1);
       println_P(s_psEnabled);
     }
     else {
       g_EvseController.EnableSerDbg(0);
       println_P(s_psDisabled);
     }
   }
   else if (!strncmp_P(p,PSTR("vntreq "),7)) {
     p += 7;
     print_P(PSTR("vent required "));
     if (!strcmp_P(p,g_pson)) {
       g_EvseController.EnableVentReq(1);
       println_P(s_psEnabled);
     }
     else {
       g_EvseController.EnableVentReq(0);
       println_P(s_psDisabled);
     }
   }
   else if (!strncmp_P(p,PSTR("diochk "),7)) {
     p += 7;
     print_P(PSTR("diode check "));
     if (!strcmp_P(p,g_pson)) {
       g_EvseController.EnableDiodeCheck(1);
       println_P(s_psEnabled);
     }
     else {
       g_EvseController.EnableDiodeCheck(0);
       println_P(s_psDisabled);
     }
   }
#ifdef ADVPWR
   else if (!strncmp_P(p,PSTR("gndchk "),7)) {
     p += 7;
     print_P(PSTR("ground check "));
     if (!strcmp_P(p,g_pson)) {
       g_EvseController.EnableGndChk(1);
       println_P(s_psEnabled);
     }
     else {
       g_EvseController.EnableGndChk(0);
       println_P(s_psDisabled);
     }
   }
   else if (!strncmp_P(p,PSTR("rlychk "),7)) {
     p += 7;
     print_P(PSTR("stuck relay check "));
     if (!strcmp_P(p,g_pson)) {
       g_EvseController.EnableStuckRelayChk(1);
       println_P(s_psEnabled);
     }
     else {
       g_EvseController.EnableStuckRelayChk(0);
       println_P(s_psDisabled);
     }
   }
#endif // ADVPWR
   else if (!strcmp_P(p,PSTR("amp"))){ // string compare
     println_P(PSTR("WARNING - Do not set higher than 80% of breaker value"));
     printlnn();
     print_P(PSTR("Enter amps ("));
          Serial.print((g_EvseController.GetCurSvcLevel() == 2) ? MIN_CURRENT_CAPACITY_L2 : MIN_CURRENT_CAPACITY_L1);
     print_P(PSTR("-"));
     Serial.print((g_EvseController.GetCurSvcLevel()  == 1) ? MAX_CURRENT_CAPACITY_L1 : MAX_CURRENT_CAPACITY_L2);
     print_P(PSTR("): "));
     amp = getInt();
     Serial.println((int)amp);
     if(g_EvseController.SetCurrentCapacity(amp,1)) {
       println_P(PSTR("Invalid Setting"));
     }
	  
     print_P(PSTR("Max current: ")); // print to the terminal
     Serial.print((int)g_EvseController.GetCurrentCapacity());
     print_P(PSTR("A"));
   } 
   else {
     goto unknown;
   }
 }
      // Start Delay Timer feature - GoldServe
#ifdef DELAYTIMER
 else if (strncmp_P(m_CLIinstr, PSTR("dt"), 2) == 0){ // string compare
   char *p = m_CLIinstr + 3;
        
   if (strncmp_P(p,PSTR("set"),3) == 0) {
     p += 4;
     println_P(PSTR("Set Date/Time (mm/dd/yy hh:mm)"));
     print_P(PSTR("Month (mm): "));
     g_month = getInt();
     Serial.println(g_month);
     print_P(PSTR("Day (dd): "));
     g_day = getInt();
     Serial.println(g_day);
     print_P(PSTR("Year (yy): "));
     g_year = getInt();
     Serial.println(g_year);
     print_P(PSTR("Hour (hh): "));
     g_hour = getInt();
     Serial.println(g_hour);
     print_P(PSTR("Minute (mm): "));
     g_min = getInt();
     Serial.println(g_min);
          
     if (g_month + g_day + g_year + g_hour + g_min) {
       g_RTC.adjust(DateTime(g_year, g_month, g_day, g_hour, g_min, 0));
       println_P(PSTR("Date/Time Set"));
     } else {
       println_P(PSTR("Date/Time NOT Set")); 
     }
   }
   else {
     g_CurrTime = g_RTC.now();
     Serial.print(g_CurrTime.year(), DEC);
     Serial.print('/');
     Serial.print(g_CurrTime.month(), DEC);
     Serial.print('/');
     Serial.print(g_CurrTime.day(), DEC);
     Serial.print(' ');
     Serial.print(g_CurrTime.hour(), DEC);
     Serial.print(':');
     Serial.print(g_CurrTime.minute(), DEC);
     Serial.print(':');
     Serial.print(g_CurrTime.second(), DEC);
     Serial.println();
     println_P(PSTR("Use 'dt set' to set the system date/time"));
   }
        
 }
 else if (strncmp_P(m_CLIinstr, PSTR("timer"), 5) == 0){ // string compare
   char *p = m_CLIinstr + 6;
        
   if (strncmp_P(p,PSTR("set start"),9) == 0) {
     println_P(PSTR("Set Start Time (hh:mm)"));
     print_P(PSTR("Hour (hh): "));
     g_hour = getInt();
     Serial.println(g_hour);
     print_P(PSTR("Minute (mm): "));
     g_min = getInt();
     Serial.println(g_min);
     g_DelayTimer.SetStartTimer(g_hour, g_min);
   } else if (strncmp_P(p,PSTR("set stop"),8) == 0) {
     println_P(PSTR("Set Stop Time (hh:mm)"));
     print_P(PSTR("Hour (hh): "));
     g_hour = getInt();
     Serial.println(g_hour);
     print_P(PSTR("Minute (mm): "));
     g_min = getInt();
     Serial.println(g_min);
     g_DelayTimer.SetStopTimer(g_hour, g_min);
   } else if (strncmp_P(p,PSTR("enable"),9) == 0) {
     println_P(PSTR("Delay timer enabled, autostart disabled"));
     g_DelayTimer.Enable();
   } else if (strncmp_P(p,PSTR("disable"),9) == 0) {
     println_P(PSTR("Delay timer disabled, autostart enabled"));
     g_DelayTimer.Disable();
   } else {
     print_P(PSTR("Delay Timer: "));
     if (g_DelayTimer.IsTimerEnabled()){
       println_P(PSTR("Enabled"));
     } else {
       println_P(PSTR("Disabled"));
     }
     print_P(PSTR("Start Time: "));
     Serial.print(g_DelayTimer.GetStartTimerHour(), DEC);
     print_P(PSTR(" hour "));
     Serial.print(g_DelayTimer.GetStartTimerMin(), DEC);
     println_P(PSTR(" min"));
     print_P(PSTR("End Time: "));
     Serial.print(g_DelayTimer.GetStopTimerHour(), DEC);
     print_P(PSTR(" hour "));
     Serial.print(g_DelayTimer.GetStopTimerMin(), DEC);
     println_P(PSTR(" min"));
     if (!g_DelayTimer.IsTimerValid()){
       println_P(PSTR("Start and Stop times can not be the same!"));  
     }
     println_P(PSTR(""));
     println_P(PSTR("Use 'timer enable/disable' to enable/disable timer function"));
     println_P(PSTR("Use 'timer set start/stop' to set timer start/stop times"));
   }
 }
#endif //#ifdef DELAYTIMER
      // End Delay Timer feature - GoldServe
 else { // if the input text doesn't match any defined above
 unknown:
   println_P(PSTR("Unknown Command -- type help for command list")); // echo back to the terminal
 } 
      printlnn();
      print_P(PSTR("OpenEVSE> "));
      g_CLI.flush();
      m_CLIstrCount = 0; // get ready for new input... reset strCount
      m_CLIinstr[0] = '\0'; // set to null to erase it
    }
  }
}

void CLI::println_P(const char PROGMEM *s)
{
  strncpy_P(m_strBuf,s,m_strBufLen);
  m_strBuf[m_strBufLen-1] = 0;
  println(m_strBuf);
}

void CLI::print_P(const char PROGMEM *s)
{
  strncpy_P(m_strBuf,s,m_strBufLen);
  m_strBuf[m_strBufLen-1] = 0;
  print(m_strBuf);
}

#endif // SERIALCLI
