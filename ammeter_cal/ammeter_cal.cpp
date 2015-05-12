/*
 * Copyright (c) 2014 Sam C. Lin <lincomatic@gmail.com>
 * This file is part of ammeter_cal.

 * ammeter_cal is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.

 * ammeter_cal is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with ammeter_cal; see the file COPYING.  If not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include <stdio.h>
#include <ctype.h>
#include "ammeter_cal.h"

#ifdef _WIN32
#include <windows.h>
#define SLEEP(x) Sleep(x*1000)
#else
#include <unistd.h>
#define SLEEP(x) sleep(x)
#endif

#define VERSTR "V0.11"

//#define LOG

// convert decimal string to uint32
static uint32 dtou(const char *s)
{
  uint32 u = 0;
  while (*s) {
    u *= 10;
    u += *(s++) - '0';
  }
  return u;
}


EvseRapiLink::EvseRapiLink()
{
}

EvseRapiLink::~EvseRapiLink()
{
  Close();
}

int8 EvseRapiLink::ReadResp(char *buf,int8 buflen,int8 nowait)
{
  *buf = 0;

  int rc = -1;
  char src;
  do {
    src= m_serial.ReadChar(buf,nowait ? 1 : 250);
  } while ((src == 1) && (buf[0] != '$'));
  
  if (src == 1) {
    src = m_serial.ReadString(buf+1,ESRAPI_EOC,buflen-1,500);
    if (src > 0) {
      int slm1 = strlen(buf)-1;
      if (buf[slm1] == ESRAPI_EOC) {
	buf[slm1] = 0; // get rid of ESRAPI_EOC
	rc = 0;
      }
    }
  }
  return rc;
}

EvseRapi::EvseRapi()
{
  m_cmdBuf[0] = ESRAPI_SOC;
  m_evseState = EVSE_STATE_UNKNOWN;
}

uint8 EvseRapi::calcChkSum()
{
  char *s = m_cmdBuf;
  uint8 chkSum = 0;
  while (*s) {
    chkSum += *(s++);
  }
  return chkSum;
}

void EvseRapi::appendChkSum()
{
  char *s = m_cmdBuf;
  uint8 chkSum = 0;
  while (*s) {
    chkSum += *(s++);
  }
  //  sprintf(s,"*%02X%c\0",(unsigned)chkSum,ESRAPI_EOC);
  sprintf(s,"*%02X",(unsigned)chkSum);
  s[3] = ESRAPI_EOC;
  s[4] = '\0';
}

//rc = 0 = success
//   = 1 = error reading response
//   = 2 = NAK received
int8 EvseRapi::runCommand()
{
  int8 rc = -1;
  appendChkSum();
  rc = m_link.WriteCmd(m_cmdBuf);
  if (!rc) {
  readresp:
    rc = m_link.ReadResp(m_respBuf,ESRAPI_RESPBUF_LEN);
    if (!rc) {
      if (!doAsync()) {
	goto readresp;
      }
      else if (!strncmp(m_respBuf,"$NK",3)) {
	rc = 2;
      }
      else if (strncmp(m_respBuf,"$OK",3)) {
	goto readresp;
      }
    }
  }

#ifdef LOG
  FILE *fp = fopen("log.txt","a");
  m_cmdBuf[strlen(m_cmdBuf)-1] = 0;
  fprintf(fp,"c=%s rc=%d r=%s\n",m_cmdBuf,rc,m_respBuf);
  fflush(fp);
  fclose(fp);
#endif // LOG

  return rc;
}

int8 EvseRapi::doAsync()
{
  int8 rc = -1;
  if (*m_respBuf == '$') {
    if (!strncmp(m_respBuf+1,"ST",2)) {
      // process state change
      m_evseState = dtou(m_respBuf+4);
#ifdef LOG
  FILE *fp = fopen("log.txt","a");
  fprintf(fp,"%s %d\n",m_respBuf,m_evseState);
  fflush(fp);
  fclose(fp);
#endif // LOG
      rc = 0;
    }
  }

  return rc;
}

int8 EvseRapi::ResetEvse()
{
  strcpy(m_cmdBuf+1,"FR");
  return runCommand();
}

int8 EvseRapi::GetVersion(char *ver)
{
  strcpy(m_cmdBuf+1,"GV");
  int8 rc = runCommand();
  if (!rc) {
    strncpy(ver,m_respBuf+4,ESRAPI_RESPBUF_LEN);
  }
  return rc;
}
void EvseRapi::GenCmd()
{
  for (;;) {
    gets(m_cmdBuf);
    appendChkSum();
    printf("%s\n",m_cmdBuf);
  }
}

int8 EvseRapi::EnableAmmeterCal(int8 tf)
{
  sprintf(m_cmdBuf+1,"S2 %c",tf ? '1' : '0');
  return runCommand();
}

int8 EvseRapi::SetAmmeter(int scale,int offset)
{
  sprintf(m_cmdBuf+1,"SA %d %d",scale,offset);
  int8 rc = runCommand();
  if (!rc) {
    FILE *fp = fopen("ammeter_cal_log.txt","a");
    fprintf(fp,"write: scale=%d %d\n",scale,offset);
    fflush(fp);
    fclose(fp);
  }
  return rc;
}

int8 EvseRapi::GetAmmeter(int *scale,int *offset)
{
  strcpy(m_cmdBuf+1,"GA");
  int8 rc = runCommand();
  if (!rc) {
    if (sscanf(m_respBuf+4,"%d %d",scale,offset) != 2) {
      rc = 1;
    }
    else {
      FILE *fp = fopen("ammeter_cal_log.txt","a");
      fprintf(fp,"read: scale=%d %d\n",*scale,*offset);
      fflush(fp);
      fclose(fp);
    }
  }
  return rc;
}

int8 EvseRapi::GetChargingCurrent(int *ma)
{
  strcpy(m_cmdBuf+1,"GG");
  int8 rc = runCommand();
  if (!rc) {
    if (sscanf(m_respBuf+4,"%d",ma) != 1) {
      rc = 1;
    }
  }
  return rc;
}




int main(int argc,char *argv[])
{
  int rc;

  printf("Lincomatic OpenEVSE Ammeter Calibrator %s  %s %s\n\n",VERSTR,__DATE__,__TIME__);
 
  char s[128];
  EvseRapi rapi;

  if (argc != 2) {
    printf("Usage: %s commport\n",argv[0]);
    printf("e.g. %s COM3\n",argv[0]);
    rc = 1;
    goto bye;
  }

  sprintf(s,"%s:",argv[1]);
  if (rapi.OpenLink(s)) {
    printf("ERROR opening %s\n",argv[1]);
    rc = 2;
    goto bye;
  }

  if (rapi.GetVersion(s)) {
    printf("ERROR getting version\n");
    rc = 3;
    goto bye;
  }
  printf("OpenEVSE Firmware/RAPI Version: %s\n",s);

  printf("\nEnabling ammeter calibration\n");
  if (rapi.EnableAmmeterCal(1)) {
    printf("ERROR enabling ammeter calibration\n");
    rc = 4;
    goto bye;
  }

  printf("\nGetting current ammeter settings\n");
  int scale,offset;
  if (rapi.GetAmmeter(&scale,&offset)) {
    printf("ERROR getting current ammeter settings\n");
    rc = 5;
    goto bye;
  }
  printf("current settings:  scale=%d offset=%d\n",scale,offset);

  printf("\n1. Manually enter new scale & offset\n2. Calibrate\n3. Quit\nEnter selection: ");
  int sel;
  scanf("%d",&sel);
  if (sel == 1) {
    printf("Enter scale: ");
    scanf("%d",&scale);
    printf("Enter offset: ");
    scanf("%d",&offset);

    printf("Save scale=%d offset=%d (y/n)? ",scale,offset);
    do {scanf("%c",&sel);} while(!isalpha(sel));
    if (tolower(sel) == 'y') {

      printf("\nSaving ammeter settings: scale=%d offset=%d\n",scale,offset);
      if (rapi.SetAmmeter(scale,offset)) {
	printf("ERROR saving ammeter settings\n");
	rc = 5;
	goto bye;
      }
    }
    printf("\nSuccess\n");
  }
  else if (sel == 2) {
    int oscale=scale,ooffset=offset;
    if (rapi.EnableAmmeterCal(1)) {
      printf("ERROR enabling ammeter calibration\n");
      rc = 1;
      goto bye;
    }
    if (rapi.SetAmmeter(1,0)) {
      printf("ERROR setting ammeter for calibration\n");
      rc = 1;
      goto bye;
    }
    printf("\nMake sure there is no load current on EVSE. Continue (y/n)? ");
    do {scanf("%c",&sel);} while(!isalpha(sel));
    if (tolower(sel) == 'y') {
      printf("\nCalculating zero offset...");
      SLEEP(5);
      int toffset;
      if (rapi.GetChargingCurrent(&toffset)) {
	printf("ERROR getting charging current\n");
	rc = 1;
	goto bye;
      }
      printf("%d...done\n",toffset);
      
      printf("\nApply a known load current of at least 5A to the EVSE. Continue (y/n)? ");
      do {scanf("%c",&sel);} while(!isalpha(sel));
      printf("\nMeasuring load current...");
      SLEEP(5);
      int c;
      if (rapi.GetChargingCurrent(&c)) {
	printf("ERROR getting charging current\n");
	rc = 1;
	goto bye;
      }
      double dc = (double)c;
      double dto = (double)toffset;
      printf("%d...done\n",c);
      printf("\nEnter load current in amps which was applied above: ");
      double refamps;
      scanf("%lf",&refamps);
      printf("refamps: %lf\n",refamps);
      double m = (refamps * 1000.0) / (dc-dto);
      scale = (int)(m + .5);
      offset = - (int)(m*dto + .5);
      printf("\nSaving ammeter settings: scale=%d offset=%d\n",scale,offset);
      if (rapi.SetAmmeter(scale,offset)) {
	printf("ERROR saving ammeter settings\n");
	rc = 5;
	goto bye;
      }
      printf("\nSuccess\n");
    }
    else {
      printf("\nRestoring ammeter settings: scale=%d offset=%d\n",scale,offset);
      if (rapi.SetAmmeter(scale,offset)) {
	printf("ERROR saving ammeter settings\n");
	rc = 5;
	goto bye;
      }
    }
  }

  rc = 0;

 bye:
  if (rc != 1) {
    printf("Restarting EVSE\n");
    rapi.ResetEvse();
  };
  return rc;
}
