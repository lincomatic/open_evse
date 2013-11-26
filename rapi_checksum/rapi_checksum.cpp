// -*- C++ -*-
/*
 * Open EVSE RAPI Command Generator
 *
 * This program generates a RAPI checksum
 *
 * Copyright (c) 2013 Sam C. Lin <lincomatic@gmail.com>
 *
 * This file is part of Open EVSE.

 * Open EVSE is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.

 * Open EVSE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with Open EVSE; see the file COPYING.  If not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */



#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <string.h>


typedef char int8;
typedef unsigned char uint8;
typedef short int16;
typedef unsigned short uint16;
typedef int int32;
typedef unsigned int uint32;

#define ESRAPI_SOC '$'
#define ESRAPI_EOC 0xd // CR


char cmdBuf[80];
uint8 calcChkSum()
{
  char *s = cmdBuf;
  uint8 chkSum = 0;
  while (*s) {
    chkSum += *(s++);
  }
  return chkSum;
}

void appendChkSum()
{
  char *s = cmdBuf;
  uint8 chkSum = 0;
  while (*s) {
    chkSum += *(s++);
  }
  sprintf(s,"*%02X%c",(unsigned)chkSum,ESRAPI_EOC);
}

int main(int argc,char *argv[])
{
  printf("Lincomatic OpenEVSE RAPI Checksum Generator V1.0 (%s %s)\n\n",__DATE__,__TIME__);
  printf("Enter a RAPI command line:\n");
  for (;;) {
    gets(cmdBuf);
    if (cmdBuf[0] != '$') return 0;
    appendChkSum();
    printf("%s\n",cmdBuf);
  }

  return 0;
}
