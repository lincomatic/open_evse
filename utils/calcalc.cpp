/*
 * Author: Sam C. Lin 20200515
 * Copyright (c) Sam C. Lin 2020 ALL RIGHTS RESERVED
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

int main(int argc,char *argv[])
{
  printf("RAPI Calibration Calculator V1.0 (%s %s)\n\n",__DATE__,__TIME__);
  printf("Enter raw1 measured1 raw2 measured2\n");
  char cmdbuf[80];
  gets_s(cmdbuf,sizeof(cmdbuf));
  double raw1,raw2,measured1,measured2,scale,offset;
  sscanf(cmdbuf,"%lf %lf %lf %lf",&raw1,&measured1,&raw2,&measured2);
  printf("raw1: %lf measured1: %lf\n",raw1,measured1);
  printf("raw2: %lf measured2: %lf\n",raw2,measured2);
  scale = (measured2-measured1)/(raw2-raw1);
  offset = measured2 - (raw2*scale);
  printf("\nscale: %d\noffset: %d\n",(int)(scale+0.5),(int)(offset+0.5));
}
