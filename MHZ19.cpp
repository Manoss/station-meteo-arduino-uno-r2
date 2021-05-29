/**************************************************************************/
/*!
@file     MHZ19.h
@author   E.KOPP-ROBERT
@license  GNU GPLv3

First version of an Arduino Library for the MH-Z19 CO2 sensor

@section  HISTORY

v1.0 - First release
*/
/**************************************************************************/

#include "MHZ19.h"

float getPPM( long duration) {
  //convert microseconds to milliseconds
  float durationToMs = duration/1000;
  //calculate CO2 for 0-5000 ppm
  return 5000*(durationToMs-2)/1000;  
}