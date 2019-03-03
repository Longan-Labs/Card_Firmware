/*
  RTC_DS1307_Arduino.h

  This is a Suly compatible Library

  2014 Copyright (c) Seeed Technology Inc.  All right reserved.

  Loovee
  2014-4-17

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, digital_compass_write_reg to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __RTC_DS1307_ARDUINO_H__
#define __RTC_DS1307_ARDUINO_H__

#include "soft_i2c.h"
#include "RTC_DS1307_Suli.h"


class RTC{

public:

    void begin()
    {
        rtc_ds1307_init();
        delay(10);
        rtc_ds1307_start();
    }
    
    void set(struct __time t)
    {
        rtc_ds1307_set_time(t);
    }
    
    void get(struct __time *t)
    {
        rtc_ds1307_get_time(t);
    }

};



#endif