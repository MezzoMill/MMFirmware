/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "spindle_control.h"
#include "settings.h"
#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

uint8_t spindleEnabled;
int spindleDirection;
uint32_t spindleSpeed;

void spindle_init()
{
  spindleEnabled = 0;
  spindleSpeed = 0;
  spindleDirection = 0;
  SPINDLE_ENABLE_DDR |= 1<<SPINDLE_ENABLE_BIT;
}

void spindle_run(int direction, uint32_t rpm) 
{
  SPINDLE_ENABLE_PORT |= 1<<SPINDLE_ENABLE_BIT;
  spindleEnabled = 1;
  spindleSpeed = rpm;
  spindleDirection = direction;
  // wait a second for the spindle to spin up.
  _delay_ms(MOTOR_SPIN_UP_AND_DOWN_TIME);
}

void spindle_stop()
{
  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
  spindleEnabled = 0;
  spindleSpeed = 0;
  spindleDirection = 0;
  // wait a second for the spindle to spin down.
  _delay_ms(MOTOR_SPIN_UP_AND_DOWN_TIME);
}

void spindle_pause()
{
  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
  // wait a second for the spindle to spin down.
  _delay_ms(MOTOR_SPIN_UP_AND_DOWN_TIME);
}

void spindle_resume()
{
  if(spindleEnabled)
  {
    spindle_run(spindleDirection, spindleSpeed);
  }
}