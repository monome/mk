/* 
 *  Copyright (C) 2006, Joe Lake, monome.org
 * 
 *  This file is part of 40h.
 *
 *  40h is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  40h is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with 40h; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  $Id: button.h,v. 1.1.1.1 2006/05/02 1:01:22
 */

#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <inttypes.h>

#define kButtonEventQueueSize 32

#define kButtonStateDown 1
#define kButtonStateUp   0

#define kButtonDownEvent 1
#define kButtonUpEvent   0

#define kButtonDownDefaultDebounceCount 1
#define kButtonUpDefaultDebounceCount   4

#define kButtonNewEvent   1
#define kButtonNoEvent    0

extern uint8_t button_current[32],             // bitmap of physical button state (depressed or released)
             button_last[32],                // bitmap of physical button state last time buttons were read
			button_state[32],              // bitmap of debounced button state
   			button_debounce_count[32][8],   // debounce counters for each button
             button_event[32];               // queued button events (kButtonDownEvent or kButtonDownEvent)

void buttonInit(void);
void buttonCheck(uint8_t row, uint8_t index);


#endif
