
#include <inttypes.h>
#include "button.h"


uint8_t button_current[32], 
      button_last[32], 
	  button_state[32], 
      button_debounce_count[32][8], 
      button_event[32];


/***************************************************************************************************
 *
 * DESCRIPTION: initializes all button debouncing variables.
 *
 * ARGUMENTS:
 *
 * RETURNS:
 *
 * NOTES:       
 *
 ****************************************************************************************************/

void buttonInit(void)
{
    uint8_t i;

    for (i = 0; i < 32; i++) {
        button_current[i] = 0x00;
        button_last[i] = 0x00;
        button_state[i] = 0x00;
        button_event[i] = 0x00;
    }
}


/***************************************************************************************************
 *
 * DESCRIPTION: debounces physical button states and queues up debounced button state change events
 *              in button_event array for outputting.
 *
 * ARGUMENTS:   row -   the row of buttons to be debounced.
 *              index - the index (column) of the button in the specified row to be debounced.
 *
 * RETURNS:
 *
 * NOTES:       we debounce buttons so that momentary, accidental changes in button state do not
 *              cause button press events to be reported.
 *
 ****************************************************************************************************/

void buttonCheck(uint8_t row, uint8_t index)
{
    if (((button_current[row] ^ button_last[row]) & (1 << index)) &&   // if the current physical button state is different from the
        ((button_current[row] ^ button_state[row]) & (1 << index))) {  // last physical button state AND the current debounced state

        if (button_current[row] & (1 << index)) {                      // if the current physical button state is depressed
            button_event[row] = kButtonNewEvent << index;              // queue up a new button event immediately
            button_state[row] |= (1 << index);                         // and set the debounced state to down.
        }
        else
            button_debounce_count[row][index] = kButtonUpDefaultDebounceCount;  // otherwise the button was previously depressed and now
                                                                                // has been released so we set our debounce counter.
    }
    else if (((button_current[row] ^ button_last[row]) & (1 << index)) == 0 &&  // if the current physical button state is the same as
             (button_current[row] ^ button_state[row]) & (1 << index)) {        // the last physical button state but the current physical
                                                                                // button state is different from the current debounce 
                                                                                // state...

        if (button_debounce_count[row][index] > 0 && --button_debounce_count[row][index] == 0) {  // if the the debounce counter has
                                                                                                  // been decremented to 0 (meaning the
                                                                                                  // the button has been up for 
                                                                                                  // kButtonUpDefaultDebounceCount 
                                                                                                  // iterations///

            button_event[row] = kButtonNewEvent << index;    // queue up a button state change event

            if (button_current[row] & (1 << index))          // and toggle the buttons debounce state.
                button_state[row] |= (1 << index);
            else
                button_state[row] &= ~(1 << index);
        }
    }
}
