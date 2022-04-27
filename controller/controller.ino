#include <Encoder.h>
#include <stdint.h>
#include "MegaJoy.h"

/**
 * Check if a timer has passed a debounce period
 * @param last The timer holder
 * @return 1 If debounced, 0 otherwise
 */
int is_debounced(uint32_t* last);

// Enables displaying switch states (disable on final upload)
// #define DEBUG_INFO

#ifdef DEBUG_INFO
#include <Arduino_AVRSTL.h>
#endif

// How long a button is held on a controller output
#define BTN_PULSE_TIME 150 //ms

// How many turns before encoder rotation will be detected
#define ENC_SENSITIVITY 4 //turns

// The delay (in ms) between detected button presses (for debouncing)
#define DEBOUNCE 20 //ms

// ---- Pinouts -----
Encoder enc0(3, 4);
Encoder enc1(6, 7);
Encoder enc2(9, 10);
Encoder enc3(12, 13);
Encoder encs[] = { enc0, enc1, enc2, enc3 };
long sw_outs[] = { 22, 26, 30, 34, 48 };    // Define the pins that output current (can be empty)
long mom_sw[] = { 2, 5, 8, 11, 49 };        // Define the pins to be treated as momentary switches
long tog_sw[] = { 25, 29, 33, 37 };         // Define the pins to be treated as toggle switches
// ------------------

// ------ Array length constants ------
#define NUM_ENCS (sizeof(encs)/sizeof(Encoder))   // The number of encoders
#define NUM_MOM_SW (sizeof(mom_sw)/sizeof(long))  // Number of momemtary switches
#define NUM_TOG_SW (sizeof(tog_sw)/sizeof(long))  // Number of toggle switches
#define NUM_SW_OUT (sizeof(sw_outs)/sizeof(long)) // Number of pins outputting power
// ------------------------------------

void setup() {
  int i;
  // Init the momentary switches to INPUT_PULLUP mode
  for (i = 0; i < NUM_MOM_SW; ++i){
    pinMode(mom_sw[i], INPUT_PULLUP); 
  }
  // And the toggles
  for (i = 0; i < NUM_TOG_SW; ++i){
    pinMode(tog_sw[i], INPUT_PULLUP);
  }

  // Init the output pins for each switch
  for (i = 0; i < NUM_SW_OUT; ++i){
    pinMode(sw_outs[i], OUTPUT);
    digitalWrite(sw_outs[i], LOW);
  }

  setupMegaJoy();

  #ifdef DEBUG_INFO
  Serial.begin(9600);
  #endif
}

// -------- Debounce Timers --------------
uint32_t mom_sw_last[NUM_MOM_SW] = {0};
uint32_t tog_sw_last[NUM_TOG_SW] = {0};
uint32_t enc_last[NUM_ENCS] = {0};
// ---------------------------------------

// -------- Toggle Switch ----------------
uint8_t tog_sw_btn_state[NUM_TOG_SW] = {0}; // Stores the button state of a toggle switch
uint32_t tog_sw_pulse[NUM_TOG_SW] = {0};    // Stores the time a toggle pulse started
uint8_t tog_sw_state[NUM_TOG_SW] = {0};     // The physical state of the toggle
// ---------------------------------------

// -------- Momentary Switch -------------
uint8_t mom_sw_state[NUM_MOM_SW] = {0};
// ---------------------------------------

// --------- Encoder -------------
int16_t encoder_pos[NUM_ENCS] = {0}; // The actual rotational position of the encoder
uint32_t enc_pulse[NUM_ENCS] = {0};  // Stores the time an encoder "button" pulse started
uint8_t  enc_states[NUM_ENCS] = {0}; // Holds bitmasked data concerning encoder "button" states
// enc_state bitmask constants
#define ENC_BTN         1   // The emitted button state 
#define ENC_DIR         2   // The button to emit (0 for 'right', 1 for 'left')
#define ENC_PULSING     4   // 1 while button is in a pulse cycle
// --------------------------------

void loop() {
  int i; 
  uint8_t sw_state;
  uint32_t cur_time = millis();

  // ------ Toggle Switches -----
  for (i = 0; i < NUM_TOG_SW; ++i){
    // Check if the debouncing delay has elapsed
    sw_state = digitalRead(tog_sw[i]) == LOW ? 1 : 0;
    if (sw_state != tog_sw_state[i] && is_debounced(&tog_sw_last[i])){
      tog_sw_state[i] = sw_state;
      tog_sw_btn_state[i] = sw_state+1;
      tog_sw_pulse[i] = cur_time;

      #ifdef DEBUG_INFO
      printf("tog%d: state= %d, btn_state= %d\n", i, tog_sw_state[i], tog_sw_btn_state[i]);;
      #endif

      continue;
    }
    
    if (tog_sw_btn_state[i] && cur_time - tog_sw_pulse[i] > BTN_PULSE_TIME){
      tog_sw_btn_state[i] = 0;
      #ifdef DEBUG_INFO
      printf("tog%d: state= %d, btn_state= %d\n", i, tog_sw_state[i], tog_sw_btn_state[i]);
      #endif
    }
  }

  // ------ Momentary switches ----------
  for (i = 0; i < NUM_MOM_SW; ++i){
    sw_state = digitalRead(mom_sw[i]) == LOW ? 1 : 0;
    if (sw_state != mom_sw_state[i] && is_debounced(&mom_sw_last[i])){
      mom_sw_state[i] = sw_state;
        
      #ifdef DEBUG_INFO
      printf("mom%d: state= %d\n", i, mom_sw_state[i]);
      #endif
    }
  }

  // ------- Encoders -------
  int16_t read_val;
  for (i = 0; i < NUM_ENCS; ++i){
    uint32_t cur_time = millis();
    
    // Check for a _valid_ change in the encoder position
    read_val = (int16_t) encs[i].read();
    if (abs(read_val - encoder_pos[i]) > ENC_SENSITIVITY && is_debounced(&enc_last[i])){
      uint8_t enc_state = enc_states[i];
      uint8_t rot_dir = read_val - encoder_pos[i] > 0 ? 0UL : ENC_DIR;
      encoder_pos[i] = read_val;

      // Check if is not currently in a pulse OR if the direction is flipped
      if (!(enc_state & ENC_PULSING) || (enc_state & ENC_DIR) != rot_dir ){
        // Enable the controller button, setting direction if needed
        enc_states[i] |= (ENC_BTN | rot_dir | ENC_PULSING);
        // Start the timer on the pulse
        enc_pulse[i] = cur_time;

        #ifdef DEBUG_INFO
        Serial.print(cur_time);
        Serial.print(" | enc");
        Serial.print(i);
        Serial.print(": started pulse -- ");
        Serial.println((int16_t) enc_states[i]);
        #endif
        
        continue;
      }
    }

    // Check if its time to end a pulse
    if ((enc_states[i] & ENC_PULSING) && cur_time - enc_pulse[i] > BTN_PULSE_TIME){
      if (enc_states[i] & ENC_BTN){
        // Clear the button bit
        enc_states[i] &= ~(1UL);

        #ifdef DEBUG_INFO
        Serial.print(cur_time);
        Serial.print(" | enc");
        Serial.print(i);
        Serial.print(": button disabled -- ");
        Serial.println((int16_t) enc_states[i]);
        #endif
      }
      else{

        enc_states[i] = 0UL;
        #ifdef DEBUG_INFO
        Serial.print(cur_time);
        Serial.print(" | enc");
        Serial.print(i);
        Serial.print(": pulse completed -- ");
        Serial.println((int16_t) enc_states[i]);
        #endif
      }
    }
  }
  
  // Build/Send the updated controller data
  megaJoyControllerData_t controllerData = getControllerData();
  setControllerData(controllerData);
}

/**
 * Builds a set of controller data using the stored states of the physical inputs.
 * 
 * @returns A freshly built set of controller data
 */
megaJoyControllerData_t getControllerData(void){
  int i, offset, btn_bit;
  uint8_t btn1, btn2;
  megaJoyControllerData_t controllerData = getBlankDataForMegaController();
  
  // Start at bit 0 in the button array
  btn_bit = 0;
  
  // Assign the momentary switches to buttons
  for (i = 0; i < NUM_MOM_SW; ++i){
    // Assign the momentary switch state to the next available bit in buttonArray
    controllerData.buttonArray[btn_bit/8] |= (mom_sw_state[i] << (btn_bit % 8));
    btn_bit++;
  }
  
  // Assign the toggle switches to buttons
  for (i = 0; i < NUM_TOG_SW; ++i){ 
    btn1 = tog_sw_btn_state[i] & 1UL ? 1 : 0;
    btn2 = tog_sw_btn_state[i] & 2UL ? 1 : 0;
    // Assign toggle switches to the next available bit in button array
    controllerData.buttonArray[btn_bit/8]     |= (btn1 << (btn_bit % 8));
    controllerData.buttonArray[(btn_bit+1)/8] |= (btn2 << ((btn_bit+1) % 8));
    btn_bit += 2;  
  }

  for (i = 0; i < NUM_ENCS; ++i){
    uint8_t enc_state = enc_states[i];
    btn1, btn2;
    btn1 = 0; btn2 = 0;
    if (enc_state & ENC_BTN){
      btn1 = enc_state & ENC_DIR ? 1 : 0;
      btn2 = btn1 ? 0 : 1;
    }
    
    // Assign encoder 'buttons' to next bits
    controllerData.buttonArray[btn_bit/8]     |= (btn1 << (btn_bit % 8));
    controllerData.buttonArray[(btn_bit+1)/8] |= (btn2 << ((btn_bit+1) % 8));
    btn_bit+=2;
  }

  #ifdef DEBUG_INFO
  if (millis() % 100){
    for (i = 0; i < btn_bit; ++i){
      uint8_t cur_byte = i/8;
      if (controllerData.buttonArray[cur_byte] == 0){continue;}   
      uint8_t local_bit = i%8;
      if (local_bit == 0) printf("[Byte:%d = %d] ", cur_byte, controllerData.buttonArray[cur_byte]);
      printf(" [%d:%d] ", local_bit, (controllerData.buttonArray[cur_byte] & (1UL<<local_bit)) >> local_bit);
      if (local_bit == 7 || i == btn_bit-1) printf("\n");
    }
  }
  #endif

  return controllerData;
}


int is_debounced(uint32_t* last){
  uint32_t cur_time = millis();
  if (cur_time - *last > DEBOUNCE){
    *last = cur_time;
    return 1;
  }

  return 0;
}

