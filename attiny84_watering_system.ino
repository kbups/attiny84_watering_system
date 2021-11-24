#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h> 
#include <util/atomic.h>

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// https://www.arduino.cc/en/Reference/Volatile
volatile bool f_wdt = 1;
volatile bool f_locked = false; // Indicates if a watering is in progress
volatile bool f_calibration = false; // Indicates if the calibration mode is active
volatile bool f_watering_threshold_min_auto = false; // Indicates that a watering, activated by the WATERING_THRESHOLD_WHEN_DRY threshold, is in progress or not
volatile bool f_watering_threshold_max_auto = false; // Indicates that a watering, activated by the WATERING_THRESHOLD_MAX threshold, is in progress or not
volatile int f_water = 0; // Watering counter that moves from ITER_WATERING_COUNTER to 0
volatile int f_watchdog_counter = 0; // Watchdogs counter

#define ITER_WATERING_COUNTER 3   // 3 x 8s = 40s
#define DEBOUNCE_INPUT_DELAY 500  // Debounce time in ms when clicking on the button

// 3600 -> 3 waterings per day
#define WATERING_THRESHOLD_WHEN_DRY 3600  // Number of watchdogs to reach for watering if the ground is wet
// 10800 at least one watering a day
#define WATERING_THRESHOLD_MAX 10800      // Number of watchdogs to reach for watering event if the ground is dry (ex: to watering one a day)
// 4 min watering
#define WATERING_DELAY 30                 // Watering duration (number of watchdogs before stopping the watering)


/* ATTINY84                  _____
 *                VCC    1 -|°    |- 14    GND
 * 10 | PCINT8  | PB0    2 -|     |- 13    PA0 | PCINT0 | 0
 *  9 | PCINT9  | PB1    3 -|     |- 12    PA1 | PCINT1 | 1
 * 11 | PCINT11 | PB3    4 -|     |- 11    PA2 | PCINT2 | 2
 *  8 | PCINT10 | PB2    5 -|     |- 10    PA3 | PCINT3 | 3
 *  7 | PCINT7  | PA7    6 -|     |-  9    PA4 | PCINT4 | 4
 *  6 | PCINT6  | PA6    7 -|_____|-  8    PA5 | PCINT5 | 5
 *                     
 */

/* ATTINY84                                       _____
 * (arduino 5V)                        VCC    1 -|°    |- 14    GND              (arduino GND)
 * (blue led)           10 | PCINT8  | PB0    2 -|     |- 13    PA0 | PCINT0 | 0 (button interrupt on/off watering)
 * (soil moisture VCC)   9 | PCINT9  | PB1    3 -|     |- 12    PA1 | PCINT1 | 1 (red led)
 * (arduino 10)         11 | PCINT11 | PB3    4 -|     |- 11    PA2 | PCINT2 | 2 (soil moisture digital pin)
 * (calibration pin)     8 | PCINT10 | PB2    5 -|     |- 10    PA3 | PCINT3 | 3 (relay VCC)
 * (green led)           7 | PCINT7  | PA7    6 -|     |-  9    PA4 | PCINT4 | 4 (arduino 13)
 * (arduino 11)          6 | PCINT6  | PA6    7 -|_____|-  8    PA5 | PCINT5 | 5 (arduino 12)
 */
#define LED_RED_PIN     1     // Red light
#define SOIL_D0_PIN     2     // Digital pin for the moisture sensor
#define RELAY_PIN       3     // Watering relay
#define LED_GREEN_PIN   7     // Green led
#define CALIBRATION_PIN 8     // Calibration pin (when connected to VCC, the calibration mode is activated)
#define SOIL_VCC_PIN    9     // VCC pin for the the moisture sensor
#define LED_BLUE_PIN    10    // Blue led
  
void blinkLed(unsigned int pin, int nb_iter) {
  pinMode(pin, OUTPUT);
  for (byte i = nb_iter ;  i > 0 ; i--){
     digitalWrite(pin, HIGH);
     delay(50);
     digitalWrite(pin, LOW); 
     delay(50);
  }
  
  pinMode(pin, INPUT); // Reduce power
}

/******************************************************************/
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {
  byte bb;
  int ww;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  ww = bb;

  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
} 

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt = 1;  // set global flag
}

ISR (PCINT0_vect) {}

void setup_interrupts() {
  pinMode(PCINT0, INPUT);        // Set the pin to input
  digitalWrite(PCINT0, HIGH);    // Activate internal pullup resistor 
  
  PCMSK0 |= bit (PCINT0);   //                                   Pin Change Mask Register
  GIFR   |= bit (PCIF0);    // clear any outstanding interrupts  General Interrupt Flag Register
  GIMSK  |= bit (PCIE0);    // enable pin change interrupts      General Interrupt Mask Register
  sei();                    // enable interrupts
}

void system_sleep() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    f_watchdog_counter++;
    // 10800 iterations of 8s per day
    if(f_watchdog_counter > WATERING_THRESHOLD_MAX + WATERING_DELAY) {
      f_watchdog_counter = WATERING_THRESHOLD_MAX + WATERING_DELAY;
    }
  }
  
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here, waiting for interrupt

  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA, ADEN);                   // switch Analog to Digitalconverter ON
}

void setup() {
  cbi(ADCSRA, ADEN);      // switch ADC OFF
  sbi(ACSR, ACD);         // switch Analog Compartaror OFF

  pinMode(CALIBRATION_PIN, INPUT);
  pinMode(SOIL_D0_PIN, INPUT);
  
  setup_watchdog(9);
  setup_interrupts();
}

/// doWatering
/// Starts watering and initialize the duration to ITER_WATERING_COUNTER
void _doWatering() {
  f_locked = true;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    f_water = ITER_WATERING_COUNTER;
  }
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);
  
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  delay(DEBOUNCE_INPUT_DELAY);
}

/// stopWatering
/// Stops watering and reset the counters
void _stopWatering() {
  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN, INPUT);
  
  digitalWrite(LED_RED_PIN, LOW);
  pinMode(LED_RED_PIN, INPUT); // Reduce power
  
  delay(DEBOUNCE_INPUT_DELAY);

  // Reset the counters
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    f_watchdog_counter = 0;
    f_water = 0;
  }
  f_watering_threshold_min_auto = false;
  f_watering_threshold_max_auto = false;
  f_locked = false;
}

/// is_dry
/// Returns TRUE is the ground is dry, otherwise returns FALSE
bool is_dry() {
  bool res = false;
  pinMode(SOIL_VCC_PIN, OUTPUT);

  digitalWrite(SOIL_VCC_PIN, HIGH);
  delay(100); // To be sure the sensor is right powering
  res = digitalRead(SOIL_D0_PIN) == 1;

  digitalWrite(SOIL_VCC_PIN, LOW);
  pinMode(SOIL_VCC_PIN, INPUT);

  return res;
}

void loop() {
  bool do_sleep = true;
  if (f_wdt > 0) { // watchdog signal
    blinkLed(LED_BLUE_PIN, 1);

    // === WATERING ============================================================================================
    // If watering is in progress
    // =========================================================================================================
    int water_counter = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      water_counter = f_water;
    }
    
    if(water_counter > 0) {
      water_counter--; // Decrement the watering counter
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        f_water = water_counter;
      }
      if(water_counter == 0) { // If the watering counter has been reached, then stops the watering
        _stopWatering();
      }
    }

    // === CALIBRATION MODE ====================================================================================
    // If watering is in progress
    // =========================================================================================================
    if (digitalRead(CALIBRATION_PIN) == HIGH) {
      do_sleep = false; // Block the sleep until the calibration mode is active
      f_calibration = true;

      // Set the red and green leds as outputs
      pinMode(LED_RED_PIN, OUTPUT);
      pinMode(LED_GREEN_PIN, OUTPUT);
      pinMode(SOIL_VCC_PIN, OUTPUT);
      
      // Swith ON the leds when the ground is dry only
      if (is_dry()) { // 1 = dry, 0 = wet
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_RED_PIN, HIGH);
      } else {
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_GREEN_PIN, HIGH);
      }
    }
    else {
      // In order to reduce power, reset the red and green leds because we are not in the calibration mode
      digitalWrite(LED_GREEN_PIN, LOW);
      if(f_calibration) {
        pinMode(LED_RED_PIN, INPUT);
        f_calibration = false;
      }
      pinMode(LED_GREEN_PIN, INPUT);
      pinMode(SOIL_VCC_PIN, INPUT);

      // === AUTO WATERING =====================================================================================
      // Watering only if the ground is wet after reached the WATERING_THRESHOLD_WHEN_DRY couter
      // OR if the counter has reached the WATERING_THRESHOLD_MAX counter
      // =======================================================================================================
      bool dry = is_dry();
      int watchdog_counter = 0;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // code with interrupts blocked (consecutive atomic operations will not get interrupted)
        watchdog_counter = f_watchdog_counter;
      }
      // Auto watering when the ground is dry and UC has reached the the min threshold
      if(!f_watering_threshold_min_auto && !f_locked && dry && watchdog_counter >= WATERING_THRESHOLD_WHEN_DRY) {
        f_watering_threshold_min_auto = true;
        _doWatering();
      }
      // Auto watering when the UC has reached the max threshold even if the ground is not dry
      else if(!f_watering_threshold_max_auto && !f_locked && watchdog_counter >= WATERING_THRESHOLD_MAX) {
        f_watering_threshold_max_auto = true;
        _doWatering();
      }
    }
    
    f_wdt = do_sleep ? 0 : 1;  // reset flag
  }
  else { // PCINT0 occured
    if(!f_locked) {
      _doWatering();
    }
    else {
      _stopWatering();
    }
  }

  if(do_sleep) {
    system_sleep();
  }
}