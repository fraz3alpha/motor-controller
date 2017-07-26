/*
 * Motor Controller sketch for H Bridge control via HIP4081 
 * 
 * 20161001
 * andrewtaylor2@gmail.com
 * 
 */

/*
 * Summary of hardware features used:
 * [x] Timer 1: RC input pulse capture
 * [ ] Timer 3: RC input pulse capture (channel 2)
 * [x] Timer 4: Complementary PWM generation with deadtime for bridge driver
 */

/*
 * Common macros
 */
/*
 * Clear bit
 */
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

/*
 * Set bit
 */
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

/*
 * Debug pulse macros
 */
#define configure_debug_pulse_pin pinMode(3, OUTPUT)
#define set_debug_pulse_high sbi(PORTD, PD0)
#define set_debug_pulse_low cbi(PORTD, PD0)

/*
 * Timer/Counter4 is used to create complementary PWM output with dead time.
 * 
 * Complete independent control of gate inputs is retained, not needing to rely
 * on the deatime provided by the driver IC
 * 
 * 
 * Pins:
 *  OC4A  >  ___  (PC7 - IO 13)
 * #OC4A  >  ___  (PC6 - IO 5)
 * 
 *  OC4B  >  ___  (PB6 - IO 10) BHI
 * #OC4B  >  ___  (PB5 - IO 9)  BLI
 * 
 *  OC4D  >  ___  (PD7 - IO 6)  ALI
 * #OC4D  >  ___  (PD6 - IO 12) AHI
 * 
 * Pay particular note that the HIGH/LOW outputs are inverted between channel A and B
 * for circuit layout reasons. Hopefully this is not a massive headache
 */
#define ALI 6
#define AHI 12

#define BLI 9
#define BHI 10

// Pull the Disable line low to activate the chip
#define DIS 5

// ADCs
#define CURRENT_SENSE_PIN A0
#define BATT_VOLTAGE_PIN A2
#define CURRENT_ADJUST_PIN A3

/*
 * Current velocity information
 * 
 * direction: -1,1. 
 *  -1 => reverse
 *  +1 => forwards
 *  
 * speed: 0->255
 *    0 => stopped
 *  255 => full forwards
 *
 */
int16_t current_velocity = 0;
int16_t commanded_velocity = 0;

#define MAX_SPEED_DELTA 5


// Configure the PWM clock fot Timer 4
void pwm_configure()
{
  // Frequency modes for TIMER4
  #define PWM187k 1   // 187500 Hz
  #define PWM94k  2   //  93750 Hz
  #define PWM47k  3   //  46875 Hz
  #define PWM23k  4   //  23437 Hz
  #define PWM12k  5   //  11719 Hz
  #define PWM6k   6   //   5859 Hz
  #define PWM3k   7   //   2930 Hz
  // Hardcode the mode
  int mode=PWM23k;
  
  // Initial speeds to zeo
  // Side A
  OCR4D=0x0;
  // Side B
  OCR4B=0x0;
  // The unused channel
  OCR4A=0x0;
  
  // TCCR4A configuration
  TCCR4A=0;
  
  // TCCR4B configuration
  // mode plus maximum dead time precale(8x)
  TCCR4B=mode | 0x30;
  
  // TCCR4C configuration
  TCCR4C=0;
  
  // TCCR4D configuration
  TCCR4D=0x01;
  
  // Dead time configuration
  // 0xFF = max deadtime
  DT4=0xFF;
  
  // PLL Configuration
  // Use 96MHz / 2 = 48MHz
  PLLFRQ=(PLLFRQ&0xCF)|0x30;
  // PLLFRQ=(PLLFRQ&0xCF)|0x10; // Will double all frequencies
  
  // Terminal count for Timer 4 PWM
  OCR4C=255;

  TCCR4A=B00010001;  // Activate channel B with complementary output (Disables channel A)
  TCCR4C=B00010101;  // Activate channel D with complementary output (Overwrites shadow bits for channel B)

}

 /*
  * Timer/Counter1 is used to measure the incoming RC pulse
  * 
  * Pins:
  *  ICP1 > PD4 - IO4
  *  
  * Clear timer on rising edge
  * Read capture timer on falling edge
  * Error flagged on timer overflow
  * 
  * Pulse validator
  * 1. The pulse must fall between the generally acceptable limits of 0.8,s to 2.2ms
  *    which encompasses the general width of 1ms to 2ms (with a nomimal 1.5ms centre)
  *    whilst allowing for outside cases
  * 2. There must have been at least 10 valid pulses in the last second for the current
  *    value to be considered reasonable.
  * 3. The last 4 pulses must have been good for the current one to be considered
  *    reasonable
  */

// Interrupts for timer1
volatile uint16_t last_sample = 0;
volatile uint8_t rising;
volatile uint32_t overflow_events = 0;
volatile uint32_t overflow_events_since_good_pulse = 0;
#define OVERFLOW_EVENTS_BEFORE_BAD_PULSE 50
volatile uint32_t rejected_pulses = 0;
volatile boolean pulse_is_bad = false;
#define PULSE_VALIDITY_HISTORY_LENGTH 4
volatile boolean pulse_validity_history[PULSE_VALIDITY_HISTORY_LENGTH];
volatile uint8_t pulse_validity_history_position = 0;
// Pulse shape in terms of timer1 count before scaling
#define MINIMUM_PULSE_WIDTH_COUNT 12800
#define MAXIMUM_PULSE_WIDTH_COUNT 35200
ISR(TIMER1_OVF_vect) {
  // The timer will overflow every 4.1ms at the 1x Fosc multiplier level
  overflow_events += 1;

  // Track the total number of overflow events since the last good pulse
  overflow_events_since_good_pulse += 1;
  // If we have waited too long for a good pulse, chalk it up as a bad pulse
  if (overflow_events_since_good_pulse > OVERFLOW_EVENTS_BEFORE_BAD_PULSE) {
    pulse_validity_history[pulse_validity_history_position++] = false;
    if (pulse_validity_history_position == PULSE_VALIDITY_HISTORY_LENGTH) {
      pulse_validity_history_position = 0;
    }
  }
  // If we are waiting for the pulse to finish, we've timed out
  if (rising == 0) {
    pulse_is_bad = true;
  }
}

// Interrupt capture handler
ISR(TIMER1_CAPT_vect) {

  // watch for the other edge to catch the half-pulse width
  if (rising) {
    // Now wait for the falling edge 
    TCCR1B &= ~(1<<ICES1); 
    TIFR1 |= (1<<ICF1); 
    rising = 0; 
    // Reset the timer to start counting the pulse length
    TCNT1H = 0;
    TCNT1L = 0;
    pulse_is_bad = false;

  } else {
    // Now wait for the next rising falling edge
    TCCR1B |= (1<<ICES1); 
    TIFR1 |= (1<<ICF1);
    rising = 1;
    if (pulse_is_bad) {
      rejected_pulses++;
      pulse_validity_history[pulse_validity_history_position++] = false;
      if (pulse_validity_history_position == PULSE_VALIDITY_HISTORY_LENGTH) {
        pulse_validity_history_position = 0;
      }
      pulse_is_bad = false;
    } else {
      byte l = ICR1L;   // grab captured timer value
      byte h = ICR1H;
      uint16_t timer_value = (h << 8) + l;
      // Check if pulse is valid 
      if (timer_value < MINIMUM_PULSE_WIDTH_COUNT || timer_value > MAXIMUM_PULSE_WIDTH_COUNT) {
        // If it isn't valid, reject this pulse and add it to the history
        rejected_pulses++;
        pulse_validity_history[pulse_validity_history_position++] = false;
        if (pulse_validity_history_position == PULSE_VALIDITY_HISTORY_LENGTH) {
          pulse_validity_history_position = 0;
        }
      } else {
        // If it is valid, make the value live and add it to the history
        last_sample = timer_value;
        pulse_validity_history[pulse_validity_history_position++] = true;
        overflow_events_since_good_pulse = 0;
        if (pulse_validity_history_position == PULSE_VALIDITY_HISTORY_LENGTH) {
          pulse_validity_history_position = 0;
        }
      }
    }
  }

}

// Trigger this interrupt whenever TIMER4 overflows, and we should trigger an ADC
// read to get the current flowing
ISR(TIMER4_OVF_vect) {

  // Trigger read of ADC

}

// Timer 0 overflows 1024 times per second, and we adjust the commanded speed herem
// according to the configured acceleration ramps, essentially how quickly the actual
// speed changes to match the commanded velocity
//ISR(TIMER0_OVF_vect) {
// 
//}

// Make the motor beep
void motor_tone(int duration) {
  current_velocity = 0;
  set_pwm_outputs();

  for (int i = 0; i< duration*5; i++) {
    current_velocity = 255;
    set_pwm_outputs();
    delay(100);
  
    current_velocity = -255;
    set_pwm_outputs();
    delay(100);
  }

  current_velocity = 0;
  set_pwm_outputs();
}

// Initial configuration
void setup() {

  // First thing we do is disable the driver chip, so all outputs are low
  pinMode(DIS, OUTPUT);
  digitalWrite(DIS, HIGH);

  // Enable the serial port, configure the debug LED, and flash it to show we are alive
  // Serial
  Serial.begin(57600);
  Serial.println("Motor Controller v1.3-20170306 by Andy Taylor");

  // Debug LED
  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);
  delay(100);
  
  // Wait for 5 seconds before doing anything
  for (int i=0; i< 25; i++) {  
    digitalWrite(A1, HIGH);
    delay(100);
    digitalWrite(A1, LOW);
    delay(100);
  }

  // Set the LED to on whilst we are good
  digitalWrite(A1, HIGH);


  // Configure Timer 4 which provides the PWM for the HIP4081
  pwm_configure();
  
  // Side A
  pinMode(ALI, OUTPUT);
  pinMode(AHI, OUTPUT);
  // Side B
  pinMode(BLI, OUTPUT);
  pinMode(BHI, OUTPUT);


  // Timer1 for RC capture
  TCCR1A = 0;
  TCCR1B = (0<<ICNC1) | (0<<ICES1) | (1<<CS10);
  TCCR1C = 0;

  //catchFallingEdge(); // initialize to catch
  { TCCR1B &= ~(1<<ICES1); TIFR1 |= (1<<ICF1); rising = 0; }

  // Interrupt setup
  // ICIE1: Input capture 
  // TOIE1: Timer1 overflow
  TIFR1 = (1<<ICF1) | (1<<TOV1);    // clear pending
  TIMSK1 = (1<<ICIE1) | (1<<TOIE1); // and enable

  // Enable TIMER4 overflow interrupt
  TIMSK4 |= 1<<TOIE4;


  // Enable the HIP4081
  // We should only do this while the input is not invalid
  digitalWrite(DIS, LOW);
  
}

void pulse_history_validity() {
  for (uint8_t i=0; i<PULSE_VALIDITY_HISTORY_LENGTH; i++) {
    Serial1.print(pulse_validity_history[i]);
  }
  Serial1.println();
}

boolean is_pulse_history_valid() {
  for (uint8_t i=0; i<PULSE_VALIDITY_HISTORY_LENGTH; i++) {
    if (!pulse_validity_history[i]) {
      return false;
    }
  }
  return true;
}



void update_velocity(boolean immediate) {
  /*
   * The speed will be changed by the maximum intervals as 
   * set by the configured rate parameters. Current options
   * are: 
   *  - LINEAR FIXED RATE
   */

  if (abs(commanded_velocity) < 16) {
    commanded_velocity = 0;
  } else if (commanded_velocity > 240) {
    commanded_velocity = 255;
  } else if (commanded_velocity < -240) {
    commanded_velocity = -255;
  }

  int16_t velocity_delta = commanded_velocity - current_velocity;

  if (velocity_delta > MAX_SPEED_DELTA) {
    velocity_delta = MAX_SPEED_DELTA;
  }
  if (velocity_delta < -MAX_SPEED_DELTA) {
    velocity_delta = -MAX_SPEED_DELTA;
  }

  current_velocity += velocity_delta;

  if (immediate) {
    current_velocity = commanded_velocity;
  }

  /* 
   * Depending on the dead time settings, then certain values aren't going to giv 
   * the MOSFETs time to turn on, and in particular at certain high duty cycles
   * there is a turnout with out the complementary MOSGET being turned on at all.
   * 
   * For this reason certain values are skipped.
   * 
   * e.g. with the maximum dead time the minimum value should be 15, and max of 240,
   * after which the value jumps to zero or maximum (255)
   */


  
  set_pwm_outputs();

}

void set_pwm_outputs() {
  /*
   * Set PWM outputs according to the current velocity request
   */
  if (current_velocity > 0) {
    OCR4D = 0;
    // B is inverted because of wiring
    OCR4B = 255 - current_velocity;
  } else {
    OCR4D = abs(current_velocity);
    // B is inverted because of wiring
    OCR4B = 255;
  }
}

void activate_failsafe() {

  commanded_velocity = 0;
  current_velocity = 0;
  set_pwm_outputs();
  
}

uint16_t pulse_lower_limit = 17900;
uint16_t pulse_upper_limit = 30100;
uint16_t pulse_total_range = pulse_upper_limit - pulse_lower_limit;
uint16_t pulse_midpoint = pulse_lower_limit + pulse_total_range/2.0;
  
int16_t parse_pulse(int16_t pulse) {
  /*
   * Scale the pulse
   */
  double parsed_pulse = pulse;
  parsed_pulse -= pulse_midpoint;
  double scaling_factor = 512.0 / pulse_total_range;
  parsed_pulse *= scaling_factor;
  
  /*
   * Coerce the pulse into the valid range.
   * 
   * It is possible that with deadtime requirements that we may
   * have to jump from, say, 95% to 100% to avoid FET driving overlap
   */
  if (parsed_pulse < -255) {
    parsed_pulse = -255;
  }
  if (parsed_pulse > 255) {
    parsed_pulse = 255;
  }

  return parsed_pulse;  
  
}

/*
 * The input could (potentially) come from a number of sources
 */
#define SERIAL_INPUT_STRING_MAX_POSITION 5
byte serial_input_string[SERIAL_INPUT_STRING_MAX_POSITION];
byte serial_input_string_position = 0;
boolean serial_input_string_is_junk = false;
void parse_input() {

  /*
   * Case 0: Debug
   */
  // Half speed, forward
  // commanded_velocity = -127;

  /*
   * Case 1: RC pulse
   */

  /*
   * Get the latest recorded sample
   */
  /*
   * Check the sample is within acceptable limits
   */
  /*
   * Scale pulse to -255 <> 255
   */
//  if (is_pulse_history_valid()) {
//    commanded_velocity = parse_pulse(last_sample);
//  } else {
//    activate_failsafe();
//  }

  /*
   * Case 2: Serial input
   * 
   * This input allows easy communication with a PC or microcontroller
   * via text. This will parse everything until a newline character
   */
  while (Serial1.available()) {
    byte c = Serial1.read();
    // Serial.println(c);
    // If we've overflowed, then mark the next time we receive a new line character
    // as potential junk
    if (serial_input_string_position == SERIAL_INPUT_STRING_MAX_POSITION) {
      serial_input_string_is_junk = true;
      serial_input_string_position = 0;
      continue;
    }
    serial_input_string[serial_input_string_position++] = c;
    // Parse the value when we get to a newline
    if (c == 0xA) {
      if (serial_input_string_position >1 && !serial_input_string_is_junk) {
        // Serial.println("Parsing input");
        // Parse the data so far
        long value = 0;
  
        // Work backwards through the values we have parsed
        // Numbers should be betweeen 0-9, or '-'.
        // Numbers are whole. Each position is a x10 multiplier
        unsigned int multiplier = 1;
        for (int i=serial_input_string_position-2 ; i >= 0; i--) {
          if (serial_input_string[i] <= '9' && \
              serial_input_string[i] >= '0') {
            // digit
            value += multiplier * (serial_input_string[i] - '0');
            multiplier *= 10;
          } else if (serial_input_string[i] == '-') {
            // sign
            value *= -1;
          } else {
            // Unknown, it's junk
            break;
          }
          
        }
        // If the value is within limits, set it as our velocity
        if (value >= -255 && value <= 255) {
          Serial1.println("Y");
          commanded_velocity = value;
        }
      }  
      // Reset position marker
      serial_input_string_position = 0;
      serial_input_string_is_junk = false;
      
    }
  }
}

// Main Loop
uint8_t loop_counter = 0;

int8_t demo_speed = 0;


void set_speed(int speed) {

  // Check the speed is in bounds, else stop
  if (speed > 255 || speed < -255) {
    speed = 0;
  }

  // Set the speed as appropriate
  if (speed > 0) {
    // Forward
    OCR4D = 255;
    OCR4B = speed;
    
  } else if (speed < 0) {
    // Backwards
    OCR4D = 255 + speed;
    OCR4B = 0;
  } else {
    // Stopped
    
    // Lowside on
    OCR4D = 255;
    OCR4B = 0;
  }

  Serial1.println(speed);
  
}

/*
 * Loop runs at nominally 1000Hz
 */
void loop() {

  while (true) {

//    Serial1.println("127, 127");
//    OCR4D = 127; // A
//    OCR4B = 127; // B
//    delay(5000);

    // D=0 => ALI=0 AHI=1 : AH was indeed on
    // B=0 => BLI=1 BHI=0 : BL was indeed on

    // D=0 => ALI=0 AHI=1 : AH was indeed on
    // B=1 => BLI=0 BHI=1 : BH was indeed on

    // Current limit
    // Bootstrap voltages all over the place
    // D=1 => ALI=1 AHI=0 : AH was 
    // B=1 => BLI=0 BHI=1 : BH was 

    // OK when I remove AHI
    // D=1 => ALI=1 AHI=0 : AL was on 
    // B=0 => BLI=1 BHI=0 : BL was on

//    Serial1.println("  0, 0");
//    OCR4D = 255; // A
//    for (byte i=0; i<=255; i++){
//      OCR4B = i; // B
//      delay(50);
//    }
//    OCR4B = 255; 
// B
//    delay(5000);


      set_speed(0);
      delay(250);
      
      set_speed(128);
      delay(250);
      set_speed(255);
      delay(250);
      set_speed(128);
      delay(250);
      set_speed(0);  

      delay(250);

      set_speed(-128);
      delay(250);
      set_speed(-255);
      delay(250);
      set_speed(-128);
      delay(250);
      set_speed(0);  

//          
//      // Lowside on
//      OCR4D = 255;
//      OCR4B = 0;
//      delay(500);
//
//      // Forward
//      OCR4D = 0;
//      OCR4B = 0;
//      delay(500);
//
//      // Backward
//      OCR4D = 255;
//      OCR4B = 255;
//      delay(500);


//    Serial1.println("255, 0");
//    OCR4D = 255; // A
//    OCR4B = 0; // B
//    delay(5000);

//    Serial1.println("  0, 255");
//    OCR4D = 0; // A
//    OCR4B = 255; // B
//    delay(5000);
    
//    Serial1.println("255, 255");
//    OCR4D = 255; // A
//    OCR4B = 255; // B
//    delay(5000);
  }

//#define MAX_SPEED 255
//  while(true) {
//    for (int i=0; i<MAX_SPEED; i++) {
//      commanded_velocity=i;
//      update_velocity(true);
//      Serial1.println(analogRead(CURRENT_SENSE_PIN));
//      delay(50);
//    }
//    for (int i=MAX_SPEED; i>-MAX_SPEED; i--) {
//      commanded_velocity=i;
//      update_velocity(true);
//      Serial1.println(analogRead(CURRENT_SENSE_PIN));
//      delay(50);
//    }
//    for (int i=-MAX_SPEED; i<0; i++) {
//      commanded_velocity=i;
//      update_velocity(true);
//      Serial1.println(analogRead(CURRENT_SENSE_PIN));
//      delay(50);
//    }
//  }

  /*
   * Set debug sync pulse
   */
  set_debug_pulse_high;

  commanded_velocity = -128;
  
  /*
   * Parse incoming pulses every loop
   */
//  parse_input();

  /*
   * Periodically adjust velocity - nominally every 8ms
   */
//  if (loop_counter % 8) {
    update_velocity(true);
//  }

  /*
   * Occasionally debug print our current velocity details
   * This takes a long time in processor terms
   */
  if (loop_counter % 128) {
    Serial1.print(commanded_velocity);
    Serial1.print("\t-\t");
    Serial1.println(current_velocity);
  }
  
  /*
   * Clear debug sync pulse
   */
  set_debug_pulse_low;

  delay(100);
  loop_counter += 1;
  
}
