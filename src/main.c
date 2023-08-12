/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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

#include "grbl.h"


// Declare system global variable structure
system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
  volatile uint8_t sys_rt_exec_debug;
#endif
volatile unsigned long micros;
volatile unsigned long millis;
unsigned long millis_timer;

unsigned long z_step_timer;
unsigned long arc_stablization_timer;
volatile int z_step_delay;

// Value to store analog result
volatile uint16_t analogVal;
volatile uint16_t analogSetVal;

void thc_update()
{
  if(PINC & (1<<PC1))
  {
    //We don't have an arc_ok signal
    jog_z_up = false;
    jog_z_down = false;
    arc_stablization_timer = millis;
  }
  else
  {
    //We have an arc_ok signal!
    //Out ADC input is 2:1 voltage divider so pre-divider is 0-10V and post divider is 0-5V. ADC resolution is 0-1024; Each ADC tick is 0.488 Volts pre-divider (AV+) at 1:50th scale!
    //or 0.009 volts at scaled scale (0-10)
    //Wait 3 secends for arc voltage to stabalize
    if ((millis - arc_stablization_timer) > 3000)
    {
      if (analogSetVal > 30) //THC is turned on
      {
        if ((analogVal > (analogSetVal - 10)) && (analogVal < (analogSetVal + 10))) //We are within our ok range
        {
          jog_z_up = false;
          jog_z_down = false;
        }
        else //We are not in range and need to deterimine direction needed to put us in range
        {
          if (analogVal > analogSetVal) //Torch is too high
          {
            jog_z_down = true;
          }
          else //Torch is too low
          {
            jog_z_up = true;
          }
        }
      }
    }
  }
}

unsigned long cycle_frequency_from_feedrate(double feedrate)
{
  return ((1000.0f * 1000.0f) / (settings.steps_per_mm[Z_AXIS])) / feedrate;
}
ISR(ADC_vect){
  // Must read low first
  analogVal = ADCL | (ADCH << 8);
  // Not needed because free-running mode is enabled.
  // Set ADSC in ADCSRA (0x7A) to start another ADC conversion
  // ADCSRA |= B01000000;
}
//Fires every 1/8 of a ms, 125uS
ISR(TIMER2_OVF_vect){
  if ((micros - z_step_timer) > z_step_delay)
  {
    if (jog_z_up)
    {
      //Dir
      if (settings.dir_invert_mask & (1 << 2)) //Z dir is inverted
      {
        PORTD |= (1 << PD7);    // set pin A2 high
        _delay_us(10);
      }
      else
      { 
        PORTD &= ~(1 << PD7);    // set pin 7 low
        _delay_us(10);
      }
      //Step
      PORTD |= (1 << PD4);     // set pin A2 high
      _delay_us(10);
      PORTD &= ~(1 << PD4);    // set pin A2 low
      sys_position[Z_AXIS]++;
    }
    else if (jog_z_down)
    {
      if (settings.dir_invert_mask & (1 << 2)) //Z dir is inverted
      {
        //Dir
        PORTD &= ~(1 << PD7);    // set pin 7 low
        _delay_us(10);
      }
      else
      {
        PORTD |= (1 << PD7);     // set pin A2 high
        _delay_us(10);
      }
      //Step
      PORTD |= (1 << PD4);     // set pin A2 high
      _delay_us(10);
      PORTD &= ~(1 << PD4);    // set pin A2 low
      sys_position[Z_AXIS]--;
    }
    z_step_timer = micros;
  }

  //Timing critical
  if (millis_timer > 7) //8 cycles is one millisecond
  {
    if (machine_in_motion == true) thc_update(); //Once a millisecond, evaluate what the THC should be doing
    millis_timer = 0;
    millis++;
  }
  TCNT2 = 223;           //Reset Timer to 130 out of 255
  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  micros += 125;
  millis_timer++;
}


int main(void)
{
  arc_stablization_timer = 0;
  /* Begin ADC Setup */
  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= 0b11011111;
  // Set REFS1..0 in ADMUX (0x7C) to change reference voltage to the
  // proper source (01)
  ADMUX |= 0b01000000;
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= 0b11110000;
  // Set MUX3..0 in ADMUX (0x7C) to read from AD8 (Internal temp)
  // Do not set above 15! You will overrun other parts of ADMUX. A full
  // list of possible inputs is available in Table 24-4 of the ATMega328
  // datasheet
  ADMUX |= 8;
  // ADMUX |= B00001000; // Binary equivalent
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= 0b10000000;
  // Set ADATE in ADCSRA (0x7A) to enable auto-triggering.
  ADCSRA |= 0b00100000;
  // Clear ADTS2..0 in ADCSRB (0x7B) to set trigger mode to free running.
  // This means that as soon as an ADC has finished, the next will be
  // immediately started.
  ADCSRB &= 0b11111000;
  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA |= 0b00000111;
  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= 0b00001000;
  // Enable global interrupts
  // AVR macro included in <avr/interrupts.h>, which the Arduino IDE
  // supplies by default.
  sei();
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |=0b01000000;
  /* End ADC Setup */

  //Setup Timer2 to fire every 1ms
  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
  TCNT2  = 130;         //Reset Timer Count to 130 out of 255
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;        //Timer2 Control Reg A: Wave Gen Mode normal
  TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prescaler set to 128
  micros = 0;
  millis = 0;
  millis_timer = 0;

  z_step_timer = 0;
  z_step_delay = 0;

  
  DDRC &= ~(1<<DDC1); // Set A1 as input for Arc Ok
  PORTC |= (1<<PC1);  // Set A1 internally pulled-up

  //Start first ADC conversion
  ADMUX = (ADMUX & 0xF0) | (0 & 0x0F);
  ADCSRA |= (1<<ADSC);

  // Initialize system upon power-up.
  serial_init();   // Setup serial baud rate and interrupts
  settings_init(); // Load Grbl settings from EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers
  system_init();   // Configure pinout pins and pin-change interrupt



  memset(sys_position,0,sizeof(sys_position)); // Clear machine position.
  sei(); // Enable interrupts

  // Initialize system state.
  #ifdef FORCE_INITIALIZATION_ALARM
    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
    sys.state = STATE_ALARM;
  #else
    sys.state = STATE_IDLE;
  #endif
  
  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif

  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
  // will return to this loop to be cleanly re-initialized.
  for(;;) {

    // Reset system variables.
    uint8_t prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
    sys.state = prior_state;
    sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
		memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
    sys_probe_state = 0;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys_rt_exec_motion_override = 0;
    sys_rt_exec_accessory_override = 0;

    // Reset Grbl primary systems.
    serial_reset_read_buffer(); // Clear serial read buffer
    gc_init(); // Set g-code parser to default state
    spindle_init();
    coolant_init();
    limits_init();
    probe_init();
    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();

    // Print welcome message. Indicates an initialization has occured at power-up or with a reset.
    report_init_message();

    // Start Grbl main loop. Processes program inputs and executes them.
    
    PORTB &= ~(1 << PB0); //Set torch pin off
    z_step_delay = cycle_frequency_from_feedrate((12.0f / 60.0f));
    protocol_main_loop();

  }
  return 0;   /* Never reached */
}
