#include "m_general.h"
#include "m_usb.h"

// IO
// ---------------------
// D6 (ADC9) - PID (ADC input)
// D7 (OC4D) - BUCK_CTRL (PWM output)
// B5 (OC1A) - L-R
// B6 (OC1B) - H-R
// B7 (OC1C) - L-L
// C6 (OC3A) - H-L
// B0 (GPIO) - LED1
// B1 (GPIO) - LED2

// COMM PROTOCOL
// ---------------------
// Vxxxx (desired Pico voltage between 0000 and 9999)
// Hxxxx (output states in order H-L, L-L, H-R, L-R; 1=ON, 0=OFF)
// Pxxxx (HV P gain between 0000 and 9999)
// Ixxxx (HV I gain between 0000 and 9999)
// Dxxxx (HV D gain between 0000 and 9999)

#define USB_RX_BYTES    1
#define USB_COMMAND_MSK 0xE0
#define USB_VALUE_MSK   0x1F
/*
#define F               0
#define P               1
#define I               2
#define D               3
*/
#define HV_GAIN_F       0	
#define HV_GAIN_P       0
#define HV_GAIN_I       0
#define HV_GAIN_D       0

#define HV_SCALE_I      1
#define HV_SCALE_D      1
#define GAIN_DIVIDER	128.0

// 0 = no filtering, 1 = too much filtering
#define HV_FILTER_BETA  0

// Duty cycle capped to prevent excessive voltage
#define DUTY_CAP		717 // 70%

// Routines always only have 
#define GRIPPER_ROUTINE 1
#define DEBUG_PWM		0		
#define DEBUG_OPTO		0
#define DEBUG_LED		0


typedef enum
{
    H_off,          // All low
    LeftH_side,     //LeftH + RightL
    RightH_side,    //RightH + LeftL
    H_discharge     //LeftL + RightL
} h_bridge_state_t;

typedef enum
{
    F,
    P,
    I,
    D,
    SET_HV_TARGET,
    SET_H_STATUS
} control_t;

// FUNCTION PROTOTYPES
void init(void);
void leds(bool led_1, bool led_2);
void set_LED_RED(bool led);
void set_LED_YLW(bool led);
void inline set_h_bridge(h_bridge_state_t state);

void inline set_dutyccycle(double hv_output);

// ROUTINES
void debug_pwm();

volatile static bool tick = FALSE;

int main(void)
{
    
    char incoming = 0; 
    uint16_t outgoing = 0;
    char usb_rx_buffer[USB_RX_BYTES];
    
    uint16_t hv_target = 1023; // RANGE = 0 to 1023 (ADC range)
    //int outputs[4] = {0,0,0,0}; // H-L, L-L, H-R, L-R
    double hv_gains[4] = {HV_GAIN_F,HV_GAIN_P,HV_GAIN_I,HV_GAIN_D}; //F, P, I, D }FPID: RANGE = 0 to 8191 (13bit)>>GAIN_SHIFT | (@GAINSHIFT=7 => (0:0.0078125:(641-0.0078125))

    double hv_feedback = 0.0, hv_filtered = 0.0, hv_p = 0.0, hv_i = 0.0, hv_d = 0.0, hv_p_last = 0.0, hv_output = 0.0;
    h_bridge_state_t h_bridge_state = H_off;
	
    int i = 0;
    init();
	set_h_bridge(h_bridge_state);
	
#if GRIPPER_ROUTINE
/************************************************************************
* This routine goes through following steps
* 1) 5s wait -> M2 green
* 2) 5s charge: duty cycle 85% -> ~6kV -> M2 red
* 3) 5s enable Left_High and Right_Low 
* 4) 5s discharge -> enable Left_low and Right_Low
* 5) 5s enable Right_High and Left_Low
* 6) 5s discharge -> enable Left_low and Right_Low
* 7) finish -> 100% duty cycle all opto diodes disabled M2 red(off)
************************************************************************/
	double HighVoltage = 1023*0.85; // set duty cycle to 85 %
	double LowVoltage = 1023; // set duty cycle to 85 %
	const uint16_t wait_time = 39063; 
	typedef enum{
		state_1_wait,
		state_2_charge,
		state_3_LeftH,
		state_4_disC,
		state_5_RightH,
		state_6_disC,
		state_7_finish
		}t_state_machine;
	bool statechange = TRUE;
	t_state_machine current_state = state_1_wait;
	uint16_t timercounter = 0;
	uint16_t timerovf = wait_time;
	while(1)
    {
        while(!tick){} // fixed timing at 7812.5 Hz (this may be a bit too aggress for the USB comms, let's see)
		tick = FALSE;
		hv_feedback = ADC;
		if(statechange){ // on state change
			statechange = FALSE;
			switch(current_state)
			{
				case state_1_wait:
					timerovf = wait_time;
					m_green(ON);
					break;
				case state_2_charge:
					timerovf = wait_time;
					m_red(ON);
					set_dutyccycle(HighVoltage);
					break;
				case state_3_LeftH:
					timerovf = wait_time;
					set_h_bridge(LeftH_side);
					break;
				case state_4_disC:
					timerovf = wait_time;
					set_h_bridge(H_discharge);
					break;
				case state_5_RightH:
					timerovf = wait_time;
					set_h_bridge(RightH_side);
					break;
				case state_6_disC:
					timerovf = wait_time;
					set_h_bridge(H_discharge);
					break;
				case state_7_finish:
					set_h_bridge(H_off);
					set_dutyccycle(LowVoltage); // turn HV off
					m_red(OFF);
					m_green(OFF);
					break;
			}
		}
		if(timercounter == timerovf && current_state < state_7_finish) // timer
		{ 
			current_state++;
			statechange = TRUE;
		}
		timercounter++;
        
		if(hv_feedback>40){// HV_LED if ADC voltage is > 200mV => 425 V at HV output
			set_LED_RED(TRUE);
			}else{
			set_LED_RED(FALSE);
		}
    }
#endif	// GRIPPER_ROUTINE
#if DEBUG_PWM
	set_h_bridge(H_off);
	m_red(ON);
	hv_gains[F] = 128;
	hv_gains[P] = 0;
	hv_target = 1023; // 2.5V
	m_red(OFF);
	m_green(ON);
	uint16_t cnt = 0;
	while(1){
		
		 while(!tick){} // fixed timing at 7812.5 Hz (this may be a bit too aggress for the USB comms, let's see)
		// increase the target:
		
		tick = FALSE;
		        // read HV feedback (ADC in free-running mode, 5V = 1024)
        hv_feedback = ADC;
		if (cnt++ == 0xFFFF)//~8s
		{
			hv_target = (hv_target > 750) ? hv_target-51 : 1023;
		}

        // FIRST-ORDER LOW-PASS FILTER
        hv_filtered = HV_FILTER_BETA * hv_filtered + (1-HV_FILTER_BETA) * hv_feedback;
        
        // PROPORTIONAL
        hv_p_last = hv_p; // store for the derivative calculation
        hv_p = hv_target - hv_filtered;
        
        // INTEGRAL
        hv_i += hv_p * HV_SCALE_I;

        // DERIVATIVE
        hv_d = (hv_p - hv_p_last) * HV_SCALE_D;
        
        // OUTPUT
        hv_output = (hv_gains[F]/GAIN_DIVIDER) * hv_target + (hv_gains[P]/GAIN_DIVIDER) * hv_p + (hv_gains[I]/GAIN_DIVIDER) * hv_i + (hv_gains[D]/GAIN_DIVIDER) * hv_d;
		set_dutyccycle(hv_output);
		
		if(hv_feedback>40){// HV_LED if ADC voltage is > 200mV => 425 V at HV output
			set_LED_RED(TRUE);
			}else{
			set_LED_RED(FALSE);
		}
		/*
		outgoing = hv_output;
		m_usb_tx_char((char)(outgoing));
		m_usb_tx_char((char)(outgoing>>8));
		
		outgoing = hv_feedback;
		m_usb_tx_char((char)(outgoing));
		m_usb_tx_char((char)(outgoing>>8));
		
		outgoing = hv_target;
		m_usb_tx_char((char)(outgoing));
		m_usb_tx_char((char)(outgoing>>8));
		
		outgoing = (pmw_cycle);
		m_usb_tx_char((char)(outgoing));
		m_usb_tx_char((char)(0));
		*/
		
		if(tick){ // we've overrun the loop, which is bad
			m_red(ON);
		}
	}

#endif
#if DEBUG_OPTO
	OCR4D = 0;
	set_h_bridge(0);
while(1){
	if(m_usb_rx_available()){
		h_bridge_state = m_usb_rx_char();
		//outgoing = hv_gains[h_bridge_state];
		set_h_bridge(h_bridge_state);
	}
	
	
}

#endif
#if DEBUG_LED
	while(1){
		
	}

#endif
	
    while(1)
    {
        while(!tick){} // fixed timing at 7812.5 Hz (this may be a bit too aggress for the USB comms, let's see)
		tick = FALSE;
		
		// read HV feedback (ADC in free-running mode, 5V = 1024)
		hv_feedback = ADC;
		
		// FIRST-ORDER LOW-PASS FILTER
		hv_filtered = HV_FILTER_BETA * hv_filtered + (1-HV_FILTER_BETA) * hv_feedback;
		
		// PROPORTIONAL
		hv_p_last = hv_p; // store for the derivative calculation
		hv_p = hv_target - hv_filtered;
		
		// INTEGRAL
		hv_i += hv_p * HV_SCALE_I;

		// DERIVATIVE
		hv_d = (hv_p - hv_p_last) * HV_SCALE_D;
		
		// OUTPUT
		hv_output = (hv_gains[F]/GAIN_DIVIDER) * hv_target + (hv_gains[P]/GAIN_DIVIDER) * hv_p + (hv_gains[I]/GAIN_DIVIDER) * hv_i + (hv_gains[D]/GAIN_DIVIDER) * hv_d;
		set_dutyccycle(hv_output);

        set_h_bridge(h_bridge_state);

        // GET USB INPUT
        if(m_usb_rx_available()){ // absorb a new incoming command packet
            incoming = m_usb_rx_char();
            switch((control_t)(incoming&USB_COMMAND_MSK)>>5){
                case SET_HV_TARGET: // SET HV OUTPUT
                    /*
                    for(i=0; i<USB_RX_BYTES; i++){
                        while(!m_usb_rx_available()){};
                        usb_rx_buffer[i] = m_usb_rx_char();
                        m_usb_tx_char(usb_rx_buffer[i]);
                    }
                    m_usb_tx_string("\n");
                    hv_target = (int)(usb_rx_buffer[3]-'0') + 10*(int)(usb_rx_buffer[2]-'0') + 100*(int)(usb_rx_buffer[1]-'0') + 1000*(int)(usb_rx_buffer[0]-'0');
                    */
                    while(!m_usb_rx_available()){};
                    usb_rx_buffer[i] = m_usb_rx_char();
                    hv_target = ((uint16_t)(incoming&USB_VALUE_MSK)<<8) | usb_rx_buffer[0];
					hv_target = (hv_target<1024) ? hv_target : 1023; // target <= 1024
                    outgoing = hv_target;
                    break;
                case F: // SET HV F GAIN
                    while(!m_usb_rx_available()){};
                    usb_rx_buffer[i] = m_usb_rx_char();
                    hv_gains[F] = ((uint16_t)(incoming&USB_VALUE_MSK)<<8) | usb_rx_buffer[0];
                    outgoing = hv_gains[F];
                    break;
                case P: // SET HV P GAIN
                    while(!m_usb_rx_available()){};
                    usb_rx_buffer[i] = m_usb_rx_char();
                    hv_gains[P] = ((uint16_t)(incoming&USB_VALUE_MSK)<<8) | usb_rx_buffer[0];
                    outgoing = hv_gains[P];
                    break;
                case I: // SET HV I GAIN
                                        while(!m_usb_rx_available()){};
                    usb_rx_buffer[i] = m_usb_rx_char();
                    hv_gains[I] = ((uint16_t)(incoming&USB_VALUE_MSK)<<8) | usb_rx_buffer[0];
                    outgoing = hv_gains[I];
                    break;
                case D: // SET HV D GAIN
                    while(!m_usb_rx_available()){};
                    usb_rx_buffer[i] = m_usb_rx_char();
                    hv_gains[D] = ((uint16_t)(incoming&USB_VALUE_MSK)<<8) | usb_rx_buffer[0];
                    outgoing = hv_gains[D];
                    break;
                case SET_H_STATUS: // SET H-BRIDGE OUTPUTS
                    h_bridge_state = (h_bridge_state_t)(incoming&USB_VALUE_MSK);
                    outgoing = hv_gains[h_bridge_state];
                    break;
                default:
                    //invalid command
                    break;
            }
            //bounce back set value
			m_usb_tx_char((char)(outgoing));
            m_usb_tx_char((char)(outgoing>>8));
            

        }
		if(hv_feedback>40){// HV_LED if ADC voltage is > 200mV => 425 V at HV output
			set_LED_RED(TRUE);
			}else{
			set_LED_RED(FALSE);
		}
        if(tick){ // we've overrun the loop, which is bad
            m_red(ON);
        }
    }
}

void leds(bool led_1, bool led_2){
    if(led_1){
        set(PORTB,0);
    } else {
        clear(PORTB,0);
    }
    if(led_2){
        set(PORTB,1);
    } else {
        clear(PORTB,1);
    }
}
void set_LED_RED(bool led){
	if(led){
		set(PORTB,0);
		} else {
		clear(PORTB,0);
	}
	
}
void set_LED_YLW(bool led){
    if(led){
	    set(PORTB,1);
	    } else {
	    clear(PORTB,1);
    }
	
}
void init(void){
    
    // GENERAL
    m_clockdivide(0);       // full 16-MHz speed
    m_usb_init();           // enable USB
    m_red(OFF);
    m_green(OFF);
    
    // GPIO ---------------------------------------

    // IR HV drivers
    set(DDRB,5);            // output
    set(PORTB,5);           // default OFF
    set(DDRB,6);            // output
    set(PORTB,6);           // default OFF
    set(DDRB,7);            // output
    set(PORTB,7);           // default OFF
    set(DDRC,6);            // output
    set(PORTC,6);           // default OFF
    
    // LED 1
    set(DDRB,0);            // output
    clear(PORTB,0);         // default OFF

    // LED 2
    set(DDRB,1);            // output
    clear(PORTB,1);         // default OFF

    // TIMERS ------------------------------------
    
    // TICK TIMER (TIM0 rollover at 7812.5 Hz)
    set(TCCR0B,CS01);       // enable TIM0 with clock divide = /8
    set(TIMSK0,TOIE0);      // interrupt on rollover
    
    // BUCK CONTROL (PWM, OC4D on pin D7)
    TCNT4 = 0;              // clear the counter
    OCR4D = 0;              // set duty cycle to zero

    clear(TCCR4D,WGM41);    // up to OCR4C (max 255)
    clear(TCCR4D,WGM40);    // ^
    
    TC4H = 0x00;            // TOP value of 255, frequency of 7.8kHz
    OCR4C = 0xFF;           // ^

    set(DDRD,7);            // output
    set(TCCR4C,COM4D1);     // clear on match with OCR4D; set at TOP
    set(TCCR4C,PWM4D);      // ^

    clear(TCCR4B,CS40);     // set prescaler to /8 (2 MHz)
    set(TCCR4B,CS41);       // ^
    clear(TCCR4B,CS42);     // ^
    clear(TCCR4B,CS43);     // ^

    // ADC ---------------------------------------
    
    // HV FEEDBACK (ADC9, D6)
    set(ADMUX,REFS0);       // voltage ref to Vcc
    clear(ADMUX,REFS1);     // ^
    
    set(ADCSRA,ADPS0);      // prescaler to /128
    set(ADCSRA,ADPS1);      // ^
    set(ADCSRA,ADPS2);      // ^

    set(DIDR2,1);           // disable digital
    
    set(ADCSRB,MUX5);       // set channel multiplexer
    set(ADMUX,MUX0);        // ^

    set(ADCSRA,ADATE);      // free-running mode
    
    set(ADCSRA,ADEN);       // enable conversion

    set(ADCSRA,ADSC);       // start conversion

    // MISC -------------------------------------

    sei();                   // enable interrupts
	set_LED_RED(FALSE);
	set_LED_YLW(TRUE);
}


/**
 * @brief sets the state of the H-bridge using predifined states to prevent accidental short of a sided (e.g. bit error)
 * 
 * @param state states of the H-bridge: LeftH_side, RightH_side, H_discharge
 */
void inline set_h_bridge(h_bridge_state_t state){
    // disconnect bridge to ensure correct activation order 
    clear(PORTC,6); // H-Left (C6)
    clear(PORTB,6); // H-Right (B6)
    clear(PORTB,7); // L-Left (B7)
    clear(PORTB,5); // L-Right (B5)

    switch (state)
    {
    case(H_off):
        break;
    case LeftH_side:
    set(PORTB,5); // L-Right (B5)
    set(PORTC,6); // H-Left (C6)
        break;
    case RightH_side:
    set(PORTB,7); // L-Left (B7)
    set(PORTB,6); // H-Right (B6)
        break;
    case H_discharge:    
    set(PORTB,7); // L-Left (B7)
    set(PORTB,5); // L-Right (B5)
    default: // this state should never be reached
        break;
    }
}
/**
 * @brief sets dutycycle of the PWM signal
 * 
 * @param hv_output the result of the PID 
 */
void inline set_dutyccycle(double hv_output){
uint8_t pwm_cycle = ((hv_output/4) > (DUTY_CAP/4)) ? (hv_output/4) : DUTY_CAP/4; //convert double to uint8 an cap duty cycle
OCR4D = pwm_cycle; // PWM output max 255
}
ISR(TIMER0_OVF_vect){ // rolls over every 1.024 ms (Julian: I think it's actually 0.1280 ms at 7812.5Hz?)
    tick = TRUE;
}



