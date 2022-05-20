#include "m_general.h"
#include "m_usb.h"
#include "pwm_lookup.h"

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
#define USB_TX_BYTES    64
#define USB_CONTROL_MSK 0xE0
#define USB_VALUE_MSK   0x1F
/*
#define F               0
#define P               1
#define I               2
#define D               3
*/

// Gain will be shifted by GAIN_SHIFT 
#define HV_GAIN_F       4 // @GAIN_SHIFT = 2| 4 => gain of 1  	
#define HV_GAIN_P       0
#define HV_GAIN_I       0
#define HV_GAIN_D       0

#define HV_SCALE_I      1
#define HV_SCALE_D      1
//#define GAIN_DIVIDER	128.0
#define GAIN_SHIFT		2 // Dividing by 4

// 0 = no filtering, 1 = too much filtering
#define HV_FILTER_BETA  0

// Duty cycle capped to prevent excessive voltage
#define DUTY_CYCLE_CAP		512 // 50%

// Routines always only have 
#define GRIPPER_ROUTINE 0
#define DEBUG_PWM		0		


typedef enum
{
    H_OFF,          // All low
    H_LEFT,			//LeftH + RightL
    H_RIGHT,		//RightH + LeftL
    H_DIS			// Discharge Hasel : LeftL + RightL
} h_bridge_state_t;
h_bridge_state_t h_current_bridge_state = H_OFF;
typedef enum
{
    F,	// 0
    P,	// 1
    I,	// 2
    D,	// 3
    SET_HV_TARGET,	// 4
    SET_H_STATUS,	// 5
	USB_STREAM,		// 6
	PARAM_DUMP = 7	//7
} control_t;

typedef enum
{
	TODO
	
} usb_command_t;

// FUNCTION PROTOTYPES
void init(void);
void leds(bool led_1, bool led_2);
void set_LED_RED(bool led);
void set_LED_YLW(bool led);
void inline set_h_bridge(h_bridge_state_t state);

void inline set_dutycycle(uint16_t hv_output);


volatile static bool tick = FALSE;
bool is_usb_streaming = FALSE; 

int main(void)
{
   
    uint16_t incoming = 0; 
	uint8_t control = 0;
    uint16_t outgoing = 0;
    uint8_t usb_rx_buffer;
	uint16_t usb_tx_buffer[USB_TX_BYTES/2];
    
    uint16_t hv_target = 0; // RANGE = 0 to 1023 (ADC range)
    //int outputs[4] = {0,0,0,0}; // H-L, L-L, H-R, L-R
    uint8_t hv_gains[4] = {HV_GAIN_F,HV_GAIN_P,HV_GAIN_I,HV_GAIN_D}; //F, P, I, D }FPID: RANGE = 0 to 31 (5bit)>>GAIN_SHIFT | (@GAINSHIFT=2 => (0:0.25:7.75)

    int16_t hv_feedback = 0, hv_filtered = 0, hv_p = 0, hv_i = 0, hv_d = 0, hv_p_last = 0, hv_output = 0;
    h_bridge_state_t h_next_bridge_state = H_OFF;
	uint8_t steam_cnt = 0;
	
    init();
	set_dutycycle(1023);
	set_h_bridge(h_next_bridge_state);
	
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
	double HighVoltage = 1023*0.837; // set duty cycle to 83.7% for 6.5 kV
	double LowVoltage = 1023; // set duty cycle to 100 %
	const uint16_t wait_time = 39063;
	typedef enum{
		state_1_wait,
		state_2_charge,
		state_3_LeftH,
		state_4_disC,
		state_5_RightH,
		state_6_disC,
		state_7_LeftH,
		state_8_disC,
		state_9_finish
		}t_state_machine;
	bool statechange = TRUE;
	t_state_machine current_state = state_1_wait;
	uint32_t timercounter = 0;
	uint32_t timerovf = wait_time;
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
					timerovf = 7812.5*3.61;
					m_green(ON);
					break;
				case state_2_charge:
					timerovf = 7812.5*3.61;
					m_red(ON);
					set_dutycycle(HighVoltage);
					break;
				case state_3_LeftH:
					timerovf = 7812.5*3.61;
					set_LED_RED(TRUE);
					set_h_bridge(H_LEFT);
					break;
				case state_4_disC:
					timerovf = 7812.5*16.16;
					set_LED_RED(FALSE);
					set_h_bridge(H_DIS);
					break;
				case state_5_RightH:
					timerovf = 7812.5*11.17;
					set_LED_RED(TRUE);
					set_h_bridge(H_RIGHT);
					break;
				case state_6_disC:
					timerovf = 7812.5*14.36;
					set_LED_RED(FALSE);
					set_h_bridge(H_DIS);
					break;
				case state_7_LeftH:
					timerovf = 7812.5*9.19;
					set_LED_RED(TRUE);
					set_h_bridge(H_LEFT);
				case state_8_disC:
					timerovf = 7812.5*14.25;
					set_LED_RED(FALSE);
					set_h_bridge(H_DIS);
				case state_9_finish:
					set_h_bridge(H_OFF);
					set_dutycycle(LowVoltage); // turn HV off
					m_red(OFF);
					m_green(OFF);
					break;
			}
		}
		if(timercounter >= timerovf && current_state < state_9_finish) // timer
		{ 
			current_state++;
			timercounter = 0;
			statechange = TRUE;
		}
		timercounter++;
        
		//if(hv_feedback>40){// HV_LED if ADC voltage is > 200mV => 425 V at HV output
		//	set_LED_RED(TRUE);
		//	}else{
		//	set_LED_RED(FALSE);
		//}
    }
#endif	// GRIPPER_ROUTINE
#if DEBUG_PWM
	

#endif //DEBUG_PWM
	m_green(ON);
	m_red(OFF);
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
		//hv_output =		(hv_gains[F]/GAIN_DIVIDER) * hv_target + (hv_gains[P]/GAIN_DIVIDER) * hv_p + (hv_gains[I]/GAIN_DIVIDER) * hv_i + (hv_gains[D]/GAIN_DIVIDER) * hv_d;
		hv_output =		hv_target;// //(hv_target	* hv_gains[F])	>>GAIN_SHIFT; 
		hv_output +=	(hv_p		* hv_gains[P]);
		hv_output +=	(hv_i		* hv_gains[I])	>>GAIN_SHIFT;
		hv_output +=	(hv_d		* hv_gains[D])	>>GAIN_SHIFT;
		
		hv_output = ((hv_output&0x8000)) ? 0 : hv_output; // 2-complement is this number negative?
		hv_output = (hv_output>1023) ? 1023 : hv_output;
		//hv_output = (hv_output< 0)	? 0 : hv_output;
		
		hv_output =		pwm_lookup[hv_output];
		set_dutycycle(hv_output);
        set_h_bridge(h_next_bridge_state);

        // GET USB INPUT
        if(m_usb_rx_available()){ // absorb a new incoming command packet
            incoming = m_usb_rx_char();
			control = incoming&USB_CONTROL_MSK;
            switch((control_t)(control)>>5){
                case SET_HV_TARGET: // SET HV OUTPUT
                    while(!m_usb_rx_available());
					usb_rx_buffer = m_usb_rx_char();
                    hv_target = ((uint16_t)(incoming&USB_VALUE_MSK)<<8) | usb_rx_buffer;
					hv_target = (hv_target<1024) ? hv_target : 1023; // target <= 1024
                    outgoing = hv_target;
                    break;
                case F: // SET HV F GAIN
					hv_gains[F] = (uint8_t)(incoming&USB_VALUE_MSK);
                    outgoing = hv_gains[F];
                    break;
                case P: // SET HV P GAIN
					hv_gains[P] = (uint8_t)(incoming&USB_VALUE_MSK);
                    outgoing = hv_gains[P];
                    break;
                case I: // SET HV I GAIN
                    hv_gains[I] = (uint8_t)(incoming&USB_VALUE_MSK);
					outgoing = hv_gains[I];
                    break;
                case D: // SET HV D GAIN
                    hv_gains[D] = (uint8_t)(incoming&USB_VALUE_MSK);
					outgoing = hv_gains[D];
                    break;
                case SET_H_STATUS: // SET H-BRIDGE OUTPUTS
                    h_next_bridge_state = (h_bridge_state_t)(incoming&USB_VALUE_MSK);
                    outgoing = h_next_bridge_state;
                    break;
				case USB_STREAM: // SET H-BRIDGE OUTPUTS
					is_usb_streaming = (uint8_t)(incoming&USB_VALUE_MSK);
					outgoing = is_usb_streaming;
					m_usb_tx_char((char)(outgoing));
					m_usb_tx_char((char)(outgoing>>8));
					steam_cnt = 0;
					break;
				case PARAM_DUMP: // Dump parameters
					m_usb_tx_char( hv_gains[P]);
					m_usb_tx_char(hv_gains[I]);
					m_usb_tx_char(hv_gains[D]);
					outgoing = hv_target;
				    break;	
                default:
                    //invalid command
                    break;
            }
            //bounce back set value
			if(!is_usb_streaming){
				m_usb_tx_char((char)(outgoing));
				m_usb_tx_char((char)(outgoing>>8));
			}
        }
		if(check(PORTE,6)){// WARNING THIS ONLY WORKS WHEN THE BOARD HAS BEEN MODIFIED ACCORDINGLY
			set_LED_RED(TRUE);
			}else{
			set_LED_RED(FALSE);
		}
        if(tick){ // we've overrun the loop, which is bad
            m_red(ON);
        }
		if (is_usb_streaming)
		{
			usb_tx_buffer[steam_cnt++] = hv_p;
			if (steam_cnt==USB_TX_BYTES/2)
			{
				usb_serial_write((uint8_t*) usb_tx_buffer, USB_TX_BYTES);
				steam_cnt = 0;
			}
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
    clear(PORTB,5);           // default OFF
    set(DDRB,6);            // output
    clear(PORTB,6);           // default OFF
    set(DDRB,7);            // output
    clear(PORTB,7);           // default OFF
    set(DDRC,6);            // output
    clear(PORTC,6);           // default OFF
    
    // LED 1
    set(DDRB,0);            // output
    clear(PORTB,0);         // default OFF

    // LED 2
    set(DDRB,1);            // output
    clear(PORTB,1);         // default OFF
	
	// HV_Enable 
	clear(DDRE,6);            // sense only
	set(PORTE,6);
    // TIMERS ------------------------------------
    
    // TICK TIMER (TIM0 rollover at 7812.5 Hz)
    set(TCCR0B,CS01);       // enable TIM0 with clock divide = /8
    set(TIMSK0,TOIE0);      // interrupt on rollover
    
    // BUCK CONTROL (PWM, OC4D on pin D7)
    TCNT4 = 0;              // clear the counter
    OCR4D = 0xFF;           // set duty cycle to 100%

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
	if(h_current_bridge_state != state){
		h_current_bridge_state = state;
		// disconnect bridge to ensure correct activation order 
		clear(PORTC,6); // H-Left (C6)
		clear(PORTB,6); // H-Right (B6)
		clear(PORTB,7); // L-Left (B7)
		clear(PORTB,5); // L-Right (B5)

		switch (state)
		{
		case(H_OFF):
			break;
		case H_LEFT:
		set(PORTB,5); // L-Right (B5)
		set(PORTC,6); // H-Left (C6)
			break;
		case H_RIGHT:
		set(PORTB,7); // L-Left (B7)
		set(PORTB,6); // H-Right (B6)
			break;
		case H_DIS:    
		set(PORTB,7); // L-Left (B7)
		set(PORTB,5); // L-Right (B5)
		default: // this state should never be reached
			break;
		}
	}
}
/**
 * @brief sets dutycycle of the PWM signal 1023 = 100%
 * 
 * @param hv_output the result of the PID 
 */
void inline set_dutycycle(uint16_t hv_output){
	hv_output = (hv_output>1023) ? 1023 : hv_output; //check if hv_output is bigger than 1023 and if so set to 1023
	uint8_t pwm_cycle = ((hv_output>>2) > (DUTY_CYCLE_CAP>>2)) ? (hv_output>>2) : DUTY_CYCLE_CAP>>2; //convert double to uint8 an cap duty cycle
	OCR4D = pwm_cycle; // PWM output max 255
}
ISR(TIMER0_OVF_vect){ // rolls over every 1.024 ms (Julian: I think it's actually 0.1280 ms at 7812.5Hz?)
    tick = TRUE;
}



