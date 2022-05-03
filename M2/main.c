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

// 0 = no filtering, 1 = too much filtering
#define HV_FILTER_BETA  0

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
void inline set_h_bridge(h_bridge_state_t state);

volatile static bool tick = FALSE;

int main(void)
{
    
    char incoming = 0; 
    uint16_t outgoing = 0;
    char usb_rx_buffer[USB_RX_BYTES];
    
    uint16_t hv_target = 0; // RANGE = 0 to 8191 (13bit)
    //int outputs[4] = {0,0,0,0}; // H-L, L-L, H-R, L-R
    uint16_t hv_gains[4] = {HV_GAIN_F,HV_GAIN_P,HV_GAIN_I,HV_GAIN_D}; //F, P, I, D // RANGE = 0 to 8191 (13bit)

    double hv_feedback = 0.0, hv_filtered = 0.0, hv_p = 0.0, hv_i = 0.0, hv_d = 0.0, hv_p_last = 0.0, hv_output = 0.0;
    h_bridge_state_t h_bridge_state = H_off;

    int i = 0;
    init();
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
        hv_output = hv_gains[F] * hv_target + hv_gains[P] * hv_p + hv_gains[I] * hv_i + hv_gains[D] * hv_d;
        OCR4D = hv_output; // PWM output max 1023


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
            m_usb_tx_char((char)(outgoing>>8));
            m_usb_tx_char((char)(outgoing));

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

    clear(TCCR4D,WGM41);    // up to OCR4C (max 1023)
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
    
}

/**
 * @brief sets the state of the H-bridge using predifined states to prevent accidental short of a sided (e.g. bit error)
 * 
 * @param state states of the H-bridge: LeftH_side, RightH_side, H_discharge
 */
void inline set_h_bridge(h_bridge_state_t state){
    // disconnect bridge to ensure correct activation order 
    clear(DDRC,6); // H-Left (C6)
    clear(DDRB,6); // H-Right (B6)
    clear(DDRB,7); // L-Left (B7)
    clear(DDRB,5); // L-Right (B5)

    switch (state)
    {
    case(H_off):
        break;
    case LeftH_side:
    set(DDRB,5); // L-Right (B5)
    set(DDRC,6); // H-Left (C6)
        break;
    case RightH_side:
    set(DDRB,7); // L-Left (B7)
    set(DDRB,6); // H-Right (B6)
        break;
    case H_discharge:    
    set(DDRB,7); // L-Left (B7)
    set(DDRB,5); // L-Right (B5)
    default: // this state should never be reached
        break;
    }
}

ISR(TIMER0_OVF_vect){ // rolls over every 1.024 ms
    tick = TRUE;
}
