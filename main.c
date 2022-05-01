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

#define USB_RX_BYTES    4

#define F               0
#define P               1
#define I               2
#define D               3

#define HV_GAIN_F       0
#define HV_GAIN_P       0
#define HV_GAIN_I       0
#define HV_GAIN_D       0

#define HV_SCALE_I      1
#define HV_SCALE_D      1

// 0 = no filtering, 1 = too much filtering
#define HV_FILTER_BETA  0

// FUNCTION PROTOTYPES
void init(void);
void leds(bool led_1, bool led_2);

volatile static bool tick = FALSE;

int main(void)
{
    char incoming = 0;
    char usb_rx_buffer[USB_RX_BYTES];
    
    int hv_target = 0; // RANGE = 0 to 9999
    int outputs[4] = {0,0,0,0}; // H-L, L-L, H-R, L-R
    int hv_gains[4] = {0,0,0,0}; // P, I, D

    double hv_feedback = 0.0, hv_filtered = 0.0, hv_p = 0.0, hv_i = 0.0, hv_d = 0.0, hv_p_last = 0.0, hv_output = 0.0;
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

        // SET H-BRIDGE OUTPUTS
        if(outputs[0]){ // H-L (C6)
            set(DDRC,6);
        } else {
            clear(DDRC,6);
        }
        if(outputs[0]){ // L-L (B7)
            set(DDRB,7);
        } else {
            clear(DDRB,7);
        }
        if(outputs[0]){ // H-R (B6)
            set(DDRB,6);
        } else {
            clear(DDRB,6);
        }
        if(outputs[0]){ // L-R (B5)
            set(DDRB,5);
        } else {
            clear(DDRB,5);
        }

        // GET USB INPUT
        if(m_usb_rx_available()){ // absorb a new incoming command packet
            incoming = m_usb_rx_char();
            switch(incoming){
                case 'V': // SET HV OUTPUT
                    for(i=0; i<USB_RX_BYTES; i++){
                        while(!m_usb_rx_available()){};
                        usb_rx_buffer[i] = m_usb_rx_char();
                        m_usb_tx_char(usb_rx_buffer[i]);
                    }
                    m_usb_tx_string("\n");
                    hv_target = (int)(usb_rx_buffer[3]-'0') + 10*(int)(usb_rx_buffer[2]-'0') + 100*(int)(usb_rx_buffer[1]-'0') + 1000*(int)(usb_rx_buffer[0]-'0');
                    break;
                case 'F': // SET HV F GAIN
                    for(i=0; i<USB_RX_BYTES; i++){
                        while(!m_usb_rx_available()){};
                        usb_rx_buffer[i] = m_usb_rx_char();
                        m_usb_tx_char(usb_rx_buffer[i]);
                    }
                    m_usb_tx_string("\n");
                    hv_gains[F] = (int)(usb_rx_buffer[3]-'0') + 10*(int)(usb_rx_buffer[2]-'0') + 100*(int)(usb_rx_buffer[1]-'0') + 1000*(int)(usb_rx_buffer[0]-'0');
                    break;
                case 'P': // SET HV P GAIN
                    for(i=0; i<USB_RX_BYTES; i++){
                        while(!m_usb_rx_available()){};
                        usb_rx_buffer[i] = m_usb_rx_char();
                        m_usb_tx_char(usb_rx_buffer[i]);
                    }
                    m_usb_tx_string("\n");
                    hv_gains[P] = (int)(usb_rx_buffer[3]-'0') + 10*(int)(usb_rx_buffer[2]-'0') + 100*(int)(usb_rx_buffer[1]-'0') + 1000*(int)(usb_rx_buffer[0]-'0');
                    break;
                case 'I': // SET HV I GAIN
                    for(i=0; i<USB_RX_BYTES; i++){
                        while(!m_usb_rx_available()){};
                        usb_rx_buffer[i] = m_usb_rx_char();
                        m_usb_tx_char(usb_rx_buffer[i]);
                    }
                    m_usb_tx_string("\n");
                    hv_gains[I] = (int)(usb_rx_buffer[3]-'0') + 10*(int)(usb_rx_buffer[2]-'0') + 100*(int)(usb_rx_buffer[1]-'0') + 1000*(int)(usb_rx_buffer[0]-'0');
                    break;
                case 'D': // SET HV D GAIN
                    for(i=0; i<USB_RX_BYTES; i++){
                        while(!m_usb_rx_available()){};
                        usb_rx_buffer[i] = m_usb_rx_char();
                        m_usb_tx_char(usb_rx_buffer[i]);
                    }
                    m_usb_tx_string("\n");
                    hv_gains[D] = (int)(usb_rx_buffer[3]-'0') + 10*(int)(usb_rx_buffer[2]-'0') + 100*(int)(usb_rx_buffer[1]-'0') + 1000*(int)(usb_rx_buffer[0]-'0');
                    break;
                case 'H': // SET H-BRIDGE OUTPUTS
                    for(i=0; i<USB_RX_BYTES; i++){
                        while(!m_usb_rx_available()){};
                        usb_rx_buffer[i] = m_usb_rx_char();
                        m_usb_tx_char(usb_rx_buffer[i]);
                    }
                    m_usb_tx_string("\n");
                    outputs[0] = (int)(usb_rx_buffer[0]-'0');
                    outputs[1] = (int)(usb_rx_buffer[1]-'0');
                    outputs[2] = (int)(usb_rx_buffer[2]-'0');
                    outputs[3] = (int)(usb_rx_buffer[3]-'0');
                    break;
            }
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

ISR(TIMER0_OVF_vect){ // rolls over every 1.024 ms
    tick = TRUE;
}
