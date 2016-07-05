
#define F_CPU 3686400L // 3.6864MHz quartz crystal makes a great source for UART frequencies

#include <avr/io.h>
#include <util/delay.h>
#include <avr/fuse.h>
#include <avr/interrupt.h>

/*
#    avr runs on 3.6864MHz quartz crystal (XTAL pins)
#
#    basic function is as follows:
#
#    while true:
#        watch out for commands from UART
#        the user wants to set one of the many attached SPI port expander bits? (decode...)
#        craft SPI message and send it out
#
*/



// fuses settings in source code get compiled into data at magic numbered addresses in the elf file
// these addresses are outside avr memory space, e.g. prog fails if avrdude does not know that

// defaults:
// avrdude: safemode: lfuse reads as 64
// avrdude: safemode: hfuse reads as DF
// avrdude: safemode: efuse reads as FF

// see http://www.engbedded.com/fusecalc/ for 0xD8, 0xDF, 0xFF

FUSES =
{
// external clock, long startup time, do not divide clock by 8
.low = (FUSE_CKSEL0 & FUSE_CKSEL1 & FUSE_CKSEL2 & /*FUSE_CKSEL3 & FUSE_SUT0 & */ FUSE_SUT1 /* & FUSE_CKDIV8 */), // 0xD8
.high = HFUSE_DEFAULT,
.extended = EFUSE_DEFAULT,
};


// uart RX buf

char _ReceiveBuffer[100];
unsigned char _ReceiveBufferPos=0;

// pseudo random number generator (do we still need this???)

uint8_t prbs(void)
{
    static uint32_t sequence = 1; // initial value
    sequence = sequence << 1 |  (((sequence >> 30) ^ (sequence >> 27)) & 1); // prbs calculation
    return sequence & 0x01; // return a bit
}

void config_uart(void)
{
    
    #define USART_BAUDRATE 9600 
    #define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
    
    // Enable USART receiver and transmitter
	UCSRB |= (1 << RXEN) | (1 << TXEN);
	
	// Set baud rate bits
	UBRRH = (BAUD_PRESCALE >> 8);
	UBRRL = BAUD_PRESCALE;
}

void uart_transmit( unsigned char data )
{
    /* Wait for empty transmit buffer */
    while ( !( UCSRA & (1<<UDRE)) );
    /* Put data into buffer, sends the data */
    UDR = data;
}

void uart_string(char* p)
{
    while(*p!=0)
    {
        uart_transmit(*p++);
    }
}

////////////////////////////////////////////////////////////////////////

void config_spi()
{
    DDRB &= ~(1<<PB5); // Data input is an input
    PORTB |= 1 << PB5; // enable pullup for DI

    // Define the following pins as output // data outputs are outputs (d-oh!)
    //DDR_SPI |= ((1<<USI_DO)|(1<<USI_SCK));
}

uint8_t spi_readWrite(uint8_t data)
{
  USIDR = data; // put data in output register
  USISR |= 1 << USIOIF; // enable shift-out

  while ((USISR & (1 << USIOIF)) == 0 ) { // while not finished
    USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC); // shift out
  }

  return USIDR; 
}

void spi_cs_low(uint8_t id)
{
    PORTB&= !( 1 << PB0 );
}
void spi_cs_high(uint8_t id)
{
    PORTB|= ( 1 << PB0 );
    
}

uint8_t spi_mcp27s17(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    spi_cs_low(0);
    spi_readWrite(0b01000000|(dev_addr<<1));
    spi_readWrite(reg_addr);    
    spi_readWrite(data);    
    spi_cs_high(0);
    return 0;
} 

////////////////////////////////////////////////////////////////////////

void cmd_interpreter()
{
    if(_ReceiveBuffer[_ReceiveBufferPos-1]!='\r') return;
    _ReceiveBuffer[_ReceiveBufferPos]=0;
    _ReceiveBufferPos=0;
    uart_string("\n\r");
    
    
    unsigned char state=0;
    // different modes
    switch(_ReceiveBuffer[0])
    {
        
        case '?':
            uart_string("wat?");
            state=1;
            break;
        case 'L':
            PORTD^= ( 1 << PD2 ); // toggle LED just because we can
            state=1;
            break;
        default:
            state=0;
    }
    
    if(state==1)
    {
        uart_string("OK");
    }
    else
    {
        uart_string("ERR"); 
    }
    
    uart_string("\n\r>");
    return;
}

void config_io_pins(void)
{
    // set these pins as output
    DDRD |= (1 << PD0); // RX pin, propably makes no sense to set this here?
    DDRD |= (1 << PD1); // TX pin, propably makes no sense to set this here?
    DDRD |= (1 << PD2); // ?
    DDRD |= (1 << PD3); // ?
    DDRD |= (1 << PD4); // ?
    DDRD |= (1 << PD5); // ?
    
    // SPI
    DDRB |= (1 << PB7); // ?
    DDRB |= (1 << PB6); // ?
    DDRB |= (1 << PB0); // ?
    
}

// finite state machine, doing nothing right now! might later be used to trigger events on timing basis (if we really need "realtime" stuff).
void fsm(void)
{
    static uint8_t state=0;

    switch(state)
    {
        case 0: // sate 0
            //output_positive();
            state=1; // go to state 1 next
            break;
        case 2: // state 2
        case 62: // state 62
            //output_zero();
            state=state+1; // go to next state
            break;
        case 60: // state 60
            //output_negative();
            state=61; // go to state 61 next
            break;
        case 119: // state 119
            state=0; // return to first state instead of state 120
            break;
        default: // default state. if no other sate applies, go to next state
            state=state+1;
    }

}

void config_interrupts(void)
{
    
    // NOTE: clock config calculation is all wrong due to changed XTAL frequency (32kHz to 3,something MHz).
    // ----------------------------------------------
    
    // clock input to AVR is 32768 Hz
    // timer 0 should fire at 1Hz
    // this means divide input clock by 2^15
    // see Figure 27. 8-bit Timer/Counter Block Diagram in attiny2313 datasheet for reference

    // timer/counter control register B - TTC0B
    // last 3 bit define clock select for timer 0
    // 001 = 2^1  = 1
    // 010 = 2^3  = 8
    // 011 = 2^6  = 64
    // 100 = 2^8  = 256
    // 101 = 2^10 = 1024
    TCCR0B = 0b00000100; // prescale = 256 / 2^8
    TCCR0A = (1<<WGM01); // CTC mode, so we can go for 2^7 here

    // the timer starts at value 0:
    TCNT0 = 0x0;

    TIMSK |= (1<<OCIE0A); // enable compare interrupt for timer 0

    // The counter value (TCNT0) increases until a Compare Match occurs
    // between TCNT0 and OCR0A, and then counter (TCNT0) is cleared.
    OCR0A = 0x7f; // half way up
    
    
    // Enable USART Receive Complete interrupt
	UCSRB |= (1 << RXCIE);
	UCSRB |= (1 << TXCIE);
	UCSRB |= (1 << UDRIE);
    


    sei(); // enable global interrupts
}

// called at 1Hz frequency, now propably a bit faster
ISR(TIMER0_COMPA_vect)
{
    //fsm();
//    PORTD^= ( 1 << PD2 ); // toggle bit to show step

}

ISR(USART_RX_vect)
{ 
    // echo!
    char receivedByte;
    receivedByte = UDR;
    UDR=receivedByte; 
    if(_ReceiveBufferPos<100)
    {
        _ReceiveBuffer[_ReceiveBufferPos++]=receivedByte; // write to buffer until it is full
    }
}

ISR(USART_TX_vect)
{    
}

ISR(USART_UDRE_vect)
{
}


int main(void)
{
    
    _delay_ms(50); // to give programmer time to interrupt and re-flash
    
    config_io_pins();
    config_interrupts();
    config_uart();
    
    uart_string("AVRSPICTRL\n\r");
    
    uint16_t q=1;

    while(1)
    {
        cmd_interpreter();
        
        spi_mcp27s17(0,0x00,0); // register IODIR A set to OUTPUT (0)
        spi_mcp27s17(0,0x01,0); // register IODIR B set to OUTPUT (0)
        spi_mcp27s17(0,0x12,q&0xff);
        spi_mcp27s17(0,0x13,(q>>8)&0xff);
        _delay_ms(1);
        q=q<<1;
        if(!q)q=1;
    } // interesting stuff happens in ISRs.

    return 0;
}
