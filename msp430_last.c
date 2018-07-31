#include <msp430.h>
#include <math.h>

#define UART_TXD 0x01           // TXD on P2.0 (Timer1_A.OUT0)
#define UART_RXD 0x02           // RXD on P2.1 (Timer1_A.CCI1A)
#define UART_TBIT_DIV_2     (16000000 / (115200 * 2))
#define UART_TBIT           (16000000 / 115200)

unsigned char data = 0;
static int i = 0;
static int dataReceived = 0;
int veri[3] = {0,0,0};
unsigned int txData;            // UART internal TX variable
unsigned char rxBuffer;         // Received UART character
static int value = 0;
int str[4] = {0,0,0,0};

void voltage(void);
void power(void);             // Read power
void set_600(void);		// Set BR 600 MSP430
void set_115200(void);		// Set BR 115200 MSP430
void high_pass(void);           // High Pass Filter Active
void reset_cirrus(void);        // Reset Cirrus
void cirrus_115200(void);       // Set BR 115200 Cirrus
void TimerA_UART_init(void);    // Software UART init
void TimerA_UART_tx(unsigned char byte);
void TimerA_UART_print(char *string);

void main(void) {

  WDTCTL = WDTPW + WDTHOLD;     // Stop Watchdog Timer

  P1DIR = 0x41;
  P1SEL = BIT1 + BIT2;          // P1.1 = RXD, P1.2=TXD
  P1SEL2 = BIT1 + BIT2;         // P1.1 = RXD, P1.2=TXD
  P1OUT = 0x00;                 // For Relay on and off
  P2OUT = 0x00;                 // Initialize all GPIO
  P2SEL = UART_TXD + UART_RXD;  // Use TXD/RXD pins
  P2DIR = 0xFF & ~UART_RXD;     // Set pins to output

  set_600();
  reset_cirrus(); 			        // Reset cirrus when swithed on
  cirrus_115200();
  set_115200();
  high_pass();

  UCA0CTL1 &= ~UCSWRST;          // Initialize USCI state machine
  IE2 |= UCA0RXIE;               // Enable USCI_A0 RX interrupt

  __enable_interrupt();          // Enable All Interrupts

  TimerA_UART_init();            // Start Timer_A UART


  while(1)
  {                     // Cirrus, BT and Relay functions are handle here
    if(rxBuffer == 'C')          // Relay off
    {
      P1OUT = 0x00;
      rxBuffer = 0;
    }
    else if(rxBuffer == 'B')     // Relay on
    {

      P1OUT = 0x01;
      rxBuffer = 0;
    }
    else if(rxBuffer == 'A')    // Read power from cirrus
    {
      voltage();
      rxBuffer = 0;
    }
    else if(rxBuffer == 'D')    // Read Page 90 Register 0 from cirrus
    {
      power();
      rxBuffer = 0;
    }
    if(dataReceived == 1)       // Has data taken from cirrus?
    {
        value = veri[0] + veri[1] + veri[2];

        if(value < 1000)
        {
          str[2] = value % 10;
          str[1] = ((value % 100) - str[2]) / 10;
          str[0] = value / 100;

          for( i = 0; i < 3 ; i++)
          {
            TimerA_UART_tx(str[i] + 48);
            str[i] = 0;
          }
        }
        else
        {
           str[0] = value / 1000;
           str[3]= value % 10;
           str[2]= ((value % 100) - str[3]) / 10;
           str[1]= ((value % 1000) - (value % 100)) / 100;

           for( i = 0; i < 4 ; i++)
           {
            TimerA_UART_tx(str[i] + 48);
            str[i] = 0;
           }
        }

        dataReceived = 0;
        value = 0;
    }
  }
}


void TimerA_UART_print(char *string) {
  while (*string != '\0')
    TimerA_UART_tx(*string++);
}

void TimerA_UART_init(void) {
  TA1CCTL0 = OUT;   // Set TXD Idle as Mark = '1'
  TA1CCTL1 = SCS + CM1 + CAP + CCIE;     // Sync, Neg Edge, Capture, Int
  TA1CTL = TASSEL_2 + MC_2;  // SMCLK, continuous mode
}

void TimerA_UART_tx(unsigned char byte) {     // send data to BT

  while (TA1CCTL0 & CCIE);                    // Ensure last char TX'd
  TA1CCR0 = TAR;                              // Current state of TA counter
  TA1CCR0 += UART_TBIT;                       // One bit time till first bit
  TA1CCTL0 = OUTMOD0 + CCIE;                  // Set TXD on EQU0, Int
  txData = byte;                              // Load global variable
  txData |= 0x100;                            // Add mark stop bit to TXData
  txData <<= 1;                               // Add space start bit
}

void set_600(void)
{
    BCSCTL1 = CALBC1_1MHZ;
	  DCOCTL = CALDCO_1MHZ;
	  UCA0CTL1 = UCSSEL_2;
	  UCA0BR0 = 0X82;
	  UCA0BR1 = 0X06;
	  UCA0MCTL = UCBRF_5+UCBRF_6;
    UCA0CTL0 = 0;
	  __delay_cycles(500000);             // Wait for 500 millisecond

}

void reset_cirrus(void)
{
    UCA0TXBUF = 0xC1;
  	__delay_cycles(500000);
}

void cirrus_115200()
{
    UCA0TXBUF = 0x80;		      // Hex numbers for Cirrus 115200BR
	  __delay_cycles(50000);
	  UCA0TXBUF = 0x47;
	  __delay_cycles(50000);
	  UCA0TXBUF = 0x9A;
	  __delay_cycles(50000);
	  UCA0TXBUF = 0x39;
	  __delay_cycles(50000);
	  UCA0TXBUF = 0x02;
	  __delay_cycles(50000);

}

void set_115200(void)
{
	  BCSCTL1 = CALBC1_16MHZ;
	  DCOCTL = CALDCO_16MHZ;
	  UCA0CTL1 |= UCSSEL_2;
    UCA0BR0 = 8;
	  UCA0BR1 = 0;
	  UCA0MCTL = 11;
    UCA0CTL0 = 0;
	  __delay_cycles(800000);
}

void high_pass(void)
{
	  UCA0TXBUF = 0x90;	        	// Hex numbers for HP filter
	  __delay_cycles(800000);
	  UCA0TXBUF = 0x40;
	  __delay_cycles(800000);
	  UCA0TXBUF = 0x0A;
	  __delay_cycles(800000);
	  UCA0TXBUF = 0x02;
	  __delay_cycles(800000);
	  UCA0TXBUF = 0x10;
	  __delay_cycles(800000);
}

#pragma vector=USCIAB0RX_VECTOR                 // data from cirrus taken here
__interrupt void USCI0RX_ISR(void)
{
        data = UCA0RXBUF;
        veri[i] = data;
        i++;

        if(i == 3)
        {
           dataReceived = 1;
           i = 0;
        }
        UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine

}

void power()
{
        UCA0TXBUF = 0x90;
	      __delay_cycles(800000);
        UCA0TXBUF = 0x05;
        __delay_cycles(800000);

}
void voltage()
{
        UCA0TXBUF = 0x90;
     	  __delay_cycles(800000);
	      UCA0TXBUF = 0x00;
        __delay_cycles(800000);

}

#pragma vector = TIMER1_A0_VECTOR                 // Data are sent BT, using P2.0 pin
__interrupt void Timer_A0_ISR(void) {

  static unsigned char txBitCnt = 10;

  TA1CCR0 += UART_TBIT;     // Add Offset to CCRx

  if (txBitCnt == 0) {     // All bits TXed?
    TA1CCTL0 &= ~CCIE;   // All bits TXed, disable int
    txBitCnt = 10;        // Re-load bit counter
  }
  else
  {
    if (txData & 0x01) {
      TA1CCTL0 &= ~OUTMOD2;  // TX Mark '1�
    }
    else
    {
      TA1CCTL0 |= OUTMOD2;
    } // TX Space '0�

  txData >>= 1;
  txBitCnt--;
  }
}
#pragma vector = TIMER1_A1_VECTOR                // Data are received from BT, using P2.1 pin
__interrupt void Timer_A1_ISR(void) {
  static unsigned char rxBitCnt = 8;
  static unsigned char rxData = 0;
  switch (__even_in_range(TA1IV, TA1IV_TAIFG)) {
    case TA1IV_TACCR1:     // TACCR1 CCIFG - UART RX
      TA1CCR1 += UART_TBIT; // Add Offset to CCRx
      if (TA1CCTL1 & CAP) { // On start bit edge
        TA1CCTL1 &= ~CAP;   // Switch to compare mode
        TA1CCR1 += UART_TBIT_DIV_2; // To middle of D0
      } else {             // Get next data bit
        rxData >>= 1;
                if (TA1CCTL1 & SCCI) { // Get bit from latch
          rxData |= 0x80; }
        rxBitCnt--;
        if (rxBitCnt == 0) {  // All bits RXed?
          rxBuffer = rxData;  // Store in global
          rxBitCnt = 8;       // Re-load bit counter
          TA1CCTL1 |= CAP;     // Switch to capture
          __bic_SR_register_on_exit(LPM0_bits);
          // Clear LPM0 bits from 0(SR)
        }
      }
      break;
    }
}
