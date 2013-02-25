/**
 *   Tofer, a time of flight.. er?
 *   Copyright (C) 2013, John Howe
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <msp430.h>
#include <string.h>
#include "stdarg.h"

//In uniarch there is no more signal.h to sugar coat the interrupts definition, so we do it here
#define interrupt(x) void __attribute__((interrupt (x)))

#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))

#define BITMASK_SET(x,y) ((x) |= (y))
#define BITMASK_CLEAR(x,y) ((x) &= (~(y)))
#define BITMASK_FLIP(x,y) ((x) ^= (y))
#define BITMASK_CHECK(x,y) ((x) & (y))

#define true 1
#define false 0

#define LED1 0
#define LED2 5
#define PWM_PIN 6
#define IR_RX_PIN 7
#define UART_TX_PIN 1
#define UART_RX_PIN 2
#define CMDLEN 12

#define DEADTIME 2000

#define FILTER_LENGTH 10
#define READ_GAIN 100
#define IR_HIGH 90
#define IR_LOW 10

void putprintf(char *format, ...);
void putch(char c);
void putstr(const char *str);

void initialise(void);
void bounceImpact(void);
void bounceDepart(void);
unsigned char isValidBounce(void);
interrupt(TIMER0_A1_VECTOR) serviceTimerA(void);
interrupt(PORT1_VECTOR) servicePort1(void);
interrupt(USCIAB0RX_VECTOR) USCIAB0RX_ISR();

enum matState {
        up, down
};

volatile unsigned long ticks; // milliseconds
volatile unsigned short rxticks;
volatile unsigned long lastIrRx;
unsigned long departTick = 0;

unsigned long departTime = 0;
unsigned long impactTime = 0;
unsigned long totalAirTime = 0;
unsigned long bounceNumber = 0;

int main(void)
{
        enum matState mat = down;
        short signalOn = 0;
        long lpFilter = 0;
        short irLevel = 0;

        initialise();
        for (;;) {
                if (rxticks > 10) { // 18 ticks ~500uS or 50% duty at 1ms tickrate
                        BIT_CLEAR(P1SEL, PWM_PIN);
                        if (rxticks > 36-1) { // 1ms tickrate at 36kHz PWM period
                                rxticks = 0;
                                ticks++;
                        }
                } else {
                        BIT_SET(P1SEL, PWM_PIN);
                }

                if ((ticks - lastIrRx) <= 1) {
                        BIT_CLEAR(P1OUT, LED1);
                        signalOn = 1;
                } else {
                        BIT_SET(P1OUT, LED1);
                        signalOn = 0;
                }

                irLevel = lpFilter >> FILTER_LENGTH;
                lpFilter += (READ_GAIN * signalOn) - irLevel;

                if (irLevel > IR_HIGH && isValidBounce()) {
                        if (mat == down) {
                                mat = up;
                                bounceDepart();
                        }
                        BIT_SET(P1OUT, LED2);
                } else if (irLevel < IR_LOW && isValidBounce()) {
                        if (mat == up) {
                                mat = down;
                                bounceImpact();
                        }
                        BIT_CLEAR(P1OUT, LED2);
                }



                LPM1;
        }
}

unsigned char isValidBounce(void)
{
#define MIN_BOUNCE_TIME 200
        return (ticks - impactTime > MIN_BOUNCE_TIME) && (ticks - departTime > MIN_BOUNCE_TIME);
}

void bounceImpact(void)
{
        unsigned long airTime = impactTime - departTime;
        impactTime = ticks;
        totalAirTime += airTime;
        if (impactTime - departTime > DEADTIME) {
                putstr("\n\r");
                bounceNumber = 1;
                totalAirTime = 0;
        }
        if (bounceNumber % 10 == 1) {
                putprintf("\n\rBounce\tAirtime\tTotal\tMatTime");
        }
        putprintf("\n\r%l\t%l\t%l\t", bounceNumber++, airTime, totalAirTime);
}

void bounceDepart(void)
{
        departTime = ticks;
        if (departTime - impactTime > DEADTIME) {
                putstr("\n\r");
                bounceNumber = 1;
                totalAirTime = 0;
        }
        putprintf("%l\t", departTime - impactTime);
}

void initialise(void)
{
        /* Stop watchdog timer */
        WDTCTL = WDTPW + WDTHOLD;

        /* Set internal clock frequency to 1MHz */
        DCOCTL = CALDCO_16MHZ;
        BCSCTL1 = CALBC1_16MHZ;

        /* Initialise I/O ports */
        P1OUT = 0;
        BIT_SET(P1DIR, LED1);
        BIT_SET(P1DIR, LED2);

        BIT_CLEAR(P1DIR, IR_RX_PIN);
        BIT_SET(P1IFG, IR_RX_PIN);
        BIT_SET(P1IE, IR_RX_PIN);

        /* Initialise PWM for IR led */
        BIT_SET(P1DIR, PWM_PIN);
        BIT_SET(P1SEL, PWM_PIN);
        TACCR0 = 442; // PWM period (36kHz at CALDCO_16MHZ)
        TACCR1 = 110; // PWM duty cycle
        TACCTL1 = OUTMOD_7; // CCR1 reset/set
        TACTL = TASSEL_2 + MC_1; // Chooses SMCLK, and Up Mode.
        TACTL |= TAIE; // Enable interrupt

        /* Initialise UART */
        BIT_SET(P1SEL, UART_RX_PIN); // Set pin modes to USCI_A0
        BIT_SET(P1SEL2, UART_RX_PIN);
        BIT_SET(P1SEL, UART_TX_PIN);
        BIT_SET(P1SEL2, UART_TX_PIN);

        UCA0CTL0 = 0;
        UCA0CTL1 = UCSSEL_2;
#define UCA0BR 1664 // 16 MHz -> 9600
        UCA0BR0 = UCA0BR & 0xff;
        UCA0BR1 = (UCA0BR & 0xff00)>>8;
        UCA0MCTL = UCBRS1; // Modulation UCBRSx = 1
        UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine
        IE2 |= UCA0RXIE; // Enable USCI_A0 RX interrupt

        putstr("AT+NAMETofer "); /* Init bluetooth module */

        _BIS_SR(GIE); // Global interrupt enable

        ticks = 0;
        rxticks = 0;
        lastIrRx = 0;
}

void putstr(const char *str)
{
        unsigned short i;
        unsigned short len = strlen(str);
        for (i = 0; i < len; i++) {
                putch(str[i]);
        }
}

void putch(char c)
{
        // wait for TXBUF to complete last send...
        // UCA0TXIFG is high when UCA0TXBUF is empty
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = c;
}


static const unsigned long dv[] = {
//      4294967296,     // 32 bit unsigned max
        1000000000,     // +0
         100000000,     // +1
          10000000,     // +2
           1000000,     // +3
            100000,     // +4
//           65535,     // 16 bit unsigned max
             10000,     // +5
              1000,     // +6
               100,     // +7
                10,     // +8
                 1,     // +9
};

static void xtoa(unsigned long x, const unsigned long *dp)
{
        char c;
        unsigned long d;
        if(x) {
                while(x < *dp) {
                        ++dp;
                }
                do {
                        d = *dp++;
                        c = '0';
                        while(x >= d) ++c, x -= d;
                        putch(c);
                } while(!(d & 1));
        } else {
                putch('0');
        }
}

static void puth(unsigned n)
{
        static const char hex[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
        putch(hex[n & 15]);
}

void putprintf(char *format, ...)
{
        if (ticks <= 1000) { // Hack to allow bluetoth to set name
                return;
        }
        char c;
        int i;
        long n;

        va_list a;
        va_start(a, format);
        while(*format) {
                c = *format++;
                if(c == '%') {
                        switch(c = *format++) {
                                /*
                                   case 's':                       // String
                                   putstr(va_arg(a, char*));
                                   break;
                                   case 'c':                       // Char
                                   putch(va_arg(a, char));
                                   break;
                                   */
                                case 'i':                       // 16 bit Integer
                                case 'u':                       // 16 bit Unsigned
                                        i = va_arg(a, int);
                                        if(c == 'i' && i < 0) i = -i, putch('-');
                                        xtoa((unsigned)i, dv + 5);
                                        break;
                                case 'l':                       // 32 bit Long
                                case 'n':                       // 32 bit uNsigned loNg
                                        n = va_arg(a, long);
                                        if(c == 'l' &&  n < 0) n = -n, putch('-');
                                        xtoa((unsigned long)n, dv);
                                        break;
                                case 'x':                       // 16 bit heXadecimal
                                        i = va_arg(a, int);
                                        puth(i >> 12);
                                        puth(i >> 8);
                                        puth(i >> 4);
                                        puth(i);
                                        break;
                                case 0: return;
                                default: goto bad_fmt;
                        }
                } else {
                        bad_fmt:    putch(c);
                }
        }
        va_end(a);
}

/******************************/
/* Interrupt service routines */
/******************************/

interrupt(TIMER0_A1_VECTOR) serviceTimerA(void)
{
        TACTL &= ~TAIFG;
        LPM1_EXIT;
        rxticks++;
}


interrupt(PORT1_VECTOR) servicePort1(void)
{
        if (BIT_CHECK(P1IFG, IR_RX_PIN)) {
                BIT_CLEAR(P1IFG, IR_RX_PIN);
                lastIrRx = ticks;
        }
}

/*
 * Accept a command entered on the serial terminal
 */
interrupt(USCIAB0RX_VECTOR) USCIAB0RX_ISR()
{
        char cmd[CMDLEN];
        unsigned short i;

        /*
         *  UCA0RXIFG is high when UCA0RXBUF has received a complete character.
         *  However, it rarely stays high more than one byte at a time.  Perhaps
         *  because I type too slow on my keyboard.  Because of this all the
         *  commands are a single character for now.
         */
        for (i = 0; (IFG2 & UCA0RXIFG) && i < CMDLEN; i++) {
                cmd[i] = UCA0RXBUF;
        }
        cmd[i] = '\0';

        putstr("\n\r");
        bounceNumber = 1;
        totalAirTime = 0;
}

