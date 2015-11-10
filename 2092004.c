/******************************************************************************
 *
 * This firmware has been written by Know How & Technologies s.a.s
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * This software works for hardware project 2092004, don't try it elsewhere.
 *
 * JUMPER FUNCTIONS
 * 
 * JUMPER 1 : OPEN FOR DEBUG
 * 
 * Eng. Alessio Palma
 * 
 ******************************************************************************/
#include<avr/io.h>
#include<stdio.h>
#include<inttypes.h>
#include<avr/eeprom.h>
#include<avr/interrupt.h>
#include<avr/signal.h>

/*
 * Definizione delle costanti relativamente all'hardware del 20 settembre 2004
 * 
 * GESTIONE SENSORI
 * GESTIONE MOTORE
 * GESTIONE I/O DI DATI
 */

#define S1 1 
#define S2 0
#define S3 3
#define S4 2
#define SENSOR_PORT PINB 

#define MFS 0
#define MRS 1
#define MOTOR_PORT PORTA

#define RCP 6
#define RDT 7
#define WCP 4
#define FGP 5
#define WDT 2
#define CONT 3
#define WCTRL_PORT PORTA
#define RCTRL_PORT  PINA

#define IN1 3
#define IN2 2
#define OUT1 0
#define OUT2 1
#define IN_PORT PINC
#define OUT_PORT PORTC

#define JUMPER_PORT PIND

#define __DEBUG1 
//#define __DEBUG2

#define WRITE_0 cbi( WCTRL_PORT, WDT )
#define WRITE_1 sbi( WCTRL_PORT, WDT )
#define LOOP_UNTIL_WCP_IS_1   loop_until_bit_is_set( RCTRL_PORT, WCP )
#define LOOP_UNTIL_WCP_IS_0   loop_until_bit_is_clear( RCTRL_PORT, WCP )
#define LOOP_UNTIL_RCP_IS_1   loop_until_bit_is_set( RCTRL_PORT, RCP )
#define LOOP_UNTIL_RCP_IS_0   loop_until_bit_is_clear( RCTRL_PORT, RCP )		
#define READ_1		      bit_is_set( RCTRL_PORT, RDT )
#define RCP_IS_0	      bit_is_clear( RCTRL_PORT, RCP )

#define BADGE_MAXBIT	   0xfe
#define BADGE_SYNC_OFFSET  8
#define BADGE_SYNC_SILENCE 4
#define BADGE_DATA_BIT	   20

/*
 * deBUG's defines
 */
#define BUG_VALUE_BYTE1 0x0e
#define BUG_ENCODE_OP   0x0f
#define BUG_DECODE_OP   0x10
#define BUG_READ_OP     0x11
#define BUG_WRITE_OP    0x12
#define BUG_INIT_OP     0x13
#define BUG_MODE_PROG   0x14
#define BUG_MODE_NORMAL 0x15
#define BUG_MAIN	0x16
#define BUG_READ_BIT1   0x17
#define BUG_READ_BIT0   0x18
#define BUG_WRITE_BIT0  0x19
#define BUG_WRITE_BIT1  0x1A
#define BUG_READ_CP1 	0x1b
#define BUG_READ_CP2    0x1c
#define BUG_DECODE_OFFSET 0x1d
#define BUG_DECODE_RESULT 0x1e
#define BUG_BUFFER	0x30

#define BUG_NORMAL_IN1  0x1d
#define BUG_NORMAL_IN2  0x1e

#define BUG_OK          0x01
#define BUG_KO          0x10

/*
 * bit buffer
 * 
 * Questo buffer contiene la sequenza di bit che sarà inviata e ricevuta dal lettore, le routine 
 * di condifica delle sequenze binarie permettono di recuperare e codicare il numero da 
 * scrivere sul badge.
 */

uint8_t  bit[255]; 

/*
 * general purpose delay loop
 */

void delay( void ) {
unsigned int i = 0xffff;

	while( i > 0 ) i--;
}

void buffer_clear( void ) {
unsigned int i;

	for (i=0; i< BADGE_MAXBIT; bit[ i++] = 0);
}

/****f*
 * Name
 * 	motor_[forward|reverse|off]()
 * Synopsis
 * 	motor_forward()
 * Notes
 * Omron engine driver functions
 */

void motor_forward( void ) {	
	cbi( MOTOR_PORT, MFS);
	sbi( MOTOR_PORT, MRS);	
}

void motor_reverse( void ) {
	cbi( MOTOR_PORT, MRS);
	sbi( MOTOR_PORT, MFS);
}

void motor_off( void ) {
	cbi( MOTOR_PORT, MRS );
	cbi( MOTOR_PORT, MFS );
}


void read_mode( void ) {
	sbi( WCTRL_PORT, CONT );
}

void write_mode( void ) {
	cbi( WCTRL_PORT, CONT );
}



/****f*
 * Name
 *	jumper_is_set( number )
 * synopsis 
 *      
 * Notes
 * It returns jumper status (x=1,2,3)
 */

unsigned short int jumper_is_set( unsigned short x ) {
	return bit_is_set( JUMPER_PORT, x-1);
}

/****f*
 * Name
 *	card_front_position(), card_front_head(), card_transfer(), card_read_position()
 * synopsis 
 *      
 * Notes
 * These functions returns a unsigned short value which show sensors status inside OMRON reader/writer
 */


unsigned short card_front_position( void ) {
	return bit_is_clear( SENSOR_PORT, S1);
}

unsigned short card_front_head( void ) {
	return bit_is_clear( SENSOR_PORT, S2);
}

unsigned short card_transfer( void ) {
	return bit_is_clear( SENSOR_PORT, S3);
}

unsigned short card_rear_position( void ) {
	return bit_is_clear( SENSOR_PORT, S4);	
}


/****f*
 * Name
 *	in( number)
 * synopsis 
 *      
 * Notes
 * Two 4n25 photocouplers are on board to allow PLCs to comunicate with ATMEGA16L,
 * use this function to read inputs status.
 */


unsigned short in( unsigned short x )
{
	return ( ( x == 1 ) ? bit_is_set( IN_PORT, IN1 ) : bit_is_set( IN_PORT, IN2 ) );
}


/****f*
 * Name
 *	out_[on|off]( number )
 * synopsis 
 *      
 * Notes
 * Actually it opens and closes relays on board. Since relay are outputs, prefix has been 
 * selected to be out_ and not relay_
 */


void out_on( unsigned short x ) {
	if ( x == 1 ) sbi( OUT_PORT, OUT1);
	else sbi( OUT_PORT, OUT2 );
}

void out_off( unsigned short x ) {
	if ( x == 1 ) cbi( OUT_PORT, OUT1);
	else cbi( OUT_PORT, OUT2 );	
}



/****f*
 * Name
 *	card_load()
 * synopsis 
 *      
 * Notes
 * Load badge until positiong it close to read/write head
 */


void card_load ( void ) {
unsigned long int timer = 0xf000;

	motor_forward();
	while ( --timer && (!card_front_head()) );
	motor_off();
}


/****f*
 * Name
 *	card_eject()
 * synopsis 
 *      
 * Notes
 * Eject badge from reader
 */

void card_eject ( void ) {
unsigned long int timer = 0xf000;

	motor_reverse();
	while ( ! card_front_position() );
	while ( --timer && card_front_position() );
	motor_off();
	while ( card_front_position() );
}


/****f*
 * Name
 *	card_eject()
 * synopsis 
 *      
 * Notes
 * retro-Eject badge 
 */

void card_retro_eject ( void ) {
unsigned long int timer = 0xf000;

	motor_forward();
	while ( ! card_rear_position() );
	while ( --timer && card_rear_position() );
	motor_off();
}

/****f*
 * Name
 *	relay_init()
 * synopsis 
 *      
 * Notes
 * It opens relays.
 */

void relay_init( void ) {

	out_off(1);
	out_off(2);	
}

/****f*
 * Name
 *	card_init()
 * synopsis 
 *      
 * Notes
 * Mechanical initialization, if there is a card inside eject it.
 */

void card_init( void ) {
unsigned short i = 4;	

	read_mode();
	
	do { 
		motor_reverse();
		delay();
		if ( ! --i ) break;
	} while ( card_front_position() | card_front_head() | card_transfer() | card_rear_position() );

	motor_off();
}

/*
 * porta la card sotto la testista di scrittura
 */

void card_under_head( void ) {

	if ( card_rear_position() | card_transfer() | card_front_head() ) {
		motor_reverse();
		while ( ! card_front_position() ) ;
	}

	motor_forward();
	while ( ! card_transfer() );
}


/****f*
 * Name
 *	avr_init()
 * synopsis 
 *      
 * Notes
 * AVR ATMEGA16L initialization issues specific
 */

void avr_init( void ) {
	
	/* 
	 * primi 4 bit della porta B come IN (lettura sensori)
	 */
	
	outp( 0x00, DDRB);
	
	/*
	 * Primi 4 bit della porta A come ingressi 
	 * Dal lettore: RCP, RDT, WCP, FGP 
	 * Al lettore: WDT, CONT, MFS, MRS
	 */
	
	outp( 0x0f, DDRA);

	/*
	 * Disattiva la porta JTAG (PORTC) 
	 */
	
	cli();
	asm volatile("push r16;");
	asm volatile("in r16, 0x0034;");
	asm volatile("ori r16, 0x80;");
	asm volatile("out 0x0034, r16;");
	asm volatile("out 0x0034, r16;");
	asm volatile("pop r16");
	sei();

	/*
	 * primi 2 bit della porta C come uscite 
	 * i successivi due come ingressi
	 */
	
	outp( 0x03, DDRC );

	/*
	 * Alta impedenza per i bit 0 ed 1 della porta C
	 */

	sbi( PORTC, 0); sbi( PORTC, 1);

	/*
	 * Gestione jumpers, i primi 3 bit della porta D come ingressi
	 */

	outp( 0xf0, DDRD );

#ifdef __DEBUG1
	eeprom_write_byte( (void *)BUG_INIT_OP, BUG_OK );
#endif
	
}

/****f*
 * name
 * 	encode
 * synopsis
 * 	encode number to record it on the badge
 */

void encode( uint32_t numero ) {
int16_t i;

#ifdef __DEBUG1
	eeprom_write_byte( (uint8_t *)BUG_ENCODE_OP, BUG_OK );
#endif

	buffer_clear(); // buffer bit azzerato
	
	for (i=BADGE_DATA_BIT-1 ; i >= 0 ; i--) { //   i=0 ; i <= BADGE_DATA_BIT-1 ; i++
		bit[ i + BADGE_SYNC_OFFSET + BADGE_SYNC_SILENCE ] =  ( numero & 0x01 ) ? 1 : 0 ;
		numero >>= 1;
	}
	
	/* 0123456789ABCD	        */
	/* 00001010010100 sync-sequence */
	/* ^^^^ -> silence before sync  */
	
	bit[4] = bit[6] = bit[11] = bit[9]  = 1;

#ifdef __DEBUG1
	
	while( jumper_is_set(3) ) {
		for ( i = 0; i <40; i++ ) {
			if ( bit[i] ) sbi( PORTD, 7);
			else cbi( PORTD, 7);
		}
	}
	
#endif	
}


/****f*
 * name
 * 	uint32_t decode( )
 * synopsis
 * 	align squence bit using synce-sequence to return the correct number
 */

uint32_t decode( void  ) {
int16_t i=0,j=0;
uint32_t result = 0;

	
#ifdef __DEBUG1
	eeprom_write_byte( (uint8_t *)BUG_DECODE_OP, BUG_KO);
#endif
	
        while( ! ( (bit[i]+bit[2+i]+bit[5+i]+bit[7+i]==4) && (bit[1+i]+bit[3+i]+bit[4+i]+bit[6+i]==0) ) )
		if ( ++i == 0x10 ) break;
        

#ifdef __DEBUG1	
	eeprom_write_byte( (void *)BUG_DECODE_OFFSET, i );	
#endif

	
        if ( i < 0x10 )
                for (j=BADGE_DATA_BIT-1; j>=0; j-- ) 
			result += ((uint32_t)bit[BADGE_SYNC_OFFSET + i + j]) << (BADGE_DATA_BIT-1 -j);

	
#ifdef __DEBUG2

	for (j=0; j <= 160 ; j++ ) {
		eeprom_write_byte( (uint8_t *)(0x30 + j), bit[ j ] ); 
	}

#endif

#ifdef __DEBUG1
	
	eeprom_write_byte( (uint8_t *)7, (uint8_t ) result & 0xff);
	eeprom_write_byte( (uint8_t *)6, (uint8_t ) ( result >> 8 ) & 0x0ff );
	eeprom_write_byte( (uint8_t *)5, (uint8_t ) ( result >> 16 ) & 0x0ff );
	eeprom_write_byte( (uint8_t *)4, (uint8_t ) ( result >> 24 ) & 0x0ff );
	eeprom_write_byte( (uint8_t *)BUG_DECODE_OP, BUG_OK);
	
#endif
	
	return result;
}

/****f*w
 *
 * Name
 *	card_write( unsigned int )
 * synopsis 
 *      
 * Notes
 * This function write the number on the badge
 */

void card_write( uint32_t x ) {
uint8_t bitcounter = 0;

#ifdef __DEBUG1
	eeprom_write_byte( (uint8_t *)BUG_READ_OP, BUG_KO );
#endif
	
	encode( x );

	write_mode();
	WRITE_0;

	card_under_head();	
	motor_forward();
	
	while ( card_transfer() ) {
		
		LOOP_UNTIL_WCP_IS_0;
		
		if ( bitcounter++ < 32) {

			if ( bit[bitcounter]  ) { 
				WRITE_1; 
				if ( jumper_is_set(1) )	sbi( PORTD, 7);
			} else { 
				WRITE_0; 
				if ( jumper_is_set(1) )	cbi( PORTD, 7); 
			}		
	
		}
		
		LOOP_UNTIL_WCP_IS_1;
	}
		
	WRITE_0;
	while( ! card_rear_position() );
		
	read_mode();
	
#ifdef __DEBUG1
	eeprom_write_byte( (uint8_t *)BUG_WRITE_OP, BUG_OK );
#endif	
}


/****f*
 * Name
 *	card_read()
 * synopsis 
 *      
 * Notes
 * It reads the number in the badge.
 * To read a badge we do: start bagde moving under head, look for start bit. When found collect bits that follow.
 * you have to be sure about write routine start writing silence 
 */
void card_read( void ) { 
uint8_t bitcounter=0, i=0;

#ifdef __DEBUG1
uint8_t bit_1=0, bit_0=0; // DEBUG //
#endif

	card_under_head();
	read_mode();
	motor_forward();
	
	buffer_clear(); // svuota il bit buffer per i nuovi dati.
	
	while ( card_transfer() ) {
		
		if ( RCP_IS_0 ) {
			
			if ( READ_1 ) { 
				bit[ bitcounter ] = 1;
#ifdef __DEBUG1				
				sbi( PORTD, 7); // DEBUG //
				bit_1++;        // DEBUG //
				
#endif				
			} else {

				bit[ bitcounter ] = 0;
#ifdef __DEBUG1				
				cbi( PORTD, 7); // DEBUG //
				bit_0++;        // DEBUG //
#endif
			}

			bitcounter++;
			LOOP_UNTIL_RCP_IS_1; 
		}

	}

	motor_off();

	while( jumper_is_set(1) ) {

		for ( i = 0; i < bitcounter; i++ ) {
			if ( bit[i] ) sbi( PORTD, 7);
			else cbi( PORTD, 7);
		}
	}
	

#ifdef __DEBUG1
	eeprom_write_byte( (void *)BUG_READ_BIT1, bit_1 );
	eeprom_write_byte( (void *)BUG_READ_BIT0, bit_0 );
#endif
}


/****f*
 * Name
 *	init_hardware()
 * synopsis 
 *      
 * Notes
 * Hardware initialization
 */
void init_hardware( void ) {
	
	avr_init();
	card_init();
	relay_init();
	read_mode();
	
}	

/****f*
 * Name
 *	mode_prog()
 * synopsis 
 *      
 * Notes
 * Used to manage program mode
 */

void mode_prog( void ) {
	
uint8_t credit;
#ifdef __DEBUG1
	eeprom_write_byte( (void *)BUG_MODE_PROG, BUG_OK );
#endif
		
	credit = 4;
	
	
	while(1) {
			
		while ( ! card_front_position() ) ;		
		card_load();		
		
		if ( card_front_head() )
			card_write( (1+1*jumper_is_set(1)+2*jumper_is_set(2)+4*jumper_is_set(3))*credit );		
		
		if ( in(1) ) card_eject();
		else card_retro_eject();
	}
}

/****f*
 * Name
 *	mode_normal()
 * synopsis 
 *      
 * Notes
 * Used to manage normal mode
 */

void mode_normal( void ) {
uint32_t uint_credito=0;
	
	while (1) {
	
		while ( ! card_front_position() );
			
		card_load(); 
		
		if ( card_front_head() ) {
				
			card_read();
			uint_credito = decode();
		
			if ( uint_credito != 0 ) {
					
				if ( 1 | ( uint_credito < jumper_is_set(1)+jumper_is_set(2)+jumper_is_set(3)+1  ) ); 
					out_on(1);
						
				out_on(2);
				
				while( ! ( in(1) | in(2) ) ); 
					
				if ( in(1) ) 
					card_write( --uint_credito );
					
				out_off(1);
				out_off(2);
			}
		}
		
		card_eject();
	}			
} 


int main ( void ) {

	init_hardware();

	if ( jumper_is_set(2) ) mode_prog();
	else mode_normal();

#ifdef __DEBUG1
	eeprom_write_byte( (void *)BUG_MAIN, BUG_KO );
#endif	
		
	return 1;	
}

