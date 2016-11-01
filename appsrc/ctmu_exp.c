//
//	ctmu_exp.cpp
//   for PIC18F67J94
//
#include	"system.h"

#define	MAX_TOUCH_SENSOR	24

/*----------------------------------------------------------------------------*/
//	Initialize
/*----------------------------------------------------------------------------*/
void initTouchSensor( void )
{
	//setup A/D converter
	for ( int i=0; i<10; i++ ) __delay_ms(10);

	ANCON1 = 0xff;			//	set AN0-7
	ANCON2 = 0xff;			//	set AN8-11, AN16-19
	ANCON3 = 0x00;			//	no AD port in 67J94

	ADCTMUEN1H = 0x00;
	ADCTMUEN1L = 0x00;
	ADCTMUEN0H = 0x00;
	ADCTMUEN0L = 0x01;

	ADCON5L = 0x00;
	ADCON5H = 0x00;			//	CTMUREQ
	ADCON3L = 0x02;			//	xx*2/Fosc = Tad
	ADCON3H = 0x1f;			//	System Clock, 31Tad
	ADCON2L = 0x00;
	ADCON2H = 0x08;			//	BUFREGEN
	ADCON1L = 0x00;
	ADCON1Hbits.FORM = 0b00;	//	Absolutefractionalresult,unsigned,left-justified
	ADCON1Hbits.MODE12 = 1;		//	12bit
	ADCON1Hbits.ADON = 1;		//Turn On A/D
	__delay_us(10);

	//setup CTMU
	CTMUCON4 = 0x00;
	CTMUCON3 = 0x00;
	CTMUCON2bits.IRNG = 0b10;		//5.5uA
	CTMUCON2bits.ITRIM = 0b000000;	//Nominal - No Adjustment
	CTMUCON1 = 0x80;
}
/*----------------------------------------------------------------------------*/
//	Measure
/*----------------------------------------------------------------------------*/
uint16_t measureTouchSenser( int idx )
{
	uint16_t	immediateValue;
	unsigned long		sum = 0;

	if ( idx >= MAX_TOUCH_SENSOR ){ return 0; }

	int	adch = idx;
	ADCHS0L = adch;		//select A/D channel
	__delay_us(10);


	for ( int i=0; i<4; i++ ){

		CTMUCON1bits.IDISSEN = 1;	//Drain any charge on the A/D circuit
		__delay_us(30);
		CTMUCON1bits.IDISSEN = 0;	//Stop discharge of A/D circuit

//		CTMUCON4bits.EDG2STAT = 0;
//		CTMUCON4bits.EDG1STAT = 1;	// Set edge1 - Start Charge
		__delay_us(20);
 		CTMUCON4bits.EDG1STAT = 0;	//Clear edge1 - Stop Charge

		ADCON1Lbits.SAMP = 1;		//Set Go bit to begin A/D conversion
		__delay_us(20);
		ADCON1Lbits.SAMP = 0;

		while(!ADCON1Lbits.DONE);		//Wait for the A/D conversion to finish

		immediateValue = ADCBUF0H;
		immediateValue = immediateValue<<8;
		immediateValue = immediateValue + ADCBUF0L;//Read the value from the A/D conversion
		sum += immediateValue;
	}

	return (uint16_t)(sum/4);
}
