
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	File Name:	blinders.c  	
	Author:		Syed Tafazul Hassan
	Date:		26/07/2023
	Modified:	Name & date or None

	Description: 	 
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

// Preprocessor >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// Libraries >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#include <p18f45k22.h>
#include <stdlib.h>
#include "pragmas.h"


// Constants >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define TRUE			1
#define	FALSE			0	
#define BROWNOUT		3.7
#define VREF			5.0
#define VRESV			(5.0/1024)
#define BROWNOUT_LED 	0xFB
#define PWRON_LED		0xEF
#define RAIL_5V			ADC_CH27
#define	LDR				ADC_CH5
#define TMR0PSC			0xFC17
#define BYTE			8
#define TMR0INTFLAG		INTCONbits.TMR0IF
#define MAXSTEP			250
#define LUXVOLT_MON		450
#define PB_FORWARD		0x02 // change it according to schematics pushbutton
#define PB_BACKWARD		0x01 // change it according to schematics pushbutton
#define LUXVOLT_NIGHT 	100
#define LUXMONFLAG		FALSE
#define LUXNIGHTFLAG	FALSE
#define PBMASK			0x03
#define PBSTATE			(PORTC & PBMASK)
#define NOPRESS			0x00
#define RESISTANCE		10000
// Global Variables >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
typedef char uint8_t;
typedef enum{PIN0 = 0,PIN1,PIN2,PIN3,PIN4,PIN5,PIN6,PIN7}pins_t;
typedef enum{PORT_A = 'A',PORT_B = 'B',PORT_C = 'C', PORT_D = 'D',PORT_E = 'E' }ports_t;
typedef enum{ADC_CH0 = 0, ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8,
			 ADC_CH9, ADC_CH10,ADC_CH11,ADC_CH12,ADC_CH13,ADC_CH14,ADC_CH15,ADC_CH16,ADC_CH17,
			 ADC_CH18,ADC_CH19,ADC_CH20,ADC_CH21,ADC_CH22,ADC_CH23,ADC_CH24,ADC_CH25,ADC_CH26,
			 ADC_CH27}ADC_CHANNEL_t;

static char steptable[8] = {0x20,0x30,0x10,0x18,0x08,0x0C,0x04,0x24};
//static char steptable[8] = {0x20,0x10,0x08,0x04,};
static int stepCount = 0;
static char step = 0;
char luxmonflag = FALSE;
char luxnightflag = FALSE;
static char morningflag = FALSE;
static char nightflag = FALSE;
static char lastState = 0x00;
// Functions >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

/*>>> portsInit: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Syed Tafazul Hassan
Date:		26/07/2023
Modified:	Name & date or None
Desc:		unsued PORT pins is configured as Digital Input
Input: 		None 
Returns:	None 
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void portsInit(void)
{
	LATA = 0x00;
	TRISA = 0xFF;
	ANSELA = 0x00;

	LATB = 0x00;
	TRISB = 0xC3;
	ANSELB = 0x00;
	
	LATC = 0x00;
	TRISC = 0xFF;
	ANSELC = 0x00;

	LATD = 0x00;
	TRISD = 0xEB;
	ANSELD = 0x00;

	LATE = 0x00;
	TRISE = 0xFF;
	ANSELE = 0x00;


}// eo portsInit::

/*>>> setOscillation: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Syed Tafazul Hassan
Date:		26/07/2023
Modified:	Name & date or None
Desc:		Initialise the Oscillator at 4MHz and stay there until it reaches stable
			frequency
Input: 		None
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void setOscillation(void)
{
	
	OSCCON = 0x53;
	OSCCON2 = 0x04;
	OSCTUNE = 0x80;
	
	while(OSCCONbits.HFIOFS != TRUE);
	

}// eo setOscillation::


/*>>> ledInit: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Syed Tafazul Hassan
Date:		26/07/2023
Modified:	Name & date or None
Desc:		Initializing the PORTD PWRON_LED Pin
Input: 		None
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void ledInit(void)
{
	TRISD |= 0x0B;
	LATD &= PWRON_LED;
}// eo ledInit::


/*>>> adcPinInit: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Syed Tafazul Hassan
Date:		26/07/2023
Modified:	Name & date or None
Desc:		initialize the PORTx and Pinx for ADC
Input: 		pins_t port_pin: pin number(PIN0 - PIN7) 
			ports_t port: PORT from A-E(case insensitive)
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void adcPinInit(pins_t port_pin,ports_t port)
{

	switch(port)
	{
		case 'A':
		case 'a':

			TRISA |= (1<<port_pin);
			ANSELA |=  (1<<port_pin);
			break;

		case 'B':
		case 'b':

			TRISB |= (1<<port_pin);
			ANSELB |= (1<<port_pin);
			break;

		case 'C':
		case 'c':

			TRISC |= (1<<port_pin);
			ANSELC |=  (1<<port_pin);
			break;

		case 'D':
		case 'd':
		
			TRISD |= (1<<port_pin);
			ANSELD |=  (1<<port_pin);
			break;

		case 'E':
		case 'e':

			TRISE |= (1<<port_pin);
			ANSELE |=  (1<<port_pin);
			break;

		default:
			break;
	}

	
	
	
}// eo adcInit::

/*>>> initUART: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Syed Tafazul Hassan
Date:		05/07/2023
Modified:	none
Desc:		Initializing UART with 9600 baudrate with 8-N-1 configuration
Input: 		none
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void initUART(void)
{
	
	SPBRG1 = 25;
	TXSTA1 = 0x24;
	RCSTA1 = 0x90;
	BAUDCON1 = 0x00;
	

}
/*>>> adcInit: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Syed Tafazul Hassan
Date:		26/07/2023
Modified:	Name & date or None
Desc:		configured the ADC module with 12tad, fosc/8, right justified
			and enabled adc module
Input: 		None
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void adcInit(void)
{
	
	ADCON0 = 0x01;
	ADCON1 = 0x00;
	ADCON2 = 0xA9;

}// eo adcInit::

/*>>> adc_ContinousConversion: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Syed Tafazul Hassan
Date:		26/07/2023
Modified:	Name & date or None
Desc:		Can read any ADC channel by passing the channel number.
Input: 		char chID, pass the channel number to read raw data.
Returns:	sensor_t, returns the raw ADC data (right justified).
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
int adc_ContinousConversion(ADC_CHANNEL_t chID)
{
	
	int ADC_Raw = 0;

	ADCON0bits.CHS = chID;
	ADCON0bits.GO = TRUE;
	while(ADCON0bits.GO);
	
	return ADRES;
	
	
}// eo adc_ContinousConversion::

/*>>> timer0Init: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Syed Tafazul Hassan
Date:		26/07/2023
Modified:	Name & date or None
Desc:		Configured the TIMER 0 at 16bit mode 
Input: 		None
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void timer0Init(void)
{
	
	T0CON = 0x84;
	TMR0H = (TMR0PSC >> BYTE);
	TMR0L = TMR0PSC;
	

}// eo timer0Init::

float rawtoVolt(ADC_CHANNEL_t chID)
{
	float volt = 0.0;
	int raw = 0;
	raw = adc_ContinousConversion(chID);
	volt = raw * VRESV;

	return volt;
}

/*>>> brownOut: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Syed Tafazul Hassan
Date:		26/07/2023
Modified:	Name & date or None
Desc:		
Input: 		None
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void brownOut(void)
{	
	float currentVolt = 0.0;
	currentVolt = rawtoVolt(RAIL_5V);
	if(currentVolt <= BROWNOUT)
	{	
		
		while(currentVolt <= BROWNOUT)
		{
			currentVolt = rawtoVolt(RAIL_5V);
			LATD =  BROWNOUT_LED;
		}
		
	}
	else 
	{
		LATD = PWRON_LED;
	}
}// eo brownOut::

/*>>> psc_Reload: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Syed Tafazul Hassan
Date:		26/07/2023
Modified:	Name & date or None
Desc:		Resetting the interrupt flag to FALSE and reloading the PSC after
			timer rollover 
Input: 		None
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void psc_Reload(void)
{
	TMR0H = (TMR0PSC >> BYTE);
	TMR0L = (TMR0PSC);
	INTCONbits.TMR0IF = FALSE;
}//eo psc_Reload::

void steppermotor_Backward(void)
{

	if(stepCount == 0)	
	{	

		if(luxnightflag	== TRUE)
		{
			luxnightflag	= FALSE;
			nightflag   	= TRUE;
			morningflag 	= FALSE;
			luxmonflag 		= FALSE;
		
		}
		else
		{
			lastState		= 0xFF;
		}

	}

	else
	{

		
		LATB = steptable[step];

		if(step <= 0)
		{
			step = 8;
		}
	
		step--;
		stepCount--;
	
		#if DEBUG
			printf("stepCount = %d\n\r",stepCount);
		#endif
 	}

}


void steppermotor_Forward(void)
{


	if(stepCount > MAXSTEP)
	{	
		if(luxmonflag == TRUE)
		{
			luxmonflag 		= FALSE;
			luxnightflag	= FALSE;
			morningflag  	= TRUE;
			nightflag		= FALSE;

		}
		else
		{
			lastState		= 0xFF;
		}

	}

	else
	{

		
			LATB = steptable[step];
			if(step == 7)
			{
				step = -1;
			}
	
			step++;		
			stepCount++;
		
		#if DEBUG
			printf("stepCount = %d\n\r",stepCount);
		#endif
	}
}

void ldr_Sampling(void)
{

	static  float ldrvolt = 0;
	static int samplingvlts[30];
	static char index = 0;
	float amps = 0.0;
	float uamps = 0.0;
	int	lux	= 0;
	samplingvlts[index]= adc_ContinousConversion(LDR);
	
	index++;
	
	if(index > 30)
	{	
		index = 0;
		for(index =0;index < 30 ;index++)
		{
			ldrvolt += samplingvlts[index];
		}

	//Taking average of 30 samples and converting into LUX
		ldrvolt = (ldrvolt/30) * VRESV;
		amps = (ldrvolt/RESISTANCE);
		uamps	= amps * 1000000;
		lux		= uamps * 2;

		if(lux > LUXVOLT_MON && morningflag == FALSE)
		{
			luxmonflag = TRUE;
		}

		else if(lux < LUXVOLT_NIGHT && nightflag == FALSE)
		{
			luxnightflag = TRUE;
		}
		index = 0;
	}


}
/*>>> systemInit: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Syed Tafazul Hassan
Date:		26/07/2023
Modified:	Name & date or None
Desc:		initialise the required pheripherals and pins
Input: 		None
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void systemInit(void)
{
	setOscillation();
	portsInit();	
	adcPinInit(PIN0,PORT_E);//5v rail adc pin
	adcPinInit(PIN7,PORT_D);//LDR sensor adc pin
	adcInit();
	timer0Init();
	initUART();
	ledInit();
}// eo systemInit::


/*>>> MAIN: FUNCTION >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void main( void )
{
	float railVolt = 0.0;
	float ldrVolt = 0.0;
	char pushButton = 0x00;


	systemInit();
	
	while(1)
	{
		pushButton = PBSTATE;
		if(pushButton != NOPRESS && pushButton != lastState)
		{
			lastState = pushButton;
		}
		
		if(INTCONbits.TMR0IF == TRUE)
		{
			psc_Reload();
			brownOut();	//Checking supply voltage for brownout
			ldr_Sampling(); // sampling the incoming data for (TMR0 * 30)ms
		
			if(luxmonflag == TRUE || PB_FORWARD == lastState)
			{
				
				steppermotor_Forward();
				
			}//eo if morning
		
			
			else if(luxnightflag == TRUE || PB_BACKWARD == lastState)
			{
					
				steppermotor_Backward();
			}//eo else if night

		}
			#if DEBUG	// make the DEBUG TRUE if you want to check data on UART
				printf("LDR = %f\n\rRAIL=%f",ldrVolt,railVolt);
			#endif


	}//eo while


}// eo main::

