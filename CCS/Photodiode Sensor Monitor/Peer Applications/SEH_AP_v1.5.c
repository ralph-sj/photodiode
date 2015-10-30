// AP_all_sensors_transmit_data.c
// Recieve data from ED and display through COM port.
// 18/09/2015
// use with corresponding ED

#include "bsp.h"
#include "mrfi.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"

#include "msp430x22x4.h"
#include "vlo_rand.h"

#define MESSAGE_LENGTH		19	// must be less than or equal to MAX_APP_PAYLOAD	//
void TXString( char* string, int length );
void MCU_Init(void);
void transmitData(int addr, signed char rssi,  char msg[MESSAGE_LENGTH] );
void transmitDataString(char addr[4],char rssi[3], char msg[MESSAGE_LENGTH]);
void createRandomAddress(void);

//data for terminal output
const char splash[] = {"\r\n--------------------------------------------------\r\n     ****\r\n     ****           eZ430-RF2500\r\n     ******o****    Temperature Sensor Network\r\n********_///_****   Copyright 2007\r\n ******/_//_/*****  Texas Instruments Incorporated\r\n  ** ***(__/*****   All rights reserved.\r\n      *********     Version 1.03\r\n       *****\r\n        ***\r\n--------------------------------------------------\r\n"};

// reserve space for the maximum possible peer Link IDs
static linkID_t sLID[NUM_CONNECTIONS] = {0};
static uint8_t  sNumCurrentPeers = 0;

// callback handler
static uint8_t sCB(linkID_t);

// work loop semaphores
static uint8_t sPeerFrameSem = 0;
static uint8_t sJoinSem = 0;
static uint8_t sSelfMeasureSem = 0;

// mode data verbose = default, deg F = default
char verboseMode = 1;
char degCMode =0;

int temperature[4] = {0};

void main (void)
{
  addr_t lAddr;
  bspIState_t intState;
  char *Flash_Addr;                         // Initialize radio address location
  Flash_Addr = (char *)0x10F0;

  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  // delay loop to ensure proper startup before SimpliciTI increases DCO
  // This is typically tailored to the power supply used, and in this case
  // is overkill for safety due to wide distribution.
  __delay_cycles(65000);

  if( CALBC1_8MHZ == 0xFF && CALDCO_8MHZ == 0xFF )// Do not run if cal values
  {
    P1DIR |= 0x03;
    BSP_TURN_ON_LED1();
    BSP_TURN_OFF_LED2();
    while(1)
    {
      __delay_cycles(65000);
      BSP_TOGGLE_LED2();
      BSP_TOGGLE_LED1();
    }
  }

  BSP_Init();

  if( Flash_Addr[0] == 0xFF &&
      Flash_Addr[1] == 0xFF &&
      Flash_Addr[2] == 0xFF &&
      Flash_Addr[3] == 0xFF )
    {
      createRandomAddress();                // Create Random device address at
    }                                       // initial startup if missing
  lAddr.addr[0]=Flash_Addr[0];
  lAddr.addr[1]=Flash_Addr[1];
  lAddr.addr[2]=Flash_Addr[2];
  lAddr.addr[3]=Flash_Addr[3];

  //SMPL_Init();
  SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);

  MCU_Init();
  //Transmit splash screen and network init notification
  TXString( (char*)splash, sizeof splash);
  TXString( "\r\nInitializing Network....", 26 );

  SMPL_Init(sCB);

  // network initialized
  TXString( "Done\r\n", 6);

  // main work loop
  while(1)
  {
    // Wait for the Join semaphore to be set by the receipt of a Join frame from a
    // device that supports and End Device.

    if (sJoinSem && (sNumCurrentPeers < NUM_CONNECTIONS))
    {
      // listen for a new connection
      SMPL_LinkListen(&sLID[sNumCurrentPeers]);
      sNumCurrentPeers++;
      BSP_ENTER_CRITICAL_SECTION(intState);
      if (sJoinSem)
      {
        sJoinSem--;
      }
      BSP_EXIT_CRITICAL_SECTION(intState);
    }

    // if it is time to measure our own temperature...
    if(sSelfMeasureSem)
    {
//    	TXString("\r\n...", 5);
      BSP_TOGGLE_LED1();
      sSelfMeasureSem = 0;
    }

    // Have we received a frame on one of the ED connections?
    // No critical section -- it doesn't really matter much if we miss a poll
    if (sPeerFrameSem)
    {
    	  uint8_t  msg[MESSAGE_LENGTH], len, i;

      // process all frames waiting
      for (i=0; i<sNumCurrentPeers; ++i)
      {
        if (SMPL_Receive(sLID[i], msg, &len) == SMPL_SUCCESS)
        {
          ioctlRadioSiginfo_t sigInfo;
          sigInfo.lid = sLID[i];
          SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SIGINFO, (void *)&sigInfo);
          transmitData( i, (signed char)sigInfo.sigInfo.rssi, (char*)msg );
          BSP_TURN_ON_LED2();               // Toggle LED2 when received packet
          BSP_ENTER_CRITICAL_SECTION(intState);
          sPeerFrameSem--;
          BSP_EXIT_CRITICAL_SECTION(intState);
          __delay_cycles(10000);
          BSP_TURN_OFF_LED2();
        }
      }
    }
  }
}

/*******************************************************************************
*BEGHDR
*
*NAME:createRandomAddress()
*
*DESCRIPTION: generate random address
*
*******************************************************************************/
void createRandomAddress(void)
{
  unsigned int rand, rand2;
  char *Flash_Addr;                         // Initialize radio address location
  Flash_Addr = (char *)0x10F0;

  do
  {
    rand = TI_getRandomIntegerFromVLO();    // first byte can not be 0x00 of 0xFF
  }
  while( (rand & 0xFF00)==0xFF00 || (rand & 0xFF00)==0x0000 );
  rand2 = TI_getRandomIntegerFromVLO();

  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
  DCOCTL = CALDCO_1MHZ;
  FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator
  FCTL3 = FWKEY + LOCKA;                    // Clear LOCK & LOCKA bits
  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

  Flash_Addr[0]=(rand>>8) & 0xFF;
  Flash_Addr[1]=rand & 0xFF;
  Flash_Addr[2]=(rand2>>8) & 0xFF;
  Flash_Addr[3]=rand2 & 0xFF;

  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCKA + LOCK;             // Set LOCK & LOCKA bit
}

/*******************************************************************************
*
*******************************************************************************/
void transmitData(int addr, signed char rssi,  char msg[MESSAGE_LENGTH] )
{
  char addrString[4];
  char rssiString[3];
  volatile signed int rssi_int;

  addrString[0] = '0';
  addrString[1] = '0';
  addrString[2] = '0'+(((addr+1)/10)%10);
  addrString[3] = '0'+((addr+1)%10);
  rssi_int = (signed int) rssi;
  rssi_int = rssi_int+128;
  rssi_int = (rssi_int*100)/256;
  rssiString[0] = '0'+(rssi_int%10);
  rssiString[1] = '0'+((rssi_int/10)%10);
  rssiString[2] = '0'+((rssi_int/100)%10);

  transmitDataString( addrString, rssiString, msg );
}

/*******************************************************************************
*
*******************************************************************************/
void transmitDataString(char addr[4],char rssi[3], char msg[MESSAGE_LENGTH] )
{

	char output[] = {"ID=X.XX,Vcc=X.XXX,T=XXX.XX,RH=XX.XX,Acc_x=XXXXX.XX,Acc_y=XXXXX.XX,Acc_z=XXXXX.XX,P=XXXX.XX,Vpd=XXXX.X,status=XXXXXXXX.X\r\n"};
	//				  0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789
	//				  0         1         2         3         4         5         6         7         8         1         2         3         4

	char id_string[] = {"Y.YY"};		// add '0' so that python can read it (for some reason only reads floats, not ints)
	char vcc_string[] = {"Y.YYY"};
	char t_string[] = {"YYY.YY"};
	char rh_string[] = {"YY.YY"};
	char acc_x_string[] = {"YYYYY.00"};
	char acc_y_string[] = {"YYYYY.00"};
	char acc_z_string[] = {"YYYYY.00"};
	char p_string[] = {"YYYY.YY"};
	char status_string[] = {"bbbbbbbb.0"};
	char vpd_string[] = {"XXXX.0"};
	int i,j;

	i=0;

	int NODE_ID = msg[i++];
	int Vcc = (msg[i++]<<8) + msg[i++];
	int T =  (msg[i++]<<8) + msg[i++];
	int RH =  (msg[i++]<<8) + msg[i++];
	int A_x =  (msg[i++]<<8) + msg[i++];
	int A_y =  (msg[i++]<<8) + msg[i++];
	int A_z =  (msg[i++]<<8) + msg[i++];
	long P =  ((msg[i++]<<8) + msg[i++])&0xFFFF;
	P = (P<<8)+msg[i++];
	int Vpd = (msg[i++]<<8) + msg[i++];
	char msg_status = msg[i++];

	// ID
//	if (NODE_ID<100)
//	{
//		id_string[0] = ' ';
//	}
//	else
//	{
		id_string[0] = '0'+((NODE_ID/100)%10);
//	}
//	if (NODE_ID<10)
//	{
//		id_string[2] = ' ';
//	}
//	else
//	{
		id_string[2] = '0'+((NODE_ID/10)%10);
//	}
	id_string[3] = '0'+(NODE_ID%10);


	// Vcc
	vcc_string[0] = '0'+((Vcc/1000)%10);
	vcc_string[2] = '0'+((Vcc/100)%10);
	vcc_string[3] = '0'+((Vcc/10)%10);
	vcc_string[4] = '0'+(Vcc%10);

	// T
	if( T < 0 )
	{
		t_string[0] = '-';
		T = T * -1;
	}
	else
	{
		t_string[0] = '0'+((T/10000)%10);
	}
	t_string[1] = '0'+((T/1000)%10);
	t_string[2] = '0'+((T/100)%10);
	t_string[4] = '0'+((T/10)%10);
	t_string[5] = '0'+(T%10);

	// RH
	rh_string[0] = '0'+((RH/1000)%10);
	rh_string[1] = '0'+((RH/100)%10);
	rh_string[3] = '0'+((RH/10)%10);
	rh_string[4] = '0'+(RH%10);

	// A_x
	if( A_x < 0 )
	{
		A_x = A_x * -1;
		if (A_x <1000)
		{
			acc_x_string[0] = ' ';
			if (A_x < 100)
			{
				acc_x_string[1] = ' ';
				if (A_x <10)
				{
					acc_x_string[2] = ' ';
					acc_x_string[3] = '-';
					acc_x_string[4] = '0'+(A_x%10);
				}
				else	// if >= 10
				{
					acc_x_string[2] = '-';
					acc_x_string[3] = '0'+((A_x/10)%10);
					acc_x_string[4] = '0'+(A_x%10);
				}
			}
			else	// if >= 100
			{
				acc_x_string[1] = '-';
				acc_x_string[2] = '0'+((A_x/100)%10);
				acc_x_string[3] = '0'+((A_x/10)%10);
				acc_x_string[4] = '0'+(A_x%10);
			}
		}
		else	// if >= 1000
		{
			acc_x_string[0] = '-';
			acc_x_string[1] = '0'+((A_x/1000)%10);
			acc_x_string[2] = '0'+((A_x/100)%10);
			acc_x_string[3] = '0'+((A_x/10)%10);
			acc_x_string[4] = '0'+(A_x%10);
		}
	}
	else	// if >= 000
	{
		acc_x_string[0] = ' ';
		acc_x_string[1] = '0'+((A_x/1000)%10);
		acc_x_string[2] = '0'+((A_x/100)%10);
		acc_x_string[3] = '0'+((A_x/10)%10);
		acc_x_string[4] = '0'+(A_x%10);
	}


	// A_y
	if( A_y < 0 )
	{
		A_y = A_y * -1;
		if (A_y <1000)
		{
			acc_y_string[0] = ' ';
			if (A_y < 100)
			{
				acc_y_string[1] = ' ';
				if (A_y <10)
				{
					acc_y_string[2] = ' ';
					acc_y_string[3] = '-';
					acc_y_string[4] = '0'+(A_y%10);
				}
				else	// if >= 10
				{
					acc_y_string[2] = '-';
					acc_y_string[3] = '0'+((A_y/10)%10);
					acc_y_string[4] = '0'+(A_y%10);
				}
			}
			else	// if >= 100
			{
				acc_y_string[1] = '-';
				acc_y_string[2] = '0'+((A_y/100)%10);
				acc_y_string[3] = '0'+((A_y/10)%10);
				acc_y_string[4] = '0'+(A_y%10);
			}
		}
		else	// if >= 1000
		{
			acc_y_string[0] = '-';
			acc_y_string[1] = '0'+((A_y/1000)%10);
			acc_y_string[2] = '0'+((A_y/100)%10);
			acc_y_string[3] = '0'+((A_y/10)%10);
			acc_y_string[4] = '0'+(A_y%10);
		}
	}
	else	// if >= 000
	{
		acc_y_string[0] = ' ';
		acc_y_string[1] = '0'+((A_y/1000)%10);
		acc_y_string[2] = '0'+((A_y/100)%10);
		acc_y_string[3] = '0'+((A_y/10)%10);
		acc_y_string[4] = '0'+(A_y%10);
	}

	// A_z
	if( A_z < 0 )
	{
		A_z = A_z * -1;
		if (A_z <1000)
		{
			acc_z_string[0] = ' ';
			if (A_z < 100)
			{
				acc_z_string[1] = ' ';
				if (A_z <10)
				{
					acc_z_string[2] = ' ';
					acc_z_string[3] = '-';
					acc_z_string[4] = '0'+(A_z%10);
				}
				else	// if >= 10
				{
					acc_z_string[2] = '-';
					acc_z_string[3] = '0'+((A_z/10)%10);
					acc_z_string[4] = '0'+(A_z%10);
				}
			}
			else	// if >= 100
			{
				acc_z_string[1] = '-';
				acc_z_string[2] = '0'+((A_z/100)%10);
				acc_z_string[3] = '0'+((A_z/10)%10);
				acc_z_string[4] = '0'+(A_z%10);
			}
		}
		else	// if >= 1000
		{
			acc_z_string[0] = '-';
			acc_z_string[1] = '0'+((A_z/1000)%10);
			acc_z_string[2] = '0'+((A_z/100)%10);
			acc_z_string[3] = '0'+((A_z/10)%10);
			acc_z_string[4] = '0'+(A_z%10);
		}
	}
	else	// if >= 000
	{
		acc_z_string[0] = ' ';
		acc_z_string[1] = '0'+((A_z/1000)%10);
		acc_z_string[2] = '0'+((A_z/100)%10);
		acc_z_string[3] = '0'+((A_z/10)%10);
		acc_z_string[4] = '0'+(A_z%10);
	}

	// P
	p_string[0] = '0'+((P/100000)%10);
	p_string[1] = '0'+((P/10000)%10);
	p_string[2] = '0'+((P/1000)%10);
	p_string[3] = '0'+((P/100)%10);
	p_string[5] = '0'+((P/10)%10);
	p_string[6] = '0'+(P%10);

	// Vpd
	vpd_string[0] = '0'+((Vpd/1000)%10);
	vpd_string[1] = '0'+((Vpd/100)%10);
	vpd_string[2] = '0'+((Vpd/10)%10);
	vpd_string[3] = '0'+(Vpd%10);

	// Status
	for (i=8; i>0; i--)
	{
		status_string[i-1] = '0' + msg_status%2;
		msg_status = msg_status/2;
	}

// CREATE OUTPUT
	j=0;
	// NODE_ID
	j += 3;
	for (i = 0; i<sizeof(id_string)-1; i++)
	{
		output[i+j] = id_string[i];
	}
	j += i;
	j += 1;
	// Vcc=
	j += 4;
	for (i = 0; i<sizeof(vcc_string)-1; i++)
	{
		output[i+j] = vcc_string[i];
	}
	j += i;
	j += 1;

	// T=
	j += 2;
	for (i = 0; i<sizeof(t_string)-1; i++)
	{
		output[i+j] = t_string[i];
	}
	j += i;
	j += 1;

	// RH=
	j += 3;
	for (i = 0; i<sizeof(rh_string)-1; i++)
	{
		output[i+j] = rh_string[i];
	}
	j += i;
	j += 1;

	// Acc_x=
	j += 6;
	for (i = 0; i<sizeof(acc_x_string)-1; i++)
	{
		output[i+j] = acc_x_string[i];
	}
	j += i;
	j += 1;

	// Acc_y=
	j += 6;
	for (i = 0; i<sizeof(acc_y_string)-1; i++)
	{
		output[i+j] = acc_y_string[i];
	}
	j += i;
	j += 1;

	// Acc_z=
	j += 6;
	for (i = 0; i<sizeof(acc_z_string)-1; i++)
	{
		output[i+j] = acc_z_string[i];
	}
	j += i;
	j += 1;

	// P=
	j += 2;
	for (i = 0; i<sizeof(p_string)-1; i++)
	{
		output[i+j] = p_string[i];
	}
	j += i;
	j += 1;

	// Vpd
	j += 4;
	for (i = 0; i<sizeof(vpd_string)-1; i++)
	{
		output[i+j] = vpd_string[i];
	}
	j += i;
	j += 1;

	// Status
	j += 7;
	for (i = 0; i<sizeof(status_string)-1; i++)
	{
		output[i+j] = status_string[i];
	}
	j += i;
	j += 1;

	TXString(output, sizeof output);
//	TXString(msg, sizeof msg);

}

/*******************************************************************************
*
*******************************************************************************/
void TXString( char* string, int length )
{
  int pointer;
  for( pointer = 0; pointer < length; pointer++)
  {
    //volatile int i;
    UCA0TXBUF = string[pointer];
    while (!(IFG2&UCA0TXIFG));              // USCI_A0 TX buffer ready?
  }
}

/*******************************************************************************
*
*******************************************************************************/
void MCU_Init()
{
  BCSCTL1 = CALBC1_8MHZ;                    // Set DCO
  DCOCTL = CALDCO_8MHZ;

  BCSCTL3 |= LFXT1S_2;                      // LFXT1 = VLO
  TBCCTL0 = CCIE;                           // TCCR0 interrupt enabled
  TBCCR0 = 12000;                           // ~1 second
  TBCTL = TBSSEL_1 + MC_1;                  // ACLK, upmode

  P3SEL |= 0x30;                            // P3.4,5 = USCI_A0 TXD/RXD
  UCA0CTL1 = UCSSEL_2;                      // SMCLK
  UCA0BR0 = 0x41;                           // 9600 from 8Mhz
  UCA0BR1 = 0x3;
  UCA0MCTL = UCBRS_2;
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
  __enable_interrupt();
}

/*******************************************************************************
* Runs in ISR context. Reading the frame should be done in the
* application thread not in the ISR thread.
*******************************************************************************/
static uint8_t sCB(linkID_t lid)
{
  if (lid)
  {
    sPeerFrameSem++;
  }
  else
  {
    sJoinSem++;
  }
  // leave frame to be read by application.
  return 0;
}

/*******************************************************************************
* ADC10 interrupt service routine
*******************************************************************************/
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(LPM0_bits);     // Clear CPUOFF bit from 0(SR)
}

/*******************************************************************************
* Timer B0 interrupt service routine
*******************************************************************************/
#pragma vector=TIMERB0_VECTOR
__interrupt void Timer_B (void)
{
  sSelfMeasureSem = 1;
}

/*******************************************************************************
* USCIA interrupt service routine
*******************************************************************************/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  char rx = UCA0RXBUF;
  if ( rx == 'V' || rx == 'v' )
  {
    verboseMode = 1;
  }
  else if ( rx == 'M' || rx == 'm' )
  {
    verboseMode = 0;
  }
  else if ( rx == 'F' || rx == 'f' )
  {
    degCMode = 0;
  }
  else if ( rx == 'C' || rx == 'c' )
  {
    degCMode = 1;
  }
}
