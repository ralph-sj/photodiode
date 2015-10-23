//******************************************************************************
// THIS PROGRAM IS PROVIDED "AS IS". TI MAKES NO WARRANTIES OR
// REPRESENTATIONS, EITHER EXPRESS, IMPLIED OR STATUTORY,
// INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR
// COMPLETENESS OF RESPONSES, RESULTS AND LACK OF NEGLIGENCE.
// TI DISCLAIMS ANY WARRANTY OF TITLE, QUIET ENJOYMENT, QUIET
// POSSESSION, AND NON-INFRINGEMENT OF ANY THIRD PARTY
// INTELLECTUAL PROPERTY RIGHTS WITH REGARD TO THE PROGRAM OR
// YOUR USE OF THE PROGRAM.
//
// IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, INCIDENTAL,
// CONSEQUENTIAL OR INDIRECT DAMAGES, HOWEVER CAUSED, ON ANY
// THEORY OF LIABILITY AND WHETHER OR NOT TI HAS BEEN ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGES, ARISING IN ANY WAY OUT
// OF THIS AGREEMENT, THE PROGRAM, OR YOUR USE OF THE PROGRAM.
// EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF
// REMOVAL OR REINSTALLATION, COMPUTER TIME, LABOR COSTS, LOSS
// OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, OR LOSS OF
// USE OR INTERRUPTION OF BUSINESS. IN NO EVENT WILL TI'S
// AGGREGATE LIABILITY UNDER THIS AGREEMENT OR ARISING OUT OF
// YOUR USE OF THE PROGRAM EXCEED FIVE HUNDRED DOLLARS
// (U.S.$500).k 
//
// Unless otherwise stated, the Program written and copyrighted
// by Texas Instruments is distributed as "freeware".  You may,
// only under TI's copyright in the Program, use and modify the
// Program without any charge or restriction.  You may
// distribute to third parties, provided that you transfer a
// copy of this license to the third party and the third party
// agrees to these terms by its first use of the Program. You
// must reproduce the copyright notice and any other legend of
// ownership on each copy or partial copy, of the Program.
//
// You acknowledge and agree that the Program contains
// copyrighted material, trade secrets and other TI proprietary
// information and is protected by copyright laws,
// international copyright treaties, and trade secret laws, as
// well as other intellectual property laws.  To protect TI's
// rights in the Program, you agree not to decompile, reverse
// engineer, disassemble or otherwise translate any object code
// versions of the Program to a human-readable form.  You agree
// that in no event will you alter, remove or destroy any
// copyright notice included in the Program.  TI reserves all
// rights not specifically granted under this license. Except
// as specifically provided herein, nothing in this agreement
// shall be construed as conferring by implication, estoppel,
// or otherwise, upon you, any license or other right under any
// TI patents, copyrights or trade secrets.
//
// You may not use the Program in non-TI devices.
//
//******************************************************************************
//  eZ430-RF2500 Temperature Sensor End Device using Cymbet Solar Energy
//  Harvesting Board
//
//  Description:
//      This is the End Device software for the eZ430-RF2500-SEH Temperature
//      Sensing demo when hooked up to a Cymbet solar Energy Harvester board.
//
//      The Energy Harvester End Device (EHED) will join the traditional
//      Access Point (AP).  The EHED was optimized to reduce active time
//      especially during start up.
//
//   W. Goh
//   Version 1.5
//   Texas Instruments, Inc
//   March 2009
//   Built with IAR Embedded Workbench Version: 4.11B
//   Built with Code Composer Essentials Version: 3.1 build 3.2.3.6.4
//******************************************************************************
// Change Log:
//******************************************************************************
// Version:  1.5 using SimpliciTI ver 1.06
// Comments: Fixed an un-initialized bug inside SimpliciTI
//           Removed unnecessary port initialization in code
// Version:  1.4 using simpliciTI ver 1.06
// Comments: Fixed several bugs.
//           Added blinking LED on power-up
//           Application files now compiles on both IAR and CCE
// Version:  1.3 using Simpliciti ver 1.06
// Comments: Added count battery used count fields
//           Added number_transmits counts up and down
//           Added check if battery charged for 1 hour
// Version:  1.0
// Comments: Inital Release date
//******************************************************************************

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

#define WakeupPeriod      15000             // ~10 sec (=15000/(12000/8))
#define a_d_wakeup_time   4500              // ~3 sec
#define TXPeriod          7500              // ~5 sec  (=7500/(12000/8))
#define delay_time        500               // led delay time
#define debounce_time     750               // key debounce

//Timer count for time between transmit
#define sec1              1500              // ~1 sec
#define sec2              2610
#define sec5              7500              // ~5 sec  (=7500/(12000/8))
#define sec10             15000             // ~10 sec
#define sec20             30000             // ~20 swec
#define sec40             60000             // ~40 sec
#define sec30_2           43000             // ~30sec 2 min?
#define sec30_4           50434             // ~30sec 4 min?
#define one_hour          5400000

#define port_delay        10                // 6ms - 1.5 msec

#define status_one        1
#define status_two        2
#define status_three      3
#define status_four       4
#define status_five       5
#define status_six        6

#define timer_state_1     1
#define timer_state_2     2
#define timer_state_3     3
#define timer_state_4     4
#define timer_state_5     5
#define timer_state_6     6

#define run_voltage       29                // Minimum voltage to execute 2.9V
#define ad_check_voltage  29
#define key_down_count    12                // # times to check if button is
                                            // still button pressed

#define battery_time_test 174               // 3 min count at 10 sec for testing
#define running_on_battery 100              // Tells GUI that it is running on
                                            // battery
#define xmt_count         400               // # max transmit on battery - 400
                                            // magic number

#define	ON                1
#define OFF               0

unsigned int timer_state;
unsigned char change_mode;
unsigned char ftt_flag;
unsigned int battery_ready = 0;
unsigned int in_delay = 0;
char status = 0;
unsigned int battery_full_flag = 0;
unsigned long battery_full_timer = 0;

unsigned int number_transmits;

void linkTo(void);
void StatusBlink_led1(int BlinkCount);
void StatusBlink_led2(int BlinkCount);
void status_indicator(char status, int status_led);
void delay(unsigned int BlinkCount);
void button_still_pressed(void);
unsigned int get_voltage(void);
void transmit_time_delay(void);
void display_mode(void);
void check_bat_full(void);
void createRandomAddress(void);

void main (void)
{
  addr_t lAddr;
  char *Flash_Addr;
  unsigned int current_voltage;

  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  
  P1DIR |= 0x03;                            // Set P1.0,1 Output
  if( CALBC1_1MHZ == 0xFF && CALDCO_1MHZ == 0xFF &&
      CALBC1_8MHZ == 0xFF && CALDCO_8MHZ == 0xFF )// Do not run if cal values
  {                                         // are erased and set LEDs ON
    P1OUT |= 0x03;                          // Set P1.0,1 High
    __bis_SR_register(LPM4_bits);           // Enter LPM4 if Cal missing
  }

  // Blink LED for startup feedback
  P1OUT |= 0x03;                            // Set P1.0,1 High
  __delay_cycles(10000);
  P1OUT &= ~0x03;                           // Set P1.0,1 Low
  
  Flash_Addr = (char *)0x10F0;              // RF Address = 0x10F0
  if( Flash_Addr[0] == 0xFF &&              // Check if device Address is missing
      Flash_Addr[1] == 0xFF &&
      Flash_Addr[2] == 0xFF &&
      Flash_Addr[3] == 0xFF )
  {
    createRandomAddress();                  // Create Random device address at
  }                                         // initial startup if missing
  lAddr.addr[0] = Flash_Addr[0];
  lAddr.addr[1] = Flash_Addr[1];
  lAddr.addr[2] = Flash_Addr[2];
  lAddr.addr[3] = Flash_Addr[3];
  SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);

  BSP_Init();                               // Initialize eZ430 hardware

  BCSCTL3 |= LFXT1S_2;                      // LFXT1 = ACLK = VLO
  TBCCTL0 = CCIE;                           // TBCCR0 interrupt enabled
  TBCCR0 =  WakeupPeriod;                   // ~10 sec (=15000/(12000/8))
  TBCTL = TBSSEL_1 + MC_1 + ID_3;           // ACLK, upmode, Divider = 8

  status = status_four;                     // Set status to 4

  // Initialize SimpliciTI
  while(SMPL_NO_JOIN == SMPL_Init((uint8_t (*)(linkID_t))0))
  {
    __bis_SR_register(LPM3_bits + GIE);     // LPM3 with interrupts enabled
  }

  // Put radio to sleep
  SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, "" );

  ftt_flag = 1;                             // first time thru the program flag

  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO = 1MHz
  DCOCTL = CALDCO_1MHZ;

  // SimpliciTI will change port pin settings as well
  P1DIR = 0xFB;                             // P1.2 (button) = input
  P1OUT = 0x04;                             // P1.2 pullup
  P1REN |= 0x04;                            // P1.2 pullup
  P1IE |= 0x04;                             // P1.3 interrupt enabled
  P1IES |= 0x04;                            // P1.3 Hi/lo edge
  P1IFG &= ~0x04;                           // P1.3 IFG cleared
  P2DIR = 0x2E;
  P2REN |= 0x01;
  P2OUT = 0x01;
  P3DIR |= 0xD0;                            // port 3 set after initilization
  P3OUT &= ~0x30;                           // set up port 3
  P3REN |= 0x20;                            // Enable Pull-Down Res for /Charge
  P4DIR = 0xFF;                             // setup port 4
  P4OUT = 0x00;

  timer_state = timer_state_2;              // set timer state to 2 ~ 10 sec
  change_mode = 10;                         // Default GUI display mode set to
                                            // 10 sec

  TBCTL |= TBCLR;                           // Clear TBR counter
  TBCCR0 = a_d_wakeup_time;                 // set timer to wakeup time ~ 3 sec
  // added to check if battery voltage stable before linking
//  current_voltage = get_voltage();          // get current battery voltage
//  if(current_voltage < run_voltage)
//  {
//    current_voltage = 0;
//    while (current_voltage < ad_check_voltage)
//    {
//      __bis_SR_register(LPM3_bits + GIE);   // Enter LPM3 w/ interrupts
//      current_voltage = get_voltage();
//    }
//  }

  SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_AWAKE, "" );
  // unconditional link to AP which is listening due to successful join.
  linkTo();
}

/*******************************************************************************
 * @fn          linkTo
 ******************************************************************************/
void linkTo(void)
{
  linkID_t linkID1;
  uint8_t  msg[7];
  unsigned int *tempOffset;                 // Initialize temperature offset
  tempOffset = (unsigned int *)0x10F4;      // coefficient
  number_transmits = xmt_count;             // Initialize to max transmit #

  // keep trying to link... Uses Timer B to wake up periodically
  while (SMPL_SUCCESS != SMPL_Link(&linkID1))
  {
    __bis_SR_register(LPM3_bits + GIE);     // LPM3 with interrupts enabled
  }
    
  // put radio to sleep once a successfull connection has been established
  SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, "" );

  while(1)
  {
    volatile long temp;
    int degC, volt;
    int results[2];

    // If battery charging, go back to sleep  (if P3.5 = 1, Sleep)
    P3REN &= ~0x20;                         // turn off pulldown resistor
    delay(port_delay);

    // Measure Temperature
    ADC10CTL1 = INCH_10 + ADC10DIV_4;       // Temp Sensor ADC10CLK/5
    ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE + ADC10SR;
    __delay_cycles(350);                    // delay to allow reference to settle
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
    __bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
    results[0] = ADC10MEM;
    ADC10CTL0 &= ~ENC;

    // Measure Battery Voltage
    ADC10CTL1 = INCH_11;                    // AVcc/2
    ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE + REF2_5V;
    __delay_cycles(350);                    // delay to allow reference to settle
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
    __bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
    results[1] = ADC10MEM;
    ADC10CTL0 &= ~ENC;
    ADC10CTL0 &= ~(REFON + ADC10ON);        // turn off A/D to save power

    // oC = ((A10/1024)*1500mV)-986mV)*1/3.55mV = A10*423/1024 - 278
    // the temperature is transmitted as an integer where 32.1 = 321
    // hence 4230 instead of 423
    temp = results[0];
    degC = ((temp - 673) * 4230) / 1024;
    if( *tempOffset != 0xFFFF )
    {
      degC += *tempOffset;
    }

    /* message format,  UB = upper Byte, LB = lower Byte
    ----------------------------------------------------------------------------
    |degC LB | degC UB |  volt LB | Mode  | # transmit LB |# transmit UB |  ?  |
    ----------------------------------------------------------------------------
       0         1          2          3          4               5         6
    */

    // Wake radio-up
    SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_AWAKE, "" );

    temp = results[1];
    volt = (temp*25)/512;
    msg[0] = degC&0xFF;
    msg[1] = (degC>>8)&0xFF;
    msg[2] = volt;

    // If using Solar & not first time through, set battery as ready
    if(((P3IN & 0x20) == 0x00) && (ftt_flag == 0) && (battery_ready == 0))
    {
      battery_ready = 1;
    }

    // If battery is ready or first time through, transmit packets
    if(battery_ready == 1 || ftt_flag == 1)
    {
      if((P3IN & 0x20))                     // If P3.5 = 1, then
      {                                     // running on battery
        msg[3] = (change_mode + running_on_battery); // +100 is for GUI to know
                                            // is on battery
        if( number_transmits != 0)
          number_transmits--;               // # transmit countdown
      }
      else
      {                                     // else using solar cells
        msg[3] = change_mode;
        if(number_transmits != xmt_count)
        {	
          number_transmits++ ;
          if(number_transmits >= xmt_count) // If max # of transmits achieved,
          {                                 // Reset counter to 400.
            number_transmits = xmt_count;
          }
        }
        if(battery_full_flag == 1)          // If battery is fully charged,
        {                                   // reset counter to 400
          number_transmits = xmt_count;
        }
      }
      msg[4] = number_transmits&0xFF;
      msg[5] = (number_transmits>>8)&0xFF;

      // used as a spare bit to indicate on and off
      if(P3IN & 0x01)
      { 	 		
        msg[6] = ON;
      }
      else
      {
        msg[6] = OFF;
      }

      // if end of battery, turn off battery
      if(number_transmits == 0)
      {
        in_delay = 1;
        while((P3IN & 0x20))                // Continue sleeping if still on
        {                                   // battery
          __bis_SR_register(LPM4_bits);
        }
        in_delay = 0;
      }

      // Send message
      if (SMPL_SUCCESS == SMPL_Send(linkID1, msg, sizeof(msg)))
      {
        delay(port_delay);

        if(P3IN & 0x20)                     // Using Battery, Blink Red
        {
          status_indicator(status_one, 1);
        }
        else                                // Using Solar & Blink Green
        {
          status_indicator(status_one, 2);
        }
      }
      else                                  // Blink both LED if transmission
      {                                     // failed
        status_indicator(status_one, 1);
        status_indicator(status_one, 2);
      }
    }

    ftt_flag = 0;                           // first time thru the program flag
    check_bat_full();
    status = status_six;
    P3REN |= 0x20;                          // Set /Charge pulldown resistor
    SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, "" );
    transmit_time_delay();                  // sleep time between transmits
  }
}

/*******************************************************************************
* BEGHDR
*
* NAME:createRandomAddress()
*
* DESCRIPTION: generate random address
*******************************************************************************/
void createRandomAddress()
{
  unsigned int rand, rand2;
  char *Flash_Addr;
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
* BEGHDR
* Function:    void StatusBlink_led1(int BlinkCount)
* DESCRIPTION: Blinks LED 1 - Red based on specified delay
* INPUTS:      BlinkCount
* PROCESSING:  Turns on and off the RED LED with specified blink time
* OUTPUTS:     VOID
*******************************************************************************/
void StatusBlink_led1(int BlinkCount)
{
  BSP_TURN_ON_LED1();
  delay(BlinkCount);
  BSP_TURN_OFF_LED1();
}

/*******************************************************************************
* BEGHDR
* Function:    void StatusBlink_led1(int BlinkCount)
* DESCRIPTION: Blinks LED 1 - Green based on specified delay
* INPUTS:      BlinkCount
* PROCESSING:  Turns on and off the Green LED with specified blink time
* OUTPUTS:     VOID
*******************************************************************************/
void StatusBlink_led2(int BlinkCount)
{
  BSP_TURN_ON_LED2();
  delay(BlinkCount);
  BSP_TURN_OFF_LED2();
}

/*******************************************************************************
* BEGHDR
* Function:    void delay(unsigned int BlinkCount)
* DESCRIPTION: Creates a low-power delay by entering LPM3 using Timer B.
*              Timer B frequency = VLO/8 = 1500 Hz.
* INPUTS:      BlinkCount
* PROCESSING:  Delay length of time of BlinkCount
* OUTPUTS:     VOID
********************************************************************************/
void delay(unsigned int BlinkCount)
{
  int TimerTemp;
  TimerTemp = TBCCR0;                       // Save current content of TBCCR0
  TBCCR0 = BlinkCount;                      // Set new TBCCR0 delay
  TBCTL |= TBCLR;                           // Clear TBR counter
  TBCCTL0 &= ~CCIFG;                        // Clear CCIFG Flag
  TBCTL |= MC_1;                            // Start Timer B
  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3
  TBCTL &= ~(MC_1);                         // Stop Timer B
  TBCCR0 = TimerTemp;
}

/*******************************************************************************
* BEGHDR
* Function:    void status_indicator(char status , int status_led)
* DESCRIPTION: This can be usefull to blink the LED to indicate where the
*              program is executing for debugging purposes. It blinks the red or
*              green LED the number of times in status. For example, status_five
*              blinks the LED 5 times.
* INPUTS:      status, status_led
* PROCESSING:  Blinks the red or green led the number of times in status and the
*              correct led in status_led
* OUTPUTS:     VOID
*******************************************************************************/
void status_indicator(char status , int status_led)
{
  volatile unsigned int i = 0;
  switch (status)
  {
    case status_one:
      if (status_led == 1)
        StatusBlink_led1(15);
      if (status_led == 2)
        StatusBlink_led2(15);
      break;

    case status_two:
      if (status_led == 1)
      {
        StatusBlink_led1(15);
        delay(delay_time);
        StatusBlink_led1(15);
      }
      if (status_led==2)
      {
        StatusBlink_led2(15);
        delay(delay_time);
        StatusBlink_led2(15);
      }
      break;

    case status_three:
      if(status_led == 1)
      {
        for(i=0 ; i < (status-1) ; i++)
        {
          StatusBlink_led1(15);
          delay(delay_time);
        }
        StatusBlink_led1(15);
      }
      if(status_led == 2)
      {
        for(i=0 ; i < (status-1) ; i++)
        {
          StatusBlink_led2(15);
          delay(delay_time);
        }
        StatusBlink_led2(15);
      }
      break;

    case status_four:
      if(status_led == 1)
      {
        for(i=0 ; i < (status-1) ; i++)
        {
          StatusBlink_led1(15);
          delay(delay_time);
        }
        StatusBlink_led1(15);
      }
      if(status_led == 2)
      {
        for(i=0 ; i < (status-1) ; i++)
        {
          StatusBlink_led2(15);
          delay(delay_time);
        }
        StatusBlink_led2(15);
      }
      break;

    case status_five:
      if(status_led == 1)
      {
        for(i=0 ; i < (status-1) ; i++)
        {
          StatusBlink_led1(15);
          delay(delay_time);
        }
        StatusBlink_led1(15);
      }
      if(status_led == 2)
      {
        for(i=0 ; i < (status-1) ; i++)
        {
          StatusBlink_led2(15);
          delay(delay_time);
        }
        StatusBlink_led2(15);
      }
      break;
    default:
      break;
  }
}

/*******************************************************************************
* BEGHDR
* Function:    unsigned int get_voltage(void)
* DESCRIPTION: Get battery voltage with A/D
* INPUTS:      void
* PROCESSING:  Read battery voltage from ADC10 and returns the value
* OUTPUTS:     Battery voltage from A/D
*******************************************************************************/
unsigned int get_voltage(void)
{
  unsigned int rt_volts;

  ADC10CTL1 = INCH_11;                    // AVcc/2
  ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE + REF2_5V;
  __delay_cycles(250);                    // delay to allow reference to settle
  ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
  __bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
  rt_volts = ADC10MEM;
  ADC10CTL0 &= ~ENC;
  ADC10CTL0 &= ~(REFON + ADC10ON);        // turn off A/D to save power
  rt_volts = (rt_volts*25)/512;
  return (rt_volts);
}

/*******************************************************************************
* BEGHDR
* NAME:        void transmit_time_delay(void)
* DESCRIPTION: Sets timer to transmit time based on timer_state
* INPUTS:      void
* PROCESSING:  Sets timer to transmit time, for 2 min and 4 min transmit times
*              loop number of 30 sec times to make 2 min and 4 min.
* OUTPUTS:     void
********************************************************************************/
void transmit_time_delay(void)
{
  volatile unsigned int i = 0;
  in_delay = 1;
  switch (timer_state)
  {
    case timer_state_1:                     // Timer State == 1; 5 Secs
      delay(sec5);
      in_delay = 0;
      battery_full_timer += sec5;
      break;
    case timer_state_2:                     // Timer State == 2; 10 Secs
      delay(sec10);
      in_delay = 0;
      battery_full_timer += sec10;
      break;

    case timer_state_3:                     // Timer State == 3; 20 Secs
      delay(sec20);
      in_delay = 0;
      battery_full_timer += sec20;
      break;

    case timer_state_4:                     // Timer State == 4; 40 Secs
      delay(sec40);
      in_delay = 0;
      battery_full_timer += sec40;
      break;

    case timer_state_5:                     // Timer State == 5; 2 mins
      while((i++ < 4) && (timer_state == timer_state_5))
      {
        delay(sec30_2);
        in_delay = 0;
        battery_full_timer += sec30_2;
      }
      break;

    case timer_state_6:                     // Timer State == 6; 4 mins
      while((i++ < 8) && (timer_state == timer_state_6))
      {
        delay(sec30_4);
        in_delay = 0;
        battery_full_timer += sec30_4;
      }
      break;
    default:
      break;
  }
}// void transmit_time_delay(void)

/*******************************************************************************
* BEGHDR
* NAME:        void display_mode(void)
* DESCRIPTION: Sets mode time to be displayed on the GUI in the voltage stage
*              for the first display
* INPUTS:      void
* PROCESSING:  Sets change_mode number based on timer_state
* OUTPUTS:     void
*******************************************************************************/
void display_mode(void)
{
  switch(timer_state)
  {
    case timer_state_1:
      change_mode=5;                        //~=5 sec
      break;
    case timer_state_2:
      change_mode=10;                       //~=10 sec
      break;
    case timer_state_3:
      change_mode=20;                       //~=20 sec
      break;
    case timer_state_4:
      change_mode=40;                       //~=40 sec
      break;
    case timer_state_5:
      change_mode=2;                        //~=2 min
      break;
    case timer_state_6:
      change_mode=4;                        //~=4 min
      break;
    default:
      break;
  }
}

/*******************************************************************************
* BEGHDR
* NAME:        void check_batt_full(void)
* DESCRIPTION: Check if the battery has been charging for 1 hour using
*              battery_full_timer as the counter. battery_full_timer is updated
*              after each delay inside transmit_time_delay() function.
* INPUTS:      void
* PROCESSING:  If battery_full_timer has arrived, switch off batteries.
*              battery_full_flag set if charging time is matched.
* OUTPUTS:     void
*******************************************************************************/
void check_bat_full(void)
{
  // If /Charge is high, No Solar, turn on battery
  if(P3IN & 0x20)                           // /Charge=1; battery, Blink Red
  {
    P3OUT &= ~0x10;                         // turn on battery
    battery_full_flag = 0;
    battery_full_timer = 0;
  }
  else                                      // /Charge=0; Solar charging,
  {                                         // blink green
    if(battery_full_timer >= one_hour)      // If battery has been charging for
    {                                       // an hour, turn off battery
      battery_full_flag = 1;
      battery_full_timer = 0;
      P3OUT |= 0x10;                        // turn off battery
    }
  }
}

/***********************************************************************
*BEGHDR
*NAME:        __interrupt void ADC10_ISR(void)
*DESCRIPTION: ADC10 interrupt service routine
*INPUTS:      void
*PROCESSING:  Exit from LPM after interrupt
*OUTPUTS:     void
***********************************************************************/
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(LPM0_bits);     // Clear CPUOFF bit from 0(SR)
}

/*******************************************************************************
* BEGHDR
* NAME:        __interrupt void Timer_B (void)
* DESCRIPTION: Timer B0 interrupt service routine
* INPUTS:      Void
* PROCESSING:  Exit from LPM after interrupt
* OUTPUTS:     Void
*******************************************************************************/
#pragma vector=TIMERB0_VECTOR
__interrupt void TimerB_ISR (void)
{
  __bic_SR_register_on_exit(LPM3_bits);     // Clear LPM3 bit from 0(SR)
}

/*******************************************************************************
* BEGHDR
* NAME:        __interrupt void Port_1(void)
* DESCRIPTION: Port 1 interrupt service routine function key
* INPUTS:      void
* PROCESSING:  process the push button to switch to the next time mode
* OUTPUTS:     void
********************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  if((P3IN & 0x20))                         // /Charge=1; battery, Blink Red
  {
    BSP_TURN_ON_LED1();
    __delay_cycles(10000);
    BSP_TURN_OFF_LED1();
  }
  else                                      // /Charge=0; Solar, blink green
  {
    BSP_TURN_ON_LED2();
    __delay_cycles(10000);
    BSP_TURN_OFF_LED2();
  }

  // If successful link, change timer state.
  if(status == status_six || status == status_five)
  {
    if(timer_state >= timer_state_6)        // If transmit time is == 6,
    {                                       // Set timer_state = 1
      timer_state = timer_state_1;
      display_mode();                       // Change GUI display time
    }
    else
    {
      timer_state++;                        // Change transmit time state
      display_mode();                       // Change GUI display time
    }
    if(in_delay)                            // If in transmit delay, exit and
    {                                       // send a new packet with new time
      __bic_SR_register_on_exit(LPM4_bits); // Clear LPM3 bit from 0(SR)
    }
  }
  __delay_cycles(150000);                   // Debounce software delay
  while(!(P1IN & 0x04));                    // Loop if button is still pressed
  P1IFG &= ~0x04;                           // P1.2 IFG cleared key interuped
}
