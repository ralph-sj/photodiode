// ED_all_sensors_transmit_data.c
// READ ALL SENSORS AND TX DATA
// 18/09/2015
// use with corresponding AP
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

#define TEST	0	// set to non-zero for testing
#define WakeupPeriod      15000             // ~10 sec (=15000/(12000/8))
#define a_d_wakeup_time   15000              // ~30 sec
#define TXPeriod          7500              // ~5 sec  (=7500/(12000/8))
#define delay_time        500               // led delay time
#define debounce_time     750               // key debounce

//Timer count for time between transmit
#define sec1              1500              // ~1 sec
#define sec2              2610
#define sec3              4500
#define sec5              7500              // ~5 sec  (=7500/(12000/8))
#define sec10             15000             // ~10 sec
#define sec20             30000             // ~20 swec
#define	sec30			  45000				// ~ 30 seconds
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
#define status_seven      7

#define timer_state_1     1
#define timer_state_2     2
#define timer_state_3     3
#define timer_state_4     4
#define timer_state_5     5
#define timer_state_6     6
#define timer_state_7     7

#define run_voltage       3000                // Minimum voltage to execute startup
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

#define MESSAGE_LENGTH		19	// must be less than or equal to MAX_APP_PAYLOAD	//5
#define NODE_ID	0x02				// NODE ID - MUST BE SEPERATE FOR EVERY NODE USED

#define	YES	1
#define	NO	0

unsigned int Vcc;
unsigned int i;
// msg_status
unsigned char msg_status = 0;
#define START_FLAG		BIT0
#define RESTART_FLAG	BIT1
#define T_FLAG			BIT2
#define RH_FLAG			BIT3
#define P_FLAG			BIT4
#define ACC_FLAG		BIT5
#define VCC_FLAG		BIT6
#define STOP_FLAG		BIT7

// sensor trigger limits
// Vcc - units of mV
#define	VCC_DIFF_MAX		10		// trigger tx if difference in voltage eceeds this
#define VCC_SLEEP_VOLTAGE	3300	// send to sleep if voltage goes below this
#define VCC_REAWAKE_VOLTAGE	3450	// to re-awaken after sleep mode
#define VCC_RUN_VOLTAGE		3300	// for first time setup
#define BAT_MIN_VOTAGE		2500	// minimum voltage for AA battery pack

// T - units of 0.01 C
#define T_DIFF_MAX		15
#define T_UPPER			3000
#define T_LOWER			2000

// RH - units of 0.01%
#define RH_DIFF_MAX		500
#define RH_UPPER 		7000
#define RH_LOWER 		3000

// P - unit of 0.01 mbar
#define P_DIFF_MAX		25
// Acc - units of mg
#define ACC_DIFF_MAX	50

unsigned int Vcc_last;
signed int Temperature_last;
unsigned int Humidity_last;
unsigned long Pressure_last;
signed int Acceleration_last[3];

signed int Vcc_diff;
signed int Temperature_diff;
signed int Humidity_diff;
signed long Pressure_diff;
signed int Acceleration_diff[3];

unsigned int MEASURE = 1;

// ********** ********** ********** ********** COMMS ********** ********** ********** ********** //

// constants
#define	SPI	0x01
#define	I2C	0x02
// variables
char UCB0_MODE = 0;

// ********** ********** I2C ********** **********//
// pins
#define SDA_PIN 		BIT1		// msp430x261x 24x UCB0SDA pin
#define SCL_PIN 		BIT2		// msp430x261x 24x UCB0SCL pin

// variables
unsigned char I2C_TXData[2], I2C_RXData[3];	// TX and RX buffer limits
unsigned char* I2C_TXCntr;
unsigned char* I2C_RXCntr;
unsigned char I2C_TXByteCtr;
unsigned char I2C_RXByteCtr;
unsigned char READ;// = NO;
unsigned char Slave_Present = 2;
unsigned char RegisterData = 0x00;

// functions
void I2C_Init(void);
void I2C_Write_Init(void);
void I2C_Read_Init(void);
char I2C_Slave_Test(char Slave_Address);
unsigned char I2C_Read_Register(unsigned char Register_Address);
void I2C_Write_Register(char Register, char Value);

// ********** ********** END OF I2C ********** **********//

// ********** ********** SPI ********** **********//

// constants
#define nSS		BIT7		// P2.nSS used for /CS for ADXL362

// variables
unsigned char SPI_TXData[10], SPI_RXData[10];	// TX and RX buffer limits
unsigned char* SPI_TXCntr;
unsigned char* SPI_RXCntr;
unsigned char SPI_TXByteCtr;
unsigned char SPI_RXByteCtr;

unsigned char ID_adxl;

unsigned int SlavePresent;

// functions
void SPI_Init(void);		// Initialise SPI (for ADXL362)

// ********** ********** END OF SPI ********** **********//


// ********** ********** ********** ********** END OF COMMS ********** ********** ********** ********** //

// ********** ********** ********** ********** SENSORS ********** ********** ********** ********** //

// ******************* LPS25H *********************** //

#define	LPS25H_SLAVE_ADDRESS	0x5C		// = 0b101 1100
// registers
#define	REF_P_XL		0x08
#define	REF_P_L			0x09
#define	REF_P_H			0x0A
#define	WHO_AM_I		0x0F
#define	RES_CONF		0x10
	#define	AVGT1			BIT3
	#define	AVGT0			BIT2
	#define	AVGP1			BIT1
	#define	AVGP0			BIT0
#define	CTRL_REG1		0x20
	#define	PD				BIT7
	#define	ODR2			BIT6
	#define	ODR1			BIT5
	#define	ODR0			BIT4
	#define	DIFF_EN			BIT3
	#define	BDU				BIT2
	#define	RESET_AZ		BIT1
	#define	SIM				BIT0
#define CTRL_REG2		0x21	// CTRL_REG2
	#define	BOOT			BIT7
	#define	FIFO_EN			BIT6
	#define	WTM_EN			BIT5
	#define	FIFO_MEAN_DEC	BIT4
	#define	SPI_ENABLE		BIT3
	#define	SWRESET			BIT2
	#define	AUTO_ZERO		BIT1
	#define	ONE_SHOT		BIT0
#define	CTRL_REG3		0x22
	#define	INT_H_L			BIT7
	#define	PP_OD			BIT6
	#define	INT1_S2			BIT1
	#define	INT1_S1			BIT0
#define CTRL_REG4		0x23	// CTRL_REG4
	#define	P1_EMPTY		BIT3
	#define	P1_WTM			BIT2
	#define	P1_OVERRUN		BIT1
	#define	P1_DRDY			BIT0
#define	INT_CFG			0x24
	#define	LIR				BIT2
	#define	PL_E			BIT1
	#define	PH_E			BIT0
#define	INT_SOURCE		0x25
	#define	IA				BIT2
	#define	PL				BIT1
	#define	PH				BIT0
#define	STATUS_REG		0x27
	#define	P_OR			BIT5
	#define	T_OR			BIT4
	#define	P_DA			BIT1
	#define	T_DA			BIT0
#define	PRESS_OUT_XL	0x28
#define	PRESS_OUT_L		0x29
#define	PRESS_OUT_H		0x2A
#define	TEMP_OUT_L		0x2B
#define	TEMP_OUT_H		0x2C
#define	FIFO_CTRL		0x2E
	#define	F_MODE2			BIT7
	#define	F_MODE1			BIT6
	#define	F_MODE0			BIT5
	#define	WTM_POINT4		BIT4
	#define	WTM_POINT3		BIT3
	#define	WTM_POINT2		BIT2
	#define	WTM_POINT1		BIT1
	#define	WTM_POINT0		BIT0
#define	FIFO_STATUS		0x2F
	#define	WTM_FIFO		BIT7
	#define	FULL_FIFO		BIT6
	#define	EMPTY_FIFO		BIT5
	#define	DIFF_POINT4		BIT4
	#define	DIFF_POINT3		BIT3
	#define	DIFF_POINT2		BIT2
	#define	DIFF_POINT1		BIT1
	#define	DIFF_POINT0		BIT0
#define	THS_P_L			0x30
#define	THS_P_H			0x31
#define	RPDS_L			0x39
#define	RPDS_H			0x3A

#define AUTO_INCR	BIT7		// add to register address to auto-increment read/write

// variables
unsigned long Pressure = 1234;

// functions
unsigned long ReadPressure();

// ******************* END OF LPS25H *********************** //


// ****************** SHT21 ****************** //

// constants
#define SHT21_SLAVE_ADDRESS	0x40	//slave address = 0b1000 000

#define WRITE_USER_REGISTER	0xE6	// 0b1110’0110
#define READ_USER_REGISTER	0xE7	// 0b1110’0111
#define	SOFT_RESET			0xFE	//0b1111 1110
#define	T_HOLD_MASTER		0xE3	//0b1110 0011
#define	RH_HOLD_MASTER		0xE5	//0b1110 0101

#define RES0	0x00	// resolution settings
#define RES1	0x01	// resolution settings
#define RES2	0x10	// resolution settings
#define RES3	0x11	// resolution settings

#define RES_UB	BIT7	// upper bit of resolution setting
#define RES_LB	BIT0	// lower bit of resolution setting

// variables
unsigned int Humidity;
signed int Temperature;

// ****************** END OF SHT21 ****************** //

// ******************* ADXL362 *********************** //

// constants
/* --- command --- */
#define XL362_REG_WRITE			0x0A
#define XL362_REG_READ			0x0B
#define XL362_FIFO_READ			0x0D

/* ------- Register names ------- */
// ID constants
#define XL362_DEVID_AD           0x00
#define XL362_DEVID_MST          0x01
#define XL362_PARTID             0x02
#define XL362_REVID              0x03
#define XL362_XID                0x04

/* -- RO data -- */
#define XL362_XDATA8             0x08
#define XL362_YDATA8             0x09
#define XL362_ZDATA8             0x0A
#define XL362_STATUS             0x0B
#define XL362_FIFO_ENTRIES_L     0x0C
#define XL362_FIFO_ENTRIES_H     0x0D
#define XL362_XDATAL             0x0e
#define XL362_XDATAH             0x0f
#define XL362_YDATAL             0x10
#define XL362_YDATAH             0x11
#define XL362_ZDATAL             0x12
#define XL362_ZDATAH             0x13
#define XL362_TEMPL              0x14
#define XL362_TEMPH              0x15
#define XL362_X_ADCL             0x16
#define XL362_X_ADCH             0x17

/* -- Control and Config -- */
#define XL362_SOFT_RESET         0x1f
#define XL362_THRESH_ACTL        0x20
#define XL362_THRESH_ACTH        0x21
#define XL362_TIME_ACT           0x22
#define XL362_THRESH_INACTL      0x23
#define XL362_THRESH_INACTH      0x24
#define XL362_TIME_INACTL        0x25
#define XL362_TIME_INACTH        0x26
#define XL362_ACT_INACT_CTL      0x27
#define XL362_FIFO_CONTROL       0x28
#define XL362_FIFO_SAMPLES       0x29
#define XL362_INTMAP1            0x2a
#define XL362_INTMAP2            0x2b
#define XL362_FILTER_CTL         0x2c
#define XL362_POWER_CTL          0x2d
#define XL362_SELF_TEST			0x2e // can be used to check function of device.
#define XL362_ID				0xe2	// device should return this number when ID is read.

// soft reset                                              */
#define XL362_SOFT_RESET_KEY      0x52

// ACT_INACT_CTL
#define XL362_ACT_ENABLE      0x01
#define XL362_ACT_DISABLE     0x00
#define XL362_ACT_AC          0x02
#define XL362_ACT_DC          0x00
#define XL362_INACT_ENABLE    0x04
#define XL362_INACT_DISABLE   0x00
#define XL362_INACT_AC        0x08
#define XL362_INACT_DC        0x00
#define XL362_ACT_INACT_LINK  0x10
#define XL362_ACT_INACT_LOOP  0x20

//FIFO_CTL
#define XL362_FIFO_MODE_OFF     0x00
#define XL362_FIFO_MODE_FIFO    0x01
#define XL362_FIFO_MODE_STREAM  0x02
#define XL362_FIFO_MODE_TRIGGER 0x03
#define XL362_FIFO_TEMP         0x04
#define XL362_FIFO_SAMPLES_AH   0x08

// INTMAP1 and INTMAP2
#define XL362_INT_DATA_READY     0x01
#define XL362_INT_FIFO_READY     0x02
#define XL362_INT_FIFO_WATERMARK 0x04
#define XL362_INT_FIFO_OVERRUN   0x08
#define XL362_INT_ACT            0x10
#define XL362_INT_INACT          0x20
#define XL362_INT_AWAKE          0x40
#define XL362_INT_LOW            0x80

// FILTER_CTL
#define XL362_RATE_400        0x05
#define XL362_RATE_200        0x04
#define XL362_RATE_100        0x03  /* default */
#define XL362_RATE_50         0x02
#define XL362_RATE_25         0x01
#define XL362_RATE_12_5       0x00

#define XL362_EXT_TRIGGER     0x08
#define XL362_HALF_BW		  0x10

#define XL362_AXIS_X          0x00
#define XL362_AXIS_Y          0x10
#define XL362_AXIS_Z          0x20

#define XL362_RANGE_2G        0x00
#define XL362_RANGE_4G        0x40
#define XL362_RANGE_8G        0x80

// POWER_CTL
#define XL362_STANDBY           0x00
#define XL362_MEASURE        	0x02
#define XL362_AUTO_SLEEP        0x04
#define XL362_SLEEP             0x08

#define XL362_LOW_POWER         0x00
#define XL362_LOW_NOISE1        0x10
#define XL362_LOW_NOISE2        0x20
#define XL362_LOW_NOISE3        0x30

#define XL362_EXT_CLOCK         0x40
#define XL362_EXT_ADC           0x80

// SELF_TEST
#define XL362_SELFTEST_ON       0x01
#define XL362_SELFTEST_OFF      0x00

#define	XL362_DEV_ID			0xAD


// pin definitions
#define INT1	BIT1	// INT1 pin = P1.INT1
#define INT2	BIT2	// INT2 pin = P2.INT2

// variables
signed int Acceleration_Hi_Res[3];
signed char Acceleration_Lo_Res[3];

unsigned char ID_adxl;

// functions
void ADXL362_Init(void);
unsigned char ADXL362_ReadID(void);
unsigned char ADXL362_Read_Register(char Register);
void ADXL362_Write_Register(char Register, char Value);
void ADXL362_Single_Read(void);
void ADXL362_Single_Read_Hi_Res(void);
void ADXL362_Config(void);
void ADXL362_Read_XYZ_Reg(void);
void ADXL362_Read_XYZ_Reg_Hi_Res(void);
void ADXL362_Sleep(void);
int ADXL362_Present(void);

// ******************* END OF ADXL362 *********************** //


// ********** ********** ********** ********** END OF SENSORS ********** ********** ********** ********** //


// functions
void SHT21_Init(void);
void SHT21_Config(char Resolution);
unsigned char SHT21_Read_Register(char Register);
void SHT21_Write_Register(char Register, char Value);
unsigned int SHT21_Read_Data(char Register);
signed int SHT21_T_Read(void);
unsigned int SHT21_RH_Read(void);
signed int SHT21_T_Calib(unsigned int temp_data);
unsigned int SHT21_RH_Calib(unsigned int hum_data);

void Set_TimerA (void);
void Set_TimerB(void);
void delay(unsigned int BlinkCount);
void delay_minutes(unsigned int minutes);

// Data
/*
 * OA_Func_Set.h
 *
 *  Created on: 16 Nov 2014
 *      Author: Ralph S-J
 */

void OA0_Config(void);	// configure OA0
void Start_OA0_Slow(void);	// start OA0 (in slow slew rate)
void Start_OA0_Fast(void);	// start OA0 (in fast slew rate)
void Stop_OA0(void);		// stop OA0 from any slew rate


void OA1_Config(void);	// configure OA1
void Start_OA1_Slow(void);	// start OA1 (in slow slew rate)
void Start_OA1_Fast(void);	// start OA1 (in fast slew rate)
void Stop_OA1(void);		// stop OA1 from any slew rate

/*
 * PD_Func_Set.h
 *
 *  Created on: 16 Nov 2014
 *      Author: Ralph S-J
 */
unsigned int Vpd;
unsigned int Read_PD(void);	// read photodiode voltage (connected to P2.?)
unsigned int Read_PD1(void);	// read photodiode voltage (connected to P2.?)


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

  timer_state = timer_state_7;              // set timer state to 2 ~ 10 sec
  change_mode = 10;                         // Default GUI display mode set to
                                            // 10 sec

  TBCTL |= TBCLR;                           // Clear TBR counter
  TBCCR0 = sec10;	// a_d_wakeup_time;                 // set timer to wakeup time ~ 3 sec


  current_voltage = get_voltage();          // get current battery voltage
  if(current_voltage < VCC_RUN_VOLTAGE)
  {
    current_voltage = 0;
    while (current_voltage < BAT_MIN_VOTAGE)
    {
      __bis_SR_register(LPM3_bits + GIE);   // Enter LPM3 w/ interrupts
      current_voltage = get_voltage();
    }
  }

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
  uint8_t  msg[MESSAGE_LENGTH];

  // keep trying to link... Uses Timer B to wake up periodically
  while (SMPL_SUCCESS != SMPL_Link(&linkID1))
  {
    __bis_SR_register(LPM3_bits + GIE);     // LPM3 with interrupts enabled
  }

  // put radio to sleep once a successfull connection has been established
  SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, "" );

  	//   I2C
	I2C_Init();
	UCB0_MODE = I2C;
	Slave_Present = I2C_Slave_Test(LPS25H_SLAVE_ADDRESS);
	RegisterData = I2C_Read_Register(WHO_AM_I);

	Slave_Present = I2C_Slave_Test(SHT21_SLAVE_ADDRESS);

	// ADXL362
	P2SEL &= ~nSS;
	UCB0_MODE = SPI;
	SPI_Init();
	SlavePresent = ADXL362_Present();
	ADXL362_Write_Register(XL362_SOFT_RESET,XL362_SOFT_RESET_KEY);		// soft reset
	ID_adxl = ADXL362_ReadID();
	UCB0CTL1 |= UCSWRST;			// set to reset (power down)

	msg_status |= START_FLAG;

	if (TEST ==0 )
	{
		delay(sec10);                  // enter sleep mode to ensure battery is replenished
	}


	//create message
  while(1)
  {
	  if (MEASURE)
	  {
		  // save old values
			Vcc_last = Vcc;
			Temperature_last = Temperature;
			Humidity_last = Humidity;
			Pressure_last = Pressure;
			int i;
			for (i=0; i<3; i++)
			{
				Acceleration_last[i] = Acceleration_Hi_Res[i];
			}

			// Vcc
			Vcc = get_voltage();

			Vcc_diff = Vcc-Vcc_last;
			if (Vcc_diff <0 )
			{
				Vcc_diff *= -1;
			}
			if (Vcc_diff > VCC_DIFF_MAX)
			{
				msg_status |= VCC_FLAG;
			}

			if ( Vcc < VCC_SLEEP_VOLTAGE )
			{
				msg_status |= STOP_FLAG;
				MEASURE = 0;
			}
	//	  	RegisterData = 0;
			P3SEL &= ~BIT0;					// set to IO function
			P3OUT |= BIT0;					// Set CC2500 CS line high (DISABLE RADIO COMMS)
			P3DIR |= BIT0;

			I2C_Init();
			UCB0_MODE = I2C;

			// LPS25H
			// TESTI2C BUS FOR LPS25H
			UCB0CTL1 |= UCTR + UCTXSTT + UCTXSTP;       // I2C TX, start condition (Transmit mode), STOP
			 while (UCB0CTL1 & UCTXSTP);                 // wait for STOP condition
			UCB0I2CSA = LPS25H_SLAVE_ADDRESS;
	//	    RegisterData = I2C_Read_Register(WHO_AM_I);
			I2C_Write_Register(CTRL_REG1, PD);		// turn on pressure sensor (one-shot mode)
			I2C_Write_Register(CTRL_REG2, ONE_SHOT);		// trigger pressure measurement

			delay(90);		// pause 60 ms
			Pressure = ReadPressure();
			I2C_Write_Register(CTRL_REG1, 0);		// turn off pressure sensor

			Pressure_diff = Pressure - Pressure_last;
			if (Pressure_diff <0 )
			{
				Pressure_diff *= -1;
			}
			if (Pressure_diff > P_DIFF_MAX)
			{
				msg_status |= P_FLAG;
			}

	//
	////	  	delay(300);		// pause
	//
			// SHT21
			UCB0I2CSA = SHT21_SLAVE_ADDRESS;
			Humidity = SHT21_RH_Read();
			Humidity_diff = Humidity - Humidity_last;
			if ( Humidity_diff < 0 )
			{
				Humidity_diff *= -1;
			}
			if ( ( Humidity_diff > RH_DIFF_MAX ) | ( Humidity < RH_LOWER ) | ( Humidity > RH_UPPER ) )
			{
				msg_status |= RH_FLAG;
			}

	//	  	delay(150);		// pause
			Temperature = SHT21_T_Read();
			Temperature_diff = Temperature-Temperature_last;
			if (Temperature_diff <0 )
			{
				Temperature_diff *= -1;
			}
			if ( ( Temperature_diff > T_DIFF_MAX ) | ( Temperature < T_LOWER ) | ( Temperature > T_UPPER ) )
			{
				msg_status |= T_FLAG;
			}
	//
	////	  	delay(300);		// pause

			// ADXL362
			SPI_Init();
			UCB0_MODE = SPI;
	//		ADXL362_Write_Register(XL362_SOFT_RESET,XL362_SOFT_RESET_KEY);		// soft reset
			//ID_adxl = ADXL362_ReadID();
			// power on
			ADXL362_Write_Register(XL362_POWER_CTL,XL362_MEASURE);		// set into measurement emode
			delay(15);		// wait for samples
			// read data
	//		ADXL362_Read_XYZ_Reg();
			ADXL362_Read_XYZ_Reg_Hi_Res();
			// power off
			ADXL362_Write_Register(XL362_POWER_CTL,0);		// set into sleep emode
			UCB0CTL1 |= UCSWRST;			// set to reset (power down)
			for (i=0; i < 3; i++)
			{
				Acceleration_diff[i] = Acceleration_Hi_Res[i] - Acceleration_last[i];
				if (Acceleration_diff[i] <0 )
				{
					Acceleration_diff[i] *= -1;
				}
				if (Acceleration_diff[i] > ACC_DIFF_MAX )
				{
					msg_status |= ACC_FLAG;
				}
			}

	        Vpd = Read_PD();	// read photodiode voltage (connected to P2.0(-) to P2.2 (+))

//			if ( msg_status != 0 )
				{

		/* 		int i = 0;
				for (i=0 ; i < (MESSAGE_LENGTH) ; i++)
				{
					msg[i] = i;
				} */

				i=0;
				msg[i++] = NODE_ID;		// ID of node
				msg[i++] = (Vcc>>8)&0xFF;
				msg[i++] = Vcc&0xFF;
				msg[i++] = (Temperature>>8)&0xFF;
				msg[i++] = Temperature&0xFF;
				msg[i++] = (Humidity>>8)&0xFF;
				msg[i++] = Humidity&0xFF;
				msg[i++] = (Acceleration_Hi_Res[0]>>8)&0xFF;
				msg[i++] = Acceleration_Hi_Res[0]&0xFF;
				msg[i++] = (Acceleration_Hi_Res[1]>>8)&0xFF;
				msg[i++] = Acceleration_Hi_Res[1]&0xFF;
				msg[i++] = (Acceleration_Hi_Res[2]>>8)&0xFF;
				msg[i++] = Acceleration_Hi_Res[2]&0xFF;
				msg[i++] = ((Pressure>>8)>>8)&0xFF;
				msg[i++] = (Pressure>>8)&0xFF;
				msg[i++] = Pressure&0xFF;
				msg[i++] = (Vpd>>8)&0xFF;
				msg[i++] = Vpd&0xFF;
				msg[i++] = msg_status;


				MRFI_Init();
		//		mrfiSpiInit();
			// Wake radio-up
			SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_AWAKE, "" );

			  // Send message
			  if (SMPL_SUCCESS == SMPL_Send(linkID1, msg, sizeof(msg)))
			  {
		 //         status_indicator(status_one, 2);	// Blink Green LED if success.
			  }
			  else                                  // Blink RED LED if transmission
			  {                                     // failed
		//        status_indicator(status_one, 1);
			  }
			SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, "" );


		 //   status_indicator(status_one, 2);	// Blink Green LED if success.
			msg_status = 0;
				}	// end of status if loop
				if (TEST)
				{
					delay(sec2);                  // enter sleep mode between measurements

				}
				else
				{
				    for (i=0 ; i < 10  ; i++)
					{
						   delay(sec30);                  // enter sleep mode between measurements
					}
				}

	}	// end of MEASURE if loop

	  else
	  {
		  Vcc = get_voltage();
		  if (Vcc > VCC_REAWAKE_VOLTAGE)
		  {
			  msg_status |= RESTART_FLAG;
			  msg_status &= STOP_FLAG;
		  }
		  else
		  {
				int i;
			    for (i=0 ; i <20  ; i++)
				{
					   delay(sec30);                  // enter sleep mode between measurements
				}
//			//	   delay(sec10);                  // enter sleep mode between measurements
		  }
	  }

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
  rt_volts = ((long)rt_volts*625)/128;
  return (unsigned int)(rt_volts);
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
	  case timer_state_7:                     // Timer State == 40; 40 Secs
      delay(sec40);
      in_delay = 0;
      battery_full_timer += sec40;
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

// ******************************************************* //
//******************************************************************************************//
// FUNCTION DEFINITIONS
//******************************************************************************************//

// ********** ********** ********** ********** SENSORS ********** **********  ********** **********//


// ********** ********** LPS25H ********** **********//

// ****************************************** //
// ReadPressure
// INPUT: NONE
// OUTPUT: Pressure (mbar * 100) as long
// ****************************************** //
unsigned long ReadPressure()
{
	unsigned long PRESSURE_LSB, Pressure;
    I2C_TXByteCtr = 1;                          // Load TX byte counter
    I2C_RXByteCtr = 3;
    I2C_TXCntr = I2C_TXData;
    I2C_RXCntr = I2C_RXData;

    I2C_TXData[0] = PRESS_OUT_XL | BIT7;
    I2C_Write_Init();
    READ = YES;
	UCB0CTL1 |= UCTXSTT;// + UCTXSTP;       // I2C TX, start condition
	__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
	                                            // Remain in LPM0 until all data
	                                            // is TX'd
	I2C_Read_Init();                      // Init I2C module for a read operation
    UCB0CTL1 |= UCTXSTT;// + UCTXSTP;                // start condition generation
	__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts

	PRESSURE_LSB = I2C_RXData[2]<<8;
	PRESSURE_LSB = PRESSURE_LSB<<8;
	PRESSURE_LSB |= ((I2C_RXData[1]<<8) & 0x0000FF00);
	PRESSURE_LSB |= I2C_RXData[0];

	PRESSURE_LSB &= ~(0xFF000000);

	Pressure = (PRESSURE_LSB*100)>>12;

	return Pressure;
}

// ********** ********** END OF LPS25H ********** **********//

// ********** **********  SHT21 ********** **********//

// Initialise SHT21
void SHT21_Init(void)
{
	// Initialise I2C
//	USCI_MODE = I2C_Init();
//	SHT21_Config();
//	SHT21 remains in LPM between measurements
}

// Configure SHT21 sensor
void SHT21_Config(char Resolution)
{
	unsigned char UserRegister;
	UserRegister = 	I2C_Read_Register(READ_USER_REGISTER);
	switch (Resolution)
	{
		case RES0:
			UserRegister &= ~(RES_UB|RES_LB);		// set resolution bits to LO
		break;

		case RES1:
			UserRegister |= RES_LB;			// LB to HI
			UserRegister &= ~RES_UB;		// set UB to LO
		break;

		case RES2:
			UserRegister &= ~RES_LB;		// LB to LO
			UserRegister |= RES_UB;			// set UB to HI
		break;

		case RES3:
			UserRegister |= (RES_UB|RES_LB);		// set resolution bits to HI
		break;

		default:
		break;
	}

//	UserRegister |= Resolution;	// set '1' bits HIGH
//
//	//UserRegister &= ~Resolution;		// set '0' bit LOW
//	UserRegister &= (Resolution|(~(RES_UB|RES_LB)));		// set '0' bits LOW

	SHT21_Write_Register(WRITE_USER_REGISTER, UserRegister);
}
// Read SHT21 Data
unsigned char SHT21_Read_Register(char Register)
{
	unsigned char RegisterContent;

	UCB0I2CSA = SHT21_SLAVE_ADDRESS;		// set I2C SA to SHT21

	I2C_TXByteCtr = 1;                          // Load TX byte counter
	I2C_RXByteCtr = 1;
	I2C_TXCntr = I2C_TXData;
	I2C_RXCntr = I2C_RXData;

	I2C_TXData[0] = Register;
	I2C_Write_Init();
	READ = YES;
	UCB0CTL1 |= UCTXSTT;// + UCTXSTP;       // I2C TX, start condition
	__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
												// Remain in LPM0 until all data
												// is TX'd
	I2C_Read_Init();                      // Init I2C module for a read operation
	UCB0CTL1 |= UCTXSTT;// + UCTXSTP;                // start condition generation
	while (UCB0CTL1 & UCTXSTT);             // Start condition sent?
	UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
	__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts

	RegisterContent = I2C_RXData[0];
	return RegisterContent;
}
// Write SHT21 Data
void 	SHT21_Write_Register(char Register, char Value)
{
	UCB0I2CSA = SHT21_SLAVE_ADDRESS;		// set I2C SA to SHT21

	I2C_TXByteCtr = 2;                          // Load TX byte counter
	I2C_RXByteCtr = 0;
	I2C_TXCntr = I2C_TXData;
	I2C_RXCntr = I2C_RXData;

	I2C_TXData[0] = Register;
	I2C_TXData[1] = Value;
	I2C_Write_Init();
	READ = NO;
	UCB0CTL1 |= UCTXSTT;// + UCTXSTP;       // I2C TX, start condition
	__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}
// Read SHT21 Register
unsigned int SHT21_Read_Data(char Register)
{
	unsigned int Data;

	UCB0I2CSA = SHT21_SLAVE_ADDRESS;		// set I2C SA to SHT21

	I2C_TXByteCtr = 1;                          // Load TX byte counter
	I2C_RXByteCtr = 2;
	I2C_TXCntr = I2C_TXData;
	I2C_RXCntr = I2C_RXData;

	I2C_TXData[0] = Register;
	I2C_Write_Init();
	READ = YES;
	UCB0CTL1 |= UCTXSTT;// + UCTXSTP;       // I2C TX, start condition
	__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
												// Remain in LPM0 until all data
												// is TX'd
	I2C_Read_Init();                      // Init I2C module for a read operation
	UCB0CTL1 |= UCTXSTT;// + UCTXSTP;                // start condition generation
//	while (UCB0CTL1 & UCTXSTT);             // Start condition sent?
//	UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
	__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts

	Data = ((I2C_RXData[0]<<8) + I2C_RXData[1]);
	return Data;
}

// Read SHT21(T & RH)-> sleep SHT21
signed int SHT21_T_Read(void)
{
	signed int Temperature;
	unsigned int T_uncal;
	T_uncal = SHT21_Read_Data(T_HOLD_MASTER);
	T_uncal = ((I2C_RXData[0]<<8) + I2C_RXData[1]);
	Temperature = SHT21_T_Calib(T_uncal);
	return Temperature;
}
unsigned int SHT21_RH_Read(void)
{
	signed int Humidity;
	unsigned int Rh_uncal;
	Rh_uncal = SHT21_Read_Data(RH_HOLD_MASTER);
	Rh_uncal = ((I2C_RXData[0]<<8) + I2C_RXData[1]);
	Humidity = SHT21_RH_Calib(Rh_uncal);
	return Humidity;
}


signed int SHT21_T_Calib(unsigned int temp_data)

{
  float temp;              // variable for result

  temp_data &= ~0x0003;           // clear bits [1..0] (status bits)
  //-- calculate temperature [у] --
  temp = -46.85 + 175.72/65536 *(float)temp_data; //T= -46.85 + 175.72 * ST/2^16

  return (signed int)(temp*100);
}

unsigned int SHT21_RH_Calib(unsigned int hum_data)

{
  float hum;              // variable for result

  hum_data &= ~0x0003;          // clear bits [1..0] (status bits)
  //-- calculate relative humidity [%RH] --
  hum = -6.0 + 125.0/65536 * (float)hum_data; // RH= -6 + 125 * SRH/2^16

  return (unsigned int)(hum*100);
}

// ********** **********  END OF SHT21 ********** **********//

// ********** ********** ADXL362 ********** **********//

// Initialise ADXL362
void ADXL362_Init(void)
{
	// Initialise SPI
//	USCI_MODE = SPI_Init();
	//Configure ADXL362
	ADXL362_Config();
	// Set ADCL362 to sleep mode
	ADXL362_Sleep();
}
void ADXL362_Interrupts_Init(void)
{

}
unsigned char ADXL362_ReadID(void)
{
	unsigned char ID_adxl;

	SPI_TXData[0] = XL362_REG_READ;		// Make outgoing message		WRITE
	SPI_TXData[1] = XL362_DEVID_AD;		// Make outgoing message		ADDRESS(POWER_CTL)

	SPI_TXByteCtr = 2;      // Load TX byte counter with no. of bytes to write.  This is only 1 in the case that a read is made.
	SPI_TXCntr = SPI_TXData;		// set TX pointer to start of TXData array
	SPI_RXData[0] = 0xFF;
	SPI_RXByteCtr = 1;		// recieve only 1 byte through SPI.
	SPI_RXCntr = SPI_RXData;		// set RX pointer to start of RXData array.

	P2OUT &= ~nSS;					// Lower nSS (make active)
	__bic_SR_register(GIE);			// disable GIE so interrupt is not triggered as soon as TXIE is set
	IE2 |= UCB0TXIE;			// Enable TX interrupts, starts SPI
	__bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
	IFG2 &= ~UCB0RXIFG;		// clear RX flag so doesn't trigger straight away
	UCB0TXBUF = 55;		// =01010101                     // Load TX buffer to trigger read from accelerometer
	IE2 |= UCB0RXIE;		// Enable RX interrupts
	__bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
	P2OUT |= nSS;					// Raise nSS (make inactive)

	ID_adxl = SPI_RXData[0];

	return ID_adxl;
}

unsigned char ADXL362_Read_Register(char Register)
{
	unsigned char Register_Content = 0;

	// Set resolution/operating mode of ADXL362void ConfigureADXL()
	SPI_TXData[0] = XL362_REG_READ;		// Make outgoing message		WRITE
	SPI_TXData[1] = Register;		// Make outgoing message		ADDRESS(POWER_CTL)

	SPI_TXByteCtr = 2;      // Load TX byte counter with no. of bytes to write.  This is only 1 in the case that a read is made.
	SPI_TXCntr = SPI_TXData;		// set TX pointer to start of TXData array
	SPI_RXData[0] = 0;
	SPI_RXByteCtr = 1;		// recieve only 1 byte through SPI.
	SPI_RXCntr = SPI_RXData;		// set RX pointer to start of RXData array.

	P2OUT &= ~nSS;					// Lower nSS (make active)
	__bic_SR_register(GIE);			// disable GIE so interrupt is not triggered as soon as TXIE is set
	IE2 |= UCB0TXIE;			// Enable TX interrupts, starts SPI
	__bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled

	IFG2 &= ~UCB0RXIFG;		// clear RX flag so doesn't trigger straight away
	UCB0TXBUF = 0;                     // Load TX buffer to trigger read from accelerometer

	__bic_SR_register(GIE);			// disable GIE so interrupt is not triggered as soon as TXIE is set
	IE2 |= UCB0RXIE;		// Enable RX interrupts
	__bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
	P2OUT |= nSS;					// Raise nSS (make inactive)

	Register_Content = SPI_RXData[0];

	return Register_Content;
}

// Read ADXL362 (X,Y,Z data)
void ADXL362_Read_XYZ_Reg(void)
{
	SPI_TXData[0] = XL362_REG_READ;		// Make outgoing message		WRITE
	SPI_TXData[1] = XL362_XDATA8;		// Make outgoing message		ADDRESS(POWER_CTL)

	SPI_TXByteCtr = 2;      // Load TX byte counter with no. of bytes to write.  This is only 1 in the case that a read is made.
	SPI_TXCntr = SPI_TXData;		// set TX pointer to start of TXData array
	SPI_RXData[0] = 0;
	SPI_RXByteCtr = 3;		// recieve 3 bytes through SPI.
	SPI_RXCntr = SPI_RXData;		// set RX pointer to start of RXData array.

	P2OUT &= ~nSS;					// Lower nSS (make active)
	__bic_SR_register(GIE);			// disable GIE so interrupt is not triggered as soon as TXIE is set
	IE2 |= UCB0TXIE;			// Enable TX interrupts, starts SPI
	__bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
	IFG2 &= ~UCB0RXIFG;		// clear RX flag so doesn't trigger straight away
	UCB0TXBUF = 0;                     // Load TX buffer to trigger read from accelerometer
	__bic_SR_register(GIE);			// disable GIE so interrupt is not triggered as soon as TXIE is set
	IE2 |= UCB0RXIE;		// Enable RX interrupts
	__bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
	P2OUT |= nSS;					// Raise nSS (make inactive)

	int i = 0;
	for (i=0; i<3; i++)
		{
			Acceleration_Lo_Res[i] = SPI_RXData[i];
		}

 }

// Read ADXL362 (X,Y,Z data)
void ADXL362_Read_XYZ_Reg_Hi_Res(void)
{

	SPI_TXData[0] = XL362_REG_READ;		// Make outgoing message		READ
	SPI_TXData[1] = XL362_XDATAL;		// Make outgoing message		ADDRESS(POWER_CTL)

	SPI_TXByteCtr = 2;      // Load TX byte counter with no. of bytes to write.  This is only 1 in the case that a read is made.
	SPI_TXCntr = SPI_TXData;		// set TX pointer to start of TXData array
	SPI_RXData[0] = 0;
	SPI_RXByteCtr = 6;		// recieve byte no. through SPI.
	SPI_RXCntr = SPI_RXData;		// set RX pointer to start of RXData array.

	P2OUT &= ~nSS;					// Lower nSS (make active)
	__bic_SR_register(GIE);			// disable GIE so interrupt is not triggered as soon as TXIE is set
	IE2 |= UCB0TXIE;			// Enable TX interrupts, starts SPI
	__bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
	IFG2 &= ~UCB0RXIFG;		// clear RX flag so doesn't trigger straight away
	UCB0TXBUF = 0;                     // Load TX buffer to trigger read from accelerometer
	__bic_SR_register(GIE);			// disable GIE so interrupt is not triggered as soon as TXIE is set
	IE2 |= UCB0RXIE;		// Enable RX interrupts
	__bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
	P2OUT |= nSS;					// Raise nSS (make inactive)


	int i = 0;
	for (i=0; i<3; i++)
	{
		Acceleration_Hi_Res[i] = (SPI_RXData[2*i]<<4 |(( SPI_RXData[2*i+1]&0x0F)<<12))/16;
	}

}
void ADXL362_Single_Read(void)
{
	unsigned char Register;
	Register = ADXL362_Read_Register(XL362_POWER_CTL);

	// power on
	ADXL362_Write_Register(XL362_POWER_CTL,(XL362_MEASURE |Register));
	delay(150);		// wait for samples

	// read data
	ADXL362_Read_XYZ_Reg();

	// power off
	Register = ADXL362_Read_Register(XL362_POWER_CTL);
	ADXL362_Write_Register(XL362_POWER_CTL,Register&~XL362_MEASURE);
}
void ADXL362_Single_Read_Hi_Res(void)
{
	unsigned char Register;
	Register = ADXL362_Read_Register(XL362_POWER_CTL);

	// power on
	ADXL362_Write_Register(XL362_POWER_CTL,(XL362_MEASURE |Register));
	delay(150);		// wait for samplee

	// read data
	ADXL362_Read_XYZ_Reg_Hi_Res();

	// power off
	Register = ADXL362_Read_Register(XL362_POWER_CTL);
	ADXL362_Write_Register(XL362_POWER_CTL,Register&~XL362_MEASURE);
}
void ADXL362_Write_Register(char Register, char Value)
{
	// Set resolution/operating mode of ADXL362void ConfigureADXL()
	SPI_TXData[0] = XL362_REG_WRITE;		// Make outgoing message		WRITE
	SPI_TXData[1] = Register;		// Make outgoing message		ADDRESS(POWER_CTL)
	SPI_TXData[2] = Value;

	SPI_TXByteCtr = 3;      // Load TX byte counter with no. of bytes to write.  This is only 1 in the case that a read is made.
	SPI_TXCntr = SPI_TXData;		// set TX pointer to start of TXData array

	P2OUT &= ~nSS;					// Lower nSS (make active)
	__bic_SR_register(GIE);			// disable GIE so interrupt is not triggered as soon as TXIE is set
	IE2 |= UCB0TXIE;			// Enable TX interrupts, starts SPI
	__bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled - wait for interrupts
	P2OUT |= nSS;					// Raise nSS (make inactive)
}
// Configure ADXL362
void ADXL362_Config(void)
{
	// Set resolution/operating mode of ADXL362void ConfigureADXL()
	SPI_TXData[0] = XL362_REG_WRITE;		// Make outgoing message		WRITE
	SPI_TXData[1] = XL362_POWER_CTL;		// Make outgoing message		ADDRESS(POWER_CTL)
	SPI_TXData[2] = XL362_MEASURE;		// Make outgoing message		WRITE
	//SPI_TXData[3] = 0x02;		// Make outgoing message		RANDOM
	//SPI_TXData[4] = 0x02;		// Make outgoing message		RANDOM

	SPI_RXCntr = SPI_TXCntr = 0;		// Initialize message byte counters
	P2OUT &= ~nSS;					// Lower nSS (make active)

	IE2 |= UCB0TXIE;			// Enable TX interrupts, starts SPI
	__low_power_mode_0();		// Wait for SPI (needs SMCLK)
	// SPI transaction completed		(but USCI would keep SMCLK active)
	P2OUT |= nSS;					// Raise nSS (make inactive)
}

// Set ADXL362 into sleep mode
void ADXL362_Sleep(void)
{
	// Write value to CTRL register
}
//#pragma vector=PORT2_VECTOR
//__interrupt void Port_2(void)
//{
//	if(P2IFG & INT1)	// is it on P2.INT1
//	{
//		P2OUT &= ~INT2;		// INT2 LOW
//		  __bic_SR_register_on_exit(LPM0_bits);     // Clear CPUOFF bit from 0(SR)
//		  P2IFG &= ~INT1;                           // P2.INT1 IFG cleared key interuped
//	}
//}
int ADXL362_Present(void)
{
	unsigned char ID_adxl;
	ID_adxl = ADXL362_ReadID();
	if (ID_adxl == XL362_DEV_ID)
	{
		return YES;
	}
	else
	{
		return NO;
	}
}
// ********** **********  END OF ADXL362 ********** **********//

// ********** ********** ********** ********** END OF SENSORS ********** **********  ********** **********//


// ********** ********** ********** ********** COMMS ********** **********  ********** **********//

// ********** **********  I2C ********** **********//

// ****************************************** //
// I2C_Init
// INPUT: NONE
// OUTPUT: NONE
// OPERATION: Sets up UCBO for I2C communication
// ****************************************** //
void I2C_Init(void)
{
	P3OUT |= 0x01;
	P3DIR |= 0x01;
	// Select I2C pins, write USCI registers
	P3SEL |= SDA_PIN + SCL_PIN;                 // Assign I2C pins to USCI
	UCB0CTL1 = UCSWRST;                         // Enable SW reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;       // I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_2 + UCSWRST;              // Use SMCLK, keep SW reset
	UCB0BR0 = 10;                         		// set prescaler (fSCL = 100kHz)
	UCB0BR1 = 0;
	// UCB0I2CSA = SHT21_ADDRESS;                  // set slave address
	UCB0CTL1 &= ~UCSWRST;                       // Clear SW reset, resume operation
//	UCB0I2CIE = UCNACKIE;
//	IE2 |= UCB0RXIE | UCB0TXIE;                 // Enable interrupts
}

// ****************************************** //
// SlavePresent
// INPUT: NONE
// OUTPUT: NONE
// OPERATION: Pulse red light repeatedly when no slave is connected.  Otherwise pulse green 5 times.
// ****************************************** //
char I2C_Slave_Test(char Slave_Address){

	char Slave_Present = 2;

    IE2 &= ~UCB0TXIE;                       // disable Transmit ready interrupt
    IE2 &= ~UCB0RXIE;                        // disable Receive ready interrupt

	UCB0I2CSA = Slave_Address;              // set slave to test for
	while (Slave_Present != YES)
	{
		UCB0CTL1 |= UCTR + UCTXSTT + UCTXSTP;       // I2C TX, start condition (Transmit mode), STOP
		 while (UCB0CTL1 & UCTXSTP);                 // wait for STOP condition
		 if ((UCB0STAT & UCNACKIFG)== 0)		// SLAVE PRESENT?
		 {
			  Slave_Present = YES;
	          status_indicator(status_five, 0);
		 }
		else
		{
			Slave_Present = NO;
			P1DIR |= BIT0+BIT1;			// Light LED
			P1OUT &= ~BIT1;				// GREEN LED OFF
			P1OUT |= BIT0;				// RED LED ON
			delay(150);	// pause 0.1 second
			P1OUT &= ~(BIT0);				// RED LEDs
			delay(150);	// pause 0.1 second
		}
	}
	 return Slave_Present;
}

// ****************************************** //
// I2C_Read_Register
// INPUT: Adress of register
// OUTPUT: Register value (char)
// ****************************************** //
unsigned char I2C_Read_Register(unsigned char Register_Address)
{
	unsigned char Register_Content;

	I2C_TXByteCtr = 1;                          // Load TX byte counter
	I2C_RXByteCtr = 1;
	I2C_TXCntr = I2C_TXData;
	I2C_RXCntr = I2C_RXData;

	I2C_TXData[0] = Register_Address;
	I2C_Write_Init();
	READ = YES;
	__bic_SR_register(GIE);			// disable GIE so interrupt is not triggered as soon as START is sent???
	UCB0CTL1 |= UCTXSTT;// + UCTXSTP;       // I2C TX, start condition
	__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
												// Remain in LPM0 until all data
												// is TX'd
	I2C_Read_Init();                      // Init I2C module for a read operation
	UCB0CTL1 |= UCTXSTT;// + UCTXSTP;                // start condition generation
	while (UCB0CTL1 & UCTXSTT);             // Start condition sent?
	UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
	__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts

	Register_Content = I2C_RXData[0];
	return Register_Content;

}
// Write I2C Data
void I2C_Write_Register(char Register, char Value)
{
	I2C_TXByteCtr = 2;                          // Load TX byte counter
	I2C_RXByteCtr = 0;
	I2C_TXCntr = I2C_TXData;
	I2C_RXCntr = I2C_RXData;

	I2C_TXData[0] = Register;
	I2C_TXData[1] = Value;
	I2C_Write_Init();
	READ = NO;
//	__bic_SR_register(GIE);			// disable GIE so interrupt is not triggered as soon as START is sent???
	UCB0CTL1 |= UCTXSTT;// + UCTXSTP;       // I2C TX, start condition
	__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

/*----------------------------------------------------------------------------*/
// Initialization of the I2C Module for Write operation.
void I2C_Write_Init(void)
{
    UCB0CTL1 |= UCTR;                       // UCTR=1 => Transmit Mode (R/W bit = 0)
    IFG2 &= ~UCB0TXIFG;                     // Clear TXIFG flag
    IE2 &= ~UCB0RXIE;                       // disable Receive ready interrupt
    IE2 |= UCB0TXIE;                       // enable Transmit ready interrupt
}

/*----------------------------------------------------------------------------*/
//   Initialization of the I2C Module for Read operation.
void I2C_Read_Init(void)
{
    UCB0CTL1 &= ~UCTR;                      // UCTR=0 => Receive Mode (R/W bit = 1)
    IFG2 &= ~UCB0RXIFG;                     // Clear RXIFG flag
    IE2 &= ~UCB0TXIE;                       // disable Transmit ready interrupt
    IE2 |= UCB0RXIE;                        // enable Receive ready interrupt
}

// ********** ********** END OF I2C ********** **********//

// ********** ********** SPI ********** **********//

// Initialise SPI (for ADXL362)
void SPI_Init(void)
{
	// Set /CS line for CC2500 HIGH (hardwired into code) and create /CS1 line in software.

	P3OUT |= BIT0;					// Set CC2500 CS line high
	P3DIR |= BIT0;

	P2OUT |= nSS;					// preload nSS for slave (s) to HIGH
	P2DIR |= nSS;					// set nSS pin for slave(s) to OUTPUT
//		P3DIR &= ~BIT2;		// set SOMI pin as input
//		P3OUT |= BIT2;		// set SOMI pin as HIGH
	P3SEL = BIT1 | BIT2 | BIT3;
	// SPI mode 3 needs CPOL=CKPL=1, CPHA=1 -> CKPH=0; msb first, master,
	//   8 bit (default), 3-wire (default, mode 0), synchronous
	UCB0CTL0 = UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;
	UCB0CTL1 = UCSSEL1 | UCSWRST;	// Clock from SMCLK; hold in reset
	UCB0BR1 = 0;					// Upper byte of divider word
	UCB0BR0 = 1;					// Clock = SMCLK / 1 = 1MHz
	UCB0CTL1 &= ~UCSWRST;			// Release from reset - increases power consumption
	IE2 &= ~UCB0RXIE;				// Disable interrupts on receive
	IE2 &= ~UCB0TXIE;				// Disable interrupts on transmit otherwise flag will be lifted instanltl
}

// ********** ********** END OF SPI ********** **********//


// ************************************************************************** //
// INTERRUPT VECTORS
// ************************************************************************** //

// I2C *******************************************
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
	switch (UCB0_MODE)
	{
		case I2C:
			if(UCB0TXIFG & IFG2)
			{
				if (I2C_TXByteCtr)                            // Check TX byte counter
				{
					UCB0TXBUF = *I2C_TXCntr++;                     // Load TX buffer
					I2C_TXByteCtr--;                            // Decrement TX byte counter
				}
				else
				{
					if(READ != YES)
					{
						UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
					}

					IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
					__bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
				}
			}
			else if(UCB0RXIFG & IFG2)                    // Make sure it is an RX interrupt
			{
				*I2C_RXCntr++ = UCB0RXBUF;               // store received data in buffer
				I2C_RXByteCtr--;
				if (I2C_RXByteCtr == 1)		// second to last TX
				{
					UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
				}
				else if (I2C_RXByteCtr == 0)
				{
					IFG2 &= ~UCB0RXIFG;             // Clear RXIFG flag
					IFG2 &= ~UCB0TXIFG;             // Clear TXIFG flag
					IE2 &= ~UCB0TXIE;               // Disable Transmit ready interrupt
					IE2 &= ~UCB0RXIE;               // Disable Receive ready interrupt
					__bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
				}
			}
		break;

		case SPI:
			UCB0TXBUF = *SPI_TXCntr++;                     // Load TX buffer
			SPI_TXByteCtr--;                            // Decrement TX byte counter
			if (SPI_TXByteCtr==0)				//last byte
			{
				IE2 &= ~UCB0TXIE;		// Disable further interrupts
				  __bic_SR_register_on_exit(LPM0_bits);     // Clear CPUOFF bit from 0(SR)
			}
		break;

		default:
		break;

	}	// end of switch
}

// SPI*******************************************

//// USCIAB0TX_ISR - pre-load TXByteCtr with the no. of bytes to TX before this interrupt is called
//
//#pragma vector = USCIAB0TX_VECTOR
//__interrupt void USCIAB0TX_ISR(void)
//{
//	case SPI:
//		UCB0TXBUF = *SPI_TXCntr++;                     // Load TX buffer
//		SPI_TXByteCtr--;                            // Decrement TX byte counter
//		if (SPI_TXByteCtr==0)				//last byte
//		{
//			IE2 &= ~UCB0TXIE;		// Disable further interrupts
//			  __bic_SR_register_on_exit(LPM0_bits);     // Clear CPUOFF bit from 0(SR)
//		}
//	break;
//}

//----------------------------------------------------------------------
 // ISR for USCI_A,B0 RX: store data, return when all bytes received
 //----------------------------------------------------------------------
 #pragma vector = USCIAB0RX_VECTOR
 __interrupt void USCIAB0RX_ISR (void)	// Acknowledge by read of RXBUF
 {
 	*SPI_RXCntr++ = UCB0RXBUF;	// Store recd data, update counter
 	SPI_RXByteCtr--;
 	if (SPI_RXByteCtr)
 	{
 		UCB0TXBUF = 0x00;
 	}
 	else
 	{		// Received complete message?
 		IE2 &= ~UCB0RXIE;		// Disable further interrupts
 		  __bic_SR_register_on_exit(LPM0_bits);     // Clear CPUOFF bit from 0(SR)
 	}
 }

 // Timer
 void Set_TimerB(void)
 {
 	BCSCTL3 |= LFXT1S_2;                      // LFXT1 = ACLK = VLO
 	TBCCTL0 = 0;						// TBCCR0 interrupt disabled
 	TBCTL = TBSSEL_1 + MC_1 + ID_3;           // ACLK, upmode, Divider = 8
 }

 unsigned int ReadADC_2_5(unsigned int INCH)	//Vref = 2.5V read ADC from INCH channel
 {
 	unsigned int ADC_Read;
     // Measure PD Voltage
     ADC10CTL1 = INCH;                    // read input channel
     ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE + REF2_5V;
     __delay_cycles(350);                    // delay to allow reference to settle
     ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
     __bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
     ADC_Read = ADC10MEM;
     ADC10CTL0 &= ~ENC;
     ADC10CTL0 &= ~(REFON + ADC10ON);        // turn off A/D to save power

 	return ADC_Read;
 }

 unsigned int ReadADC_1_5(unsigned int INCH)	//Vref = 2.5V read ADC from INCH channel
 {
 	unsigned int ADC_Read;
     // Measure PD Voltage
     ADC10CTL1 = INCH;                    // read input channel
     ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;	//
     __delay_cycles(350);                    // delay to allow reference to settle
     ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
     __bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
     ADC_Read = ADC10MEM;
     ADC10CTL0 &= ~ENC;
     ADC10CTL0 &= ~(REFON + ADC10ON);        // turn off A/D to save power

 	return ADC_Read;
 }

 /*
  * OA_Func_Set.c
  *
  *  Created on: 16 Nov 2014
  *      Author: Ralph S-J
  * Functions for reading op-amps
  */
 void OA0_Config(void)	// configure OA0 and start op-amp
 {
     OA0CTL0 = OAN_1 | OAP_0;			//-ive input = OA0I1, +ive input = OA0I0, output channels O:A1,A3,A5
     ADC10AE0 = BIT0 + BIT1 + BIT2;		// route output to A1 (p2.1) and enable inputs.  Without this line the pins are not enabled.
 }

 void Start_OA0_Slow(void)	// start op amp (in slow slew rate)
 {
 	OA0CTL0 |= OAPM_1;		// start op-amp in slow slew rate mode (lowest power consumption
 }

 void Start_OA0_Fast(void)	// start op amp (in slow slew rate)
 {
 	OA0CTL0 |= OAPM_3;		// start op-amp in fast slew rate mode (highest power consumption
 }

 void Stop_OA0(void)
 {
 	OA0CTL0 &= ~OAPM_3;		// stops op-amp by clearing bits form slew rate selector (will turn op-amp off form fast. medium or slow mode)
 }

 void OA1_Config(void)	// configure OA0 and start op-amp
 {
 	OA1CTL0 = OAN_1 | OAP_0 | OAADC0;		// select input/output channels I-:OA1I1, I+:OA1I0, O:A12,13,14
     ADC10AE1 = BIT5;		// enable AO1 output to A13 (p4.4)
     ADC10AE0 =  BIT3 + BIT4;	// enable OA1I1, enable OA1I0
 }

 void Start_OA1_Slow(void)	// start op amp (in slow slew rate)
 {
 	OA1CTL0 |= OAPM_1;		// start op-amp in slow slew rate mode (lowest power consumption
 }
 void Start_OA1_Fast(void)	// start op amp (in slow slew rate)
 {
 	OA1CTL0 |= OAPM_3;		// start op-amp in fast slew rate mode (highest power consumption
 }
 void Stop_OA1(void)
 {
 	OA1CTL0 &= ~OAPM_3;		// stops op-amp by clearing bits form slew rate selector (will turn op-amp off form fast. medium or slow mode)
 }

 /*
  * PD_Func_Set.c
  *
  *  Created on: 16 Nov 2014
  *      Author: Ralph S-J
  */

 unsigned int Read_PD(void)	// read photodiode voltage (connected to P2.0 pin3(+) to P2.2 (-) pin5)
 {
 	unsigned int voltage;
 	unsigned long voltage_long;
 	unsigned int ADC;

 	OA0_Config();	// configure OA0 (pins and input/ouput channels)
 	Start_OA0_Slow();	// start op amp (in slow slew rate)
 	delay(150);		// delay to allow op-amp to settle
     ADC = ReadADC_1_5(INCH_1);	//Vref = 1.5V read ADC from INCH channel A1, P2.1 pin4
     Stop_OA0();

     voltage_long = (unsigned long)ADC;
     voltage_long = voltage_long*1000;
     voltage = voltage_long>>10;

 	return voltage;
 }

 unsigned int Read_PD1(void)	// read photodiode voltage (connected to OA1I0 P2.3 pin6(+) - OA1I1 P2.4 (-) pin7)
 {
 	unsigned int voltage;
 	unsigned long voltage_long;
 	unsigned int ADC;

 	OA1_Config();	// configure OA0 (pins and input/ouput channels)
 	Start_OA1_Slow();	// start op amp (in slow slew rate)
 	delay(150);		// delay to allow op-amp to settle
     ADC = ReadADC_1_5(INCH_13);	//Vref = 1.5V read ADC from INCH channel A13, p4.4 pin 9
     Stop_OA1();

     voltage_long = (unsigned long)ADC;
     voltage_long = voltage_long*1500;
     voltage = voltage_long>>10;

 	return voltage;
 }
