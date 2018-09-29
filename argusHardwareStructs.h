#ifndef ARGUSHARDWARESTRUCTS_H
#define ARGUSHARDWARESTRUCTS_H


/* Structures containing address, scaling, etc. information for 
Argus bias monitor boards.

For biasBoardB cards
First bank A (lower on card, CHSel = F), then bank B (higher on card, CHSel = L)

AH 2014.07.01
*/

// set manual flag for bias or dcm2 system
#define FOUNDLNABIASSYS 1  // 1 for bias, 0 for DCM2
// Version label
#define VER "comap_20180929_b"

// Run with hardware or standalone by commenting/uncommenting #define SIMULATE
//#define SIMULATE
#ifdef SIMULATE
#define I2CSEND1 0
#define I2CSEND2 0
#define I2CSEND3 0
#define I2CREAD1 0
#define I2CREAD2 0
#define BYPASS 1
#else
#define I2CSEND1 (int)I2CSendBuf(address, buffer, 1)
#define I2CSEND2 (int)I2CSendBuf(address, buffer, 2)
#define I2CSEND3 (int)I2CSendBuf(address, buffer, 3)
#define I2CREAD1 (int)I2CReadBuf(address, buffer, 1)
#define I2CREAD2 (int)I2CReadBuf(address, buffer, 2)
#define BYPASS 0
#endif

// Misc parameters
#define CMDDELAY 1        // pause before executing command, in units of 50 ms
#define I2CBUSERRVAL -100 // value to return for I2C bus lock error
#define FREEZEERRVAL -200 // value to return for system freeze violation error
#define WRONGBOX -1000    // value to return if wrong box (bias/dcm2) is addressed

// Hardware parameters -- must match structure definitions in argusHardwareStructs.h!
#define NUM_ELEM(x) (sizeof(x) / sizeof(*(x)))
#define NRX 20      // number of receivers
#define JNRX 19     // number of receivers to read out in JSON commands, 1:JNRX (19)
#define NRXPERBC 4  // number of receivers per bias card
#define NSTAGES 2   // number of amplifier stages in each receiver
#define NBCMP 2     // number of monitor point sets for each bias card
#define NBIASC 5    // number of bias cards (should get from length of address vector, really)
#define NMIX 0      // number of mixers in each receiver

// Software limits for bias settings
#define VDGMAX 1.7    // Max drain-gate voltage [V]
#define VGMIN -0.3    // Min allowable gate voltage [V]
#define VGMAX 0.3     // Max allowable gate voltage [V]
#define VDMIN 0.0     // Min allowable drain voltage [V]
#define VDMAX 1.8     // Max allowable drain voltage [V]
#define VMMIN -.25    // Min allowable mixer voltage [V]
#define VMMAX 5.0     // Max allowable mixer voltage [V]
#define IDMIN 10.0    // Min operating drain current [mA]
#define IDMAX 30.0    // Max operating drain current [mA]
#define IMMIN 0.1     // Min operating mixer current [mA]
#define IMMAX 5.0     // Max operating mixer current [mA]
#define VDEVMAX 0.1   // Max allowable deviation from setpoint, error check [V]

// Startup voltages for gates, drains, mixers
#define VGSTART -0.2   // Gate
#define VDSTART 0.0    // Drain
#define VMSTART 0.0    // Mixer

// Power supply limits
#define MINVCCV 4.75       // Min digital vcc voltage
#define MAXVCCV 5.25       // Max digital vcc voltage
#define MINVDSV MINVCCV    // Max drain supply voltage
#define MAXVDSV MAXVCCV    // Min drain supply voltage
#define MINAMPV 10.0       // Min amplifier supply voltage (absolute)
#define MAXAMPV 15.5       // Max amplifier supply voltage (absolute)

// Over-temperature limits
#define MAXCOLDT 40.   // Maximum nom 20K temperature [K]
#define MAXINTT  80.   // Maximum nom 77K temperature [K]
#define MAXELEXT 60.   // Maximum warm electronics temperature [C]

//Warm IF limits
#define MAXATTEN 31.5  // max attenuation, 5-bit, 1 dB/step, including 0

// I2C bus and switch mapping
#define I2CSWITCH_BP 0x77      // low bus nos. on micro, Argus warm elex chassis backplane
#define I2CSWITCH_SP 0x73      // high bus nos. on micro  (unused hardware)
#define I2CSSB_L 0x74          // high addresses for I2C subsubbus switches
#define I2CSSB_H 0x75          // high addresses for I2C subsubbus switches

//I2C mapping off warm electronics chassis backplane
#define THERM_I2CADDR 0x10 // thermometry
#define I2CSSB_I2CADDR 0x20 // I2C subbusses switches
#define PWCTL_I2CADDR 0x40 //power control
#define BCARD_I2CADDR {0x08, 0x04, 0x02, 0x01, 0x80} // bias card addresses, 0..4
#define ALLBCARD_I2CADDR 0x8f // all bias card addresses, off backplane
#define I2CSWITCH_VMUB 0x00   // Vane and muBox switch addresses (sub-bus)


/****************************************/
// Parameter structure definitions

// Receiver parameter array of structures
struct receiverParams {
  int cardNo;           // bias card number: 0..3 for Argus (four cards)
  int bcChan[NSTAGES];  // channel no. within a bias card: bcChan 0..7
  float LNAsets[NSTAGES*2*(2 + NMIX/2)];     // command values, two per rx: gate, drain, mixer
  float LNAmonPts[NSTAGES*2*(1 + 2 + NMIX)];  // monitor points, two per rx: gate V, drain V I, mixer V I
};

struct biasCardParams {
  float v[8];    // pv, nv, dsv, vcc
};

struct cryostatParams { 
  float cryoTemps[6];     // cryostat temperatures
  float auxInputs[2];     // aux inputs
};

struct calSysParams {
	float adcv[8]; 		// includes angle [V], temperature [C], and motorMeanI[A]
	float minAngle; 	// minimum vane angle [V]
	float maxAngle; 	// maximum vane angle [V]
	float meanCurr; 	// mean motor current [A]
	float maxCurr;      // maximum motor current [A]
	float varCurr;  	// motor current variance [A]
	char *state;     	// system state
};

struct warmIFparams {
	float psv[2];       // power supply voltage monitor points
	float ItotPow[20];   // total power reading for each IF channel
	float QtotPow[20];   // total power reading for each IF channel
	float cardTemp[20]; // card temperature
	char  atten[NRX*NSTAGES];     // quad-I attenuation for each IF channel
};

/*********************************************************************/

// Structures for DACs and ADCs on bias boards

struct chSet {  // set DACs
  char i2c[8];  // board-level i2c addr
  char add[8];  // addr in device
  float sc;     // scale for setting
  float offset; // offset for setting
  char bip;     // bipolar or unipolar, bipolar = 1
};

struct chRead { // read ADCs 
  char i2c[8];  // board-level i2c addr 
  char add[8];  // addr in device
  float sc;     // scale for reading
  float offset; // offset for reading
  char bip;     // bipolar or unipolar, bipolar = 1
};

struct chRead2 { // read ADCs
  char i2c[2];  // board-level i2c addr 
  char add[2];  // addr in device
  float sc;     // scale for reading
  float offset; // offset for reading
  char bip;     // bipolar or unipolar, bipolar = 1
};

/***************************************************************************/
/* DCM2 definitions */
#define NO_DCM2ERR -1     // No DCM2 present
#define DBMSCALE -45.5   // scale factor for power detector conversion to dBm
#define DBMOFFSET 23     // offset value for power detector conversion to dBm (20 + output pad atten)
#define ADCVREF 3.3      // reference voltage for 16-bit ADCs
#define PLLLOCKTHRESH 0.5   // voltage threshold for 4 and 8 GHz PLL lock indication

// I2C subbus and subsubbus switch addresses
#define DCM2_SBADDR 0x77
#define DCM2_SSBADDR 0x73

// subbus switch setting for DCM2 main board peripherals
#define DCM2PERIPH_SBADDR 0x80

// SPI masks for temp sensor, main board BEX
#define SPI_CLK0_M 0x04
#define SPI_DAT0_M 0x01
#define SPI_CSB1_M 0x02   // main board BEX, CS& for P1, temp. sensor
#define DCM2_FP_LED 0x10  // front panel LED
#define DCM2_AMPPOW 0x20  // amplifier power control bit
#define DCM2_P6 0x04      // P6 (unused but available)
#define DCM2_BD_LED 0x80  // main board LED
// BEX I2C address for main board
#define BEX_ADDR0 0x21   // Bus expander addresses for DCM2 main board
// BEX init values for main board
#define BEXCONF0 SPI_DAT0_M  // read P0, write P1..P7 for on-board TCA6408A
#define BEXINIT0 SPI_CSB1_M & ~DCM2_AMPPOW  // init: set CS high, amp low, others X

// SPI masks for downconverter cards
#define QLOG_CS 0x01
#define ILOG_CS 0x02
#define Q_ATTEN_LE 0x04
#define I_ATTEN_LE 0x08
#define BOARD_T_CS 0x10
#define SPI_MISO_M 0x20
#define SPI_MOSI_M 0x40
#define SPI_CLK_M 0x80
// BEX I2C address for downconverter cards
#define BEX_ADDR 0x20       // 0x21 for lab testing with I2C interface, 0x20 for true DCM2 modules
// BEX init values for downconverter cards
#define BEXCONF SPI_MISO_M  // read SPI_MISO_M, write all others on BEX
#define BEXINIT QLOG_CS | ILOG_CS | Q_ATTEN_LE | I_ATTEN_LE | BOARD_T_CS

struct dcm2params {
	BYTE status[NRX]; // status byte
	BYTE attenI[NRX]; // command attenuation, I channel
	BYTE attenQ[NRX]; // command attenuation, Q channel
	float powDetI[NRX]; // nominal power in dBm, I channel
	float powDetQ[NRX]; // nominal power in dBm, Q channel
	float bTemp[NRX];   // board temperature, C
};

/***************************************************************************/
/* Saddlebag definitions */

// I2C subbus and subsubbus switch addresses
#define SB_SBADDR 0x77
#define SB_SSBADDR 0x74

//#define SADDLEBAG_SWADDR {0x08, 0x08, 0x08, 0x08, 0x00}  // for testing, on SSC3/SSD3
//#define SADDLEBAG_SWADDR {0x00, 0x00, 0x00, 0x00, 0x00}  // for testing, no connection
#define SADDLEBAG_SWADDR {0x01, 0x02, 0x04, 0x08, 0x00}  //I2C switch addresses on I2C subbus card
#define SBBEX_ADDR 0x21  // I2C bus address for saddlebag ADCs
#define SBADC_ADDR 0x08  // I2C bus address for saddlebag bus expanders
#define NSBG 4           // ones-base number of saddlebags, used in error checking

struct saddlebagParams {
	float adcv[8];
	BYTE pll;
	BYTE ampPwr;
	char *ampStatus;
};

/***************************************************************************/
/* Vane definitions */
// Use many definitions from saddlebags since interface hardware is identical

//#define VANE_SWADDR 0x08 // for testing, on SSC3/SSD3
//#define VANE_SWADDR 0x00 // for testing, no connection
#define VANE_SWADDR 0x10 // I2C switch address on I2C subbus card, SSC4/SSD4
#define VANEOBSCMD (BYTE)(~0x80 & ~0x20) // P5 low, LED on (low)
#define VANECALCMD (BYTE)(~0x80 & ~0x40) // P6 low, LED on (low)
#define VANEMANCMD (BYTE)(~0x80 & ~0x00) // All Px high except LED on (low)

#define VANESWINGANGLE 180.  // vane swing angle from cal (0 deg) to stow, in degrees
#define VANECALERRANGLE 1.   // maximum absolute error for vane to arrive at cal position
#define VANEOBSERRANGLE 5.   // maximum absolute error for vane to arrive at obs (stow) position
#define STALLERRANG 5.       // minimum absolute angle vane must move to avoid stall designation
#define VANETIMEOUT 10.      // seconds for vane movement; declare timeout if longer
#define VANESTALLTIME 0.5    // seconds; if negligible movement in this time, declare stall

struct vaneParams {
	float adcv[8];
	float vaneAngleDeg;
	BYTE vaneFlag;
	char *vanePos;
};

#endif
