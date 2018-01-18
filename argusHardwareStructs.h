#ifndef ARGUSHARDWARESTRUCTS_H
#define ARGUSHARDWARESTRUCTS_H


/* Structures containing address, scaling, etc. information for 
Argus bias monitor boards.

For biasBoardB cards
First bank A (lower on card, CHSel = F), then bank B (higher on card, CHSel = L)

AH 2014.07.01
*/

// Version label
#define VER "comap_20180118"

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

// Hardware parameters -- must match structure definitions in argusHardwareStructs.h!
#define NUM_ELEM(x) (sizeof(x) / sizeof(*(x)))
#define NRX 20      // number of receivers
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
#define I2CSSB {0x74, 0x75}    // addresses for I2C subsubbus switches
#define I2CSWITCH_WIF {0x00, 0x00} // Warm IF switch addresses (jumper selectable)
#define I2CADC_WIF {0x00, 0x00}    // Warm IF power mon ADC  (jumper selectable)

//I2C mapping off warm electronics chassis backplane
#define THERM_I2CADDR 0x10 // thermometry
#define I2CSSB_I2CADDR 0x20 // I2C subbusses switches
#define PWCTL_I2CADDR 0x40 //power control
//#define BCARD_I2CADDR {0x80, 0x01, 0x02, 0x04, 0x08} // bias card addresses, 0..4
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
#define DBMSCALE -45.5   // scale factor for power detector conversion to dBm
#define DBMOFFSET 20     // offset value for power detector conversion to dBm
#define ADCVREF 3.3      // reference voltage for 16-bit ADCs
#define PLLLOCKTHRESH 0.5   // voltage threshold for 4 and 8 GHz PLL lock indication

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
#define BEX_ADDR 0x20
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
//#define I2CSSB_I2CADDR 0x20 // I2C subbusses switches
//I2C mapping for subbuses from I2CSSB card
#define SADDLEBAG_I2CADDR {0x00, 0x00, 0x00, 0x00}

// order: +12V, -8V, fan 1, fan 2, temp 1, temp 2, temp 3, temp 4
struct saddlebagParams {
	float v[8];
};

#endif
