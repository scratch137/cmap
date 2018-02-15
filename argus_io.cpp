/**
  \file
  \author Andy Harris
  \brief  Low-level Argus hardware input/output routines.

  $Id: argus_io.cpp,v 1.2 2014/06/04 18:51:10 harris Exp $
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <ctype.h>
#include <taskmon.h>
#include <smarttrap.h>

#include "i2cmulti.h"
//Overwrite values in 12cmulti.h
#define I2C_RX_TX_TIMEOUT (5)   // Ticks allowed before timeout of a single byte transmission; default 5
#define I2C_START_TIMEOUT (20)  // Ticks allowed before timeout when attempting start on I2C bus; default 20

#include <pins.h>  // individual pin manipulation

#include "constants.h"

#include "argus.h"

//I2C global setups
BYTE buffer[I2C_MAX_BUF_SIZE];
char I2CInputBuffer[I2C_MAX_BUF_SIZE];   // User created I2C input buffer
char* inbuf = I2CInputBuffer;            // Pointer to user I2C buffer
BYTE address = 0;
int I2CStat = 0;

//Initialize Argus-global state flags and variables
int lnaPwrState = 0;
unsigned char lnaPSlimitsBypass = BYPASS;  // bypass LNA power supply limits when = 1
unsigned char lnaLimitsBypass = BYPASS;    // bypass soft limits on LNA bias when = 1
float gvdiv;
unsigned char i2cBusBusy = 1;              // set to 1 when I2C bus is busy (clears in argus_init())
unsigned int busLockCtr = 0;               // I2C successful bus lock request counter
unsigned int busNoLockCtr = 0;             // I2C unsuccessful bus lock request counter
unsigned char freezeSys =  0;              // freeze system state when = 1
unsigned int freezeCtr = 0;                // freeze request counter
unsigned int thawCtr = 0;                  // thaw request counter
unsigned int freezeErrCtr = 0;             // freeze error counter (access request while frozen)
int i2cState[2];                           // I2C bus SCL (0/1) and SDA (0/2) values, before and after reset

//Pointer defs
struct chSet *chSetPtr; // pointer to structure of form chSet
struct chRead *chReadPtr; // pointer to structure of form chRead

// control bits within power control board PIO
BYTE ctlVDS = 0x01;
BYTE ctlnVamp = 0x02;
BYTE ctlpVamp = 0x04;
BYTE ctlVCC = 0x08;
BYTE ctlVIF = 0x10;
BYTE FPLED = 0x20;
BYTE FPOn = 0x40;
BYTE FPOff = 0x80;

// hardware status words
short unsigned int biasSatus[NRX]; // receiver status word (see argus_rxCheck(void))
short unsigned int powStatus;     // power system status word (see argus_powCheck(void))

/****************************************************************************/
// storage structure and array definitions
/*struct receiverParams {
  int cardNo;           // bias card number: 0..4 for COMAP (five cards)
  int bcChan[NSTAGES];  // channel no. within a bias card: bcChan 0..7
  float LNAsets[2*NSTAGES+NMIX];     // command values: gate, drain, mixer
  float LNAmonPts[NSTAGES+2*NSTAGES+2*NMIX];  // monitor points: gate V, drain V I, mixer V I
};
two receivers per board, each with
  LNAsets   index: vg 0 1; vd 2 3, vm 4 5
  LNAmonPts index: vg 0 1; vd 2 3; id 4 5; vm 6 7; im 8 9
*/
struct receiverParams rxPar[] = {
  //rx0:
  {0, {0, 4}, {0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99}},
  //rx1:
  {0, {1, 5}, {0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99}},
  //rx2:
  {0, {2, 6}, {0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99}},
  //rx3:
  {0, {3, 7}, {0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99}},
  //rx4:
  {1, {0, 4}, {0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99}},
  //rx5:
  {1, {1, 5}, {0, 0, 0, 0},
    {99, 99, 99, 99, 99, 99}},
  //rx6:
  {1, {2, 6}, {0, 0, 0, 0},
	{99, 99, 99, 99, 99, 99}},
  //rx7:
  {1, {3, 7}, {0, 0, 0, 0},
	{99, 99, 99, 99, 99, 99}},
  //rx8:
  {2, {0, 4}, {0, 0, 0, 0},
	{99, 99, 99, 99, 99, 99}},
  //rx9:
  {2, {1, 5}, {0, 0, 0, 0},
	{99, 99, 99, 99, 99, 99}},
  //rx10:
  {2, {2, 6}, {0, 0, 0, 0},
    {99, 99, 99, 99, 99, 99}},
  //rx11:
  {2, {3, 7}, {0, 0, 0, 0},
    {99, 99, 99, 99, 99, 99}},
  //rx12:
  {3, {0, 4}, {0, 0, 0, 0},
    {99, 99, 99, 99, 99, 99}},
  //rx13:
  {3, {1, 5}, {0, 0, 0, 0},
    {99, 99, 99, 99, 99, 99}},
  //rx14:
  {3, {2, 6}, {0, 0, 0, 0},
    {99, 99, 99, 99, 99, 99}},
  //rx15:
  {3, {3, 7}, {0, 0, 0, 0},
    {99, 99, 99, 99, 99, 99}},
  //rx16:
  {4, {0, 4}, {0, 0, 0, 0},
    {99, 99, 99, 99, 99, 99}},
  //rx17:
  {4, {1, 5}, {0, 0, 0, 0},
    {99, 99, 99, 99, 99, 99}},
  //rx18:
  {4, {2, 6}, {0, 0, 0, 0},
    {99, 99, 99, 99, 99, 99}},
  //rx19:
  {4, {3, 7}, {0, 0, 0, 0},
    {99, 99, 99, 99, 99, 99}}
};

/*struct biasCardParams {  // definition in argusHarwareStructs.h
  float v[8];    // two each of pv, nv, dsv, vcc
};
Number of rows matches number of bias cards */
struct biasCardParams bcPar[] = {
		  {{99, 99, 99, 99, 99, 99, 99, 99}},
		  {{99, 99, 99, 99, 99, 99, 99, 99}},
		  {{99, 99, 99, 99, 99, 99, 99, 99}},
		  {{99, 99, 99, 99, 99, 99, 99, 99}},
		  {{99, 99, 99, 99, 99, 99, 99, 99}}
};

/*struct cryostatParams {  // definition in argusHarwareStructs.h
  float cryoTemps[6];     // cryostat temperatures
  float auxInputs[2];     // aux inputs
}; */
struct cryostatParams cryoPar = {
	  {99, 99, 99, 99, 99, 99}, {99, 99},
};

// vds, -15V, +15, vcc, cal sys, cold if in, cold if out, cold if curr, chassis temp
float pwrCtrlPar[] = {99, 99, 99, 99, 99, 99, 99, 99, 99};


/*struct calSysParams {
	float adcv[8]; 		// includes angle [V], temperature [C], and motorMeanI[A]
	float minAngle; 	// minimum vane angle [V]
	float maxAngle; 	// maximum vane angle [V]
	float meanCurr; 	// mean motor current [A]
	float maxCurr;      // maximum motor current [A]
	float varCurr;  	// motor current variance [A]
	char state[15];     // system state
};*/
struct calSysParams calSysPar = {
	{99., 99., 99., 99., 99., 99., 99., 99.}, 99., 99., 99., 99., 99., " "
};

/**************************/
// DACs

// drain voltage setups
struct chSet vdSet = {
		{0x40, 0x40, 0x40, 0x40, 0x31, 0x31, 0x31, 0x31},
		{0x31, 0x37, 0x32, 0x36, 0x31, 0x37, 0x32, 0x36},
		1.0, 0.0, 0};

// gate voltage or servo current setups
struct chSet vgSet = {
		{0x41, 0x41, 0x41, 0x41, 0x32, 0x32, 0x32, 0x32},
		{0x31, 0x37, 0x33, 0x36, 0x31, 0x37, 0x33, 0x36},
		0.1470, 0.0, 1};

// mixer voltage setups
struct chSet vmSet = {
		{0x40, 0x40, 0x41, 0x41, 0x31, 0x31, 0x32, 0x32},
		{0x34, 0x33, 0x30, 0x32, 0x34, 0x33, 0x30, 0x32},
		0.42824, -0.27676, 1};

// offset
struct chSet voSet = {
		{0x41, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
		{0x35, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
		0.001, 2.047, 1};  // offset in mV
/**************************/
//ADCs

// drain voltage monitor points
struct chRead vdRead = {
		{0x19, 0x19, 0x19, 0x19, 0x08, 0x08, 0x08, 0x08},
		{0xf8, 0xa8, 0xb8, 0xe8, 0xf8, 0xa8, 0xb8, 0xe8},
		1.0, 0.0, 0};  // no scaling needed for sc

// drain current monitor points
struct chRead idRead = {
		{0x18, 0x18, 0x18, 0x18, 0x09, 0x09, 0x09, 0x09},
		{0xc0, 0xf0, 0x80, 0xb0, 0xc0, 0xf0, 0x80, 0xb0},
		16.75, 34.15, 1};  // offset is slope*2.048

// gate voltage monitor point
struct chRead vgRead = {
		{0x18, 0x18, 0x18, 0x18, 0x09, 0x09, 0x09, 0x09},
		{0x90, 0xe0, 0xd0, 0xa0, 0x90, 0xe0, 0xd0, 0xa0},
		-6.8, 0.0, 1};

// mixer voltage monitor points
struct chRead vmRead = {
		{0x0b, 0x0b, 0x0b, 0x0b, 0x0a, 0x0a, 0x0a, 0x0a},
		{0xf0, 0x80, 0xa0, 0xd0, 0xf0, 0x80, 0xa0, 0xd0},
		1.564, 2.179, 1};  // with 4.7k shunt at ADC input

// mixer current monitor points
struct chRead imRead = {
		{0x0b, 0x0b, 0x0b, 0x0b, 0x0a, 0x0a, 0x0a, 0x0a},
		{0xb0, 0xc0, 0xe0, 0x90, 0xb0, 0xc0, 0xe0, 0x90},
		2.439, 0.0, 1};

// amplifier positive voltage monitor point on bias card
struct chRead2 pvRead = {
		{0x19, 0x08},
		{0xc8, 0xc8},
		4.727, 0.0, 0};

// amplifier negative voltage monitor point on bias card
struct chRead2 nvRead = {
		{0x19, 0x08},
		{0xd8, 0xd8},
		-4.545, 0.0, 0};

// drains supply voltage monitor point on bias card
struct chRead2 vdsRead = {
		{0x19, 0x08},
		{0x88, 0x88},
		2.0, 0.0, 0};

// vcc supply voltage monitor point on bias card
struct chRead2 vccRead = {
		{0x19, 0x08},
		{0x98, 0x98},
		2.0, 0.0, 0};

// power control board monitor points (pcRead.add used in other locations too)
struct chRead pcRead = {
		{0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08},
		{0x88, 0xc8, 0x98, 0xd8, 0xa8, 0xe8, 0xb8, 0xf8},
		1, 0, 0};

// thermometry board monitor points
struct chRead thRead = {
		{0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08},
		{0xf8, 0xb8, 0xe8, 0x98, 0xc8, 0x88, 0xd8, 0xa8},
		1, 0, 0};

/************************************************************************/

/**
  \brief Convert voltage to DAC word

  Function converts voltage to DAC word.  Sets to max or min value for 10-bit DAC if 
  input value is out of range. 

  \param  v   input value (float)
  \param  sc  multiplicitive scaling factor (float)
  \param  offs additive offset (float)
  \param  bip bipolar = 1, unipolar = 0 (char)
  \return Zero on success, else -1.
*/
/*------------------------------------------------------------------
  Convert voltage to DAC word
 ------------------------------------------------------------------*/
unsigned short int v2dac(float v, float sc, float offs, char bip) {

	unsigned int dacw;
	if (bip) {
		dacw = ((v+offs)*sc*65535/4.096 + 0x7fff);
		if (dacw & 0xffff0000) {  // check for values out of range, clip to in-range
			if (v > 0) dacw = 0x0000ffff;
			else dacw = 0x00000000;
		}
	}
	else {
		dacw = ((v+offs)*sc*65535/4.096);
		if (v < 0) dacw = 0x00000000; // if below zero, set to zero
		if (dacw & 0xffff0000) dacw = 0x0000ffff;  // if above max, clip to max
	}
	return (unsigned short int)dacw;
}

/****************************************************************************************/

/**
  \brief Convert ADC word to voltage

  Function converts ADC word to volage in mV.

  \param  adcw  ADC word (short int)
  \param  sc    Multiplicitive scaling factor (float)
  \param  offs  Additive offset (float)
  \param  bip   Bipolar = 1, unipolar = 0 (char)
  \return Zero on success, else -1.
*/
/*------------------------------------------------------------------
  Convert ADC word to voltage
 ------------------------------------------------------------------*/
float adc2v(short int adcw, float sc, float offs, char bip) {

  float v;
    v = adcw*sc*4.096/65535 + offs;
    return v;
}


/********************************************************************/
/**
  \brief Set LNA DAC.

  This command sets the DACs for LNA and mixer bias voltages

  \param  i   terminal: g, d, or m for gate, drain, and mixer (char).
  \param  m   mth receiver.
  \param  n   nth stage within a receiver.
  \param  v   value in V
  \param busy set to 0 to release I2C bus, 1 to retain (1 for loops)
  \return Zero on success, -1 for invalid selection, -10 if bias card power is not on,
              else number of I2C read fails.
*/
int argus_setLNAbias(char *term, int m, int n, float v, unsigned char busyOverride)
{
	if (!noDCM2) return WRONGBOX;

	unsigned short int dacw;
	short I2CStat;
	int baseAdd; //base address offset for gate, drain, mixer
	char bcard_i2caddr[] = BCARD_I2CADDR;//{0x01, 0x01, 0x01, 0x01, 0xff}; //BCARD_I2CADDR;
	float vDiv;  // voltage divider ratio,  vDiv <= 1

	// return if the LNA boards are not powered
	if (!lnaPwrState) return (-10);

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

	// check that I2C bus is available, else return
	if (i2cBusBusy && !busyOverride) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

    // Write to device
	// first set I2C bus switch for bias card in backplane
    address = I2CSWITCH_BP;  // bias cards are in Argus backplane
   	buffer[0] = bcard_i2caddr[rxPar[m].cardNo];  // select bias card
	I2CStat = I2CSEND1;    // set i2c bus switch to talk to card

	// then check that voltage is within limits, set address internal to card and channel,
	// convert voltage, send chip i2c address on card, internal address for channel,
	// convert voltage to word for DAC
	if (strcmp(term, "g") == 0) {
		if (lnaLimitsBypass == 0) {   // bypass soft limits on LNA bias when = 1
			if (v > VGMAX) v = VGMAX;
			if (v < VGMIN) v = VGMIN;
			if (rxPar[m].LNAsets[n+NSTAGES] - v > VDGMAX) v = rxPar[m].LNAsets[n+NSTAGES] - VDGMAX;
		}
		v = v/gvdiv;  // convert from gate voltage to bias card output voltage
		address = vgSet.i2c[rxPar[m].bcChan[n]];
		buffer[0] = vgSet.add[rxPar[m].bcChan[n]];
		dacw = v2dac(v, vgSet.sc, vgSet.offset, vgSet.bip);
		baseAdd = 0;
		vDiv = gvdiv;
	} else if (strcmp(term, "d") == 0) {
		if (lnaLimitsBypass == 0) {   // bypass soft limits on LNA bias when = 1
			if (v > VDMAX) v = VDMAX;
			if (v < VDMIN) v = VDMIN;
			if (v - rxPar[m].LNAsets[n] > VDGMAX) v = rxPar[m].LNAsets[n] + VDGMAX;
		} else {
			if (v < 0) v = 0;  // hardware limit
		}
		address = vdSet.i2c[rxPar[m].bcChan[n]];
		buffer[0] = vdSet.add[rxPar[m].bcChan[n]];
		dacw = v2dac(v, vdSet.sc, vdSet.offset, vdSet.bip);
		baseAdd = 2;
		vDiv = 1.;
	} else if (strcmp(term, "m") == 0) {
		if (lnaLimitsBypass == 0) {   // bypass soft limits on LNA bias when = 1
			if (v > VMMAX) v = VMMAX;
			if (v < VMMIN) v = VMMIN;
		}
		address = vmSet.i2c[rxPar[m].bcChan[n]];
		buffer[0] = vmSet.add[rxPar[m].bcChan[n]];
		dacw = v2dac(v, vmSet.sc, vmSet.offset, vmSet.bip);
		baseAdd = 4;
		vDiv = 1.;
	} else {
		// Disconnect I2C sub-bus
		address = I2CSWITCH_BP;
		buffer[0] = 0;
		i2cBusBusy = 0; // release I2C bus
		I2CStat = I2CSEND1;
		return -1;
	}
	// write to DAC
	buffer[2] = BYTE(dacw);
	buffer[1] = BYTE(dacw>>8);
	I2CStat = I2CSEND3;    // send set command
	// write set value v into structure
	if (I2CStat==0) {
        rxPar[m].LNAsets[n+baseAdd] = v*vDiv;
	}
	else {
		if (lnaPSlimitsBypass == 1) rxPar[m].LNAsets[n+baseAdd] = v*vDiv;
		else rxPar[m].LNAsets[n+baseAdd] = 99.;
	}
	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = busyOverride;

	return I2CStat;
}


/****************************************************************************************/

/**
  \brief Set all LNA bias voltages

  This command sets all LNA bias voltages to a common value.  Useful
  for initialization.

  This command does not set i2cBusBusy semaphore; that should be done outside
  if needed

  \param  inp  Select input: char g, d, m for gate, drain, mixer.
  \param  v    Voltage [V].
  \return 0 on success; -1 for invalid request; -10 if LNA boards have no power,
               else a number giving the number of failed I2C writes.
  */

int argus_setAllBias(char *inp, float v, unsigned char busyOverride){

	if (!noDCM2) return WRONGBOX;

	int i, j, stat=0;

    // return if the LNA boards are not powered
	if (!lnaPwrState) return (-10);

    // check that I2C bus is available, else return
	if (i2cBusBusy && !busyOverride) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

 	if (strcmp(inp, "g") == 0 || strcmp(inp, "d") == 0) {
		for (i=0; i<NRX; i++) {
			for (j=0 ; j<NSTAGES ; j++) {
				stat += argus_setLNAbias(inp, i, j, v, 1);
			}
		}
	}
	else if (strcmp(inp, "m") == 0 ) {
		if (NMIX > 0) {
			for (i=0; i<NRX; i++) {
				for (j=0 ; j<NMIX ; j++) {
					stat += argus_setLNAbias("m", i, j, v, 1);
				}
			}
		}
	} else {
		i2cBusBusy = 0; // release I2C bus
		return -1;
	}

    // release I2C bus
	i2cBusBusy = busyOverride;

	return stat;
}


/****************************************************************************************/
/**
  \brief Read LNA monitor points.

  This command reads the LNA voltage monitor points for all receivers.  Results are put in
  the receiver parameter structure, with 99 the value for an unsuccessful read.

  \return Zero on success, else number of failed I2C writes.
*/

int argus_readLNAbiasADCs(char *sw)
{
	if (!noDCM2) return WRONGBOX;

	int n, m, mmax; //index counters
	int baseAddr;  // offset in values vector
	unsigned short int rawu;
	short int rawb;
	float vDivRatio;
	BYTE bcard_i2caddr[] = BCARD_I2CADDR;
	char idFlag = 0; // set to 1 for drain shunt current correction

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

// check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	// set up for particular monitor point: vg, vd, id, vm, im
	// baseAddr offsets correspond to offsets in receiver parameters structure
	if (strcmp(sw, "vg") == 0) {
		chReadPtr = &vgRead;
		baseAddr = 0;
		mmax = NSTAGES;
		vDivRatio = gvdiv;
	} else if (strcmp(sw, "vd") == 0) {
		chReadPtr = &vdRead;
		baseAddr = 2;
		mmax = NSTAGES;
		vDivRatio = 1.;
	} else if (strcmp(sw, "id") == 0) {
		chReadPtr = &idRead;
		baseAddr = 4;
		mmax = NSTAGES;
		vDivRatio = 1.;
		idFlag = 1;  // set to 1 to make shunt current correction
	} else if (strcmp(sw, "vm") == 0) {
		if (NMIX == 0) {
			i2cBusBusy = 0; // release I2C bus
			return -1;
		}
		chReadPtr = &vmRead;
		baseAddr = 6;
		mmax = NMIX;
		vDivRatio = 1.;
	} else if (strcmp(sw, "im") == 0) {
		if (NMIX == 0) {
			i2cBusBusy = 0; // release I2C bus
			return -1;
		}
		chReadPtr = &imRead;
		baseAddr = 8;
		mmax = NMIX;
		vDivRatio = 1.;
	} else {
		i2cBusBusy = 0; // release I2C bus
		return -1;
	}

	// loop over receivers
	if (lnaPwrState) {
		for (n = 0 ; n < NRX; n++) {
			// set I2C bus switch for correct bias card
			address = I2CSWITCH_BP;  // select backplane
			buffer[0] = bcard_i2caddr[rxPar[n].cardNo];  // bias card address in backplane
			I2CStat = I2CSEND1;    // set i2c bus switch to talk to card
			// loop over stages
			for (m = 0 ;  m < mmax ; m++) {
				address = chReadPtr->i2c[rxPar[n].bcChan[m]];    // chip i2c address on card
				buffer[0] = chReadPtr->add[rxPar[n].bcChan[m]];  // internal address for channel
				I2CStat = I2CSEND1;        // send command for conversion
				I2CREAD2;   // read device buffer back
				//I2CStat += I2CREAD2 - 2;   // read device buffer back, accumulate error flag
				// write result to structure; two cases for bipolar and unipolar ADC settings
				// if idFlag correct correct I_D for current from 1k shunt resistor (units are V, mA)
				if (I2CStat == 0) {
					if (chReadPtr->bip == 1) {
						rawb = (short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
						rxPar[n].LNAmonPts[m+baseAddr] = rawb*chReadPtr->sc*4.096/65535*vDivRatio + chReadPtr->offset;
						if (idFlag) rxPar[n].LNAmonPts[m+baseAddr] = rxPar[n].LNAmonPts[m+baseAddr] - rxPar[n].LNAmonPts[m+2];
					} else {
						rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
						rxPar[n].LNAmonPts[m+baseAddr] = rawu*chReadPtr->sc*4.096/65535*vDivRatio + chReadPtr->offset;
						if (idFlag) rxPar[n].LNAmonPts[m+baseAddr] = rxPar[n].LNAmonPts[m+baseAddr] - rxPar[n].LNAmonPts[m+2];
					}
				} else rxPar[n].LNAmonPts[m+baseAddr] = 99;
			}
		}
	} else {
		// set value to no-measurement if power is not on
		for (n = 0 ; n < NRX; n++) {
			for (m = 0 ;  m < mmax ; m++) {
				rxPar[n].LNAmonPts[m+baseAddr] = 99;
			}
		}
	}

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return I2CStat;
}
/****************************************************************************************/
/**
  \brief Read LNA bias card power monitors.

  This command reads the power supply voltage monitor points on a bias card.  Results are put in
  the bias card structure, with 99 the value for an unsuccessful read.

  \return Zero on success, else number of failed I2C writes.
*/

int argus_readBCpsV(void)
{
	if (!noDCM2) return WRONGBOX;

	struct chRead2 *bcPsVptr[] = {&pvRead, &nvRead, &vdsRead, &vccRead};  // pointers to bias card struct
	BYTE bcard_i2caddr[] = BCARD_I2CADDR;
	int k, n, m;
	BYTE writeErrs = 0;
	short baseAddr[] = {0, 2, 4, 6, 8};  // vector has same length as number of cards, NBIASC
	unsigned short int rawu;
	short int rawb;
	// offsets correspond to offsets in results vector

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

    // check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	if (lnaPwrState) {
		for (k = 0; k < NBIASC; k++) {  // loop over cards
			// set I2C bus switch for correct bias card
			address = I2CSWITCH_BP;  // select backplane
			buffer[0] = bcard_i2caddr[k];  // bias card address in backplane
			writeErrs = I2CSEND1;    // set i2c bus switch to talk to card
			for (m = 0; m < 4; m++) { // loop over monitor points
				for (n = 0; n < 2; n++) {  // loop over bias points
					address = bcPsVptr[m]->i2c[n];    // chip i2c address on card
					buffer[0] = bcPsVptr[m]->add[n];  // internal address for channel
						I2CStat = I2CSEND1;    // send command for conversion
						I2CREAD2;   // read device buffer back, accumulate error flag
					    // write result to structure; two cases for bipolar and unipolar ADC settings
						if (I2CStat == 0) {
							if (bcPsVptr[m]->bip == 1) {
								rawb = (short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
								bcPar[k].v[n+baseAddr[m]] = rawb*bcPsVptr[m]->sc*4.096/65535 + bcPsVptr[m]->offset;
							} else {
								rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
								bcPar[k].v[n+baseAddr[m]] = rawu*bcPsVptr[m]->sc*4.096/65535 + bcPsVptr[m]->offset;
							}
						} else {
							bcPar[k].v[n+baseAddr[m]] = 99.;
							writeErrs += 1;
						}
				}
			}
		}
	} else {
		// set value to no-measurement if power is not on
		for (k = 0; k < NBIASC; k++) {  // loop over cards
			for (m = 0; m < 4; m++) { // loop over monitor points
				for (n = 0; n < 2; n++) {  // loop over bias points
					bcPar[k].v[n+baseAddr[m]] = 99.;
				}
			}
		}
		writeErrs = 0;
	}

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return writeErrs;
}

/**************************************************************************************/

/**
  \brief Read power control ADC.

  This command reads the ADC on the power control card

  \return Zero on success, else number of failed I2C writes.
*/
int argus_readPwrADCs(void)
{

	if (!noDCM2) return WRONGBOX;

	short i;
	unsigned short int rawu;
	static float offset[8] = {0};
	static float scale[8] = {2., -4.545, 4.727, 2., 7.818, 2., 2., 1.};
	//float scale[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // for calibration
	// vds, -15, +15, vcc, vcal, vif, swvif, iif

    if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
    i2cBusBusy = 1;
    busLockCtr += 1;

    // Write to device
	// first set I2C bus switch
    address = I2CSWITCH_BP;  // select Argus backplane
    buffer[0] = PWCTL_I2CADDR;  // select power control card
	I2CStat = I2CSEND1;    // set i2c bus switch to talk to card

	//Read all channels of ADC
	for (i = 0 ;  i < 8 ; i++) {
		address = (BYTE)0x08;    // ADC device address on I2C bus
		buffer[0] = (BYTE)pcRead.add[i];        // internal address for channel
		I2CStat = I2CSEND1;  // send command for conversion
		I2CREAD2;  // read device buffer back
		if (I2CStat == 0) {
			rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
			pwrCtrlPar[i] = rawu*scale[i]*4.096/65535 + offset[i];
		}
		else pwrCtrlPar[i] = 99.;  // error condition
	}

	// Read thermometer chip on power control board
	short int rawtemp;
	address = 0x4f;
	buffer[0] = 0x00;
	I2CStat += I2CSEND1;
	I2CREAD2;
	rawtemp = (short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
	if (I2CStat == 0) {
		pwrCtrlPar[8] = (float)rawtemp/256.;
	} else {
		pwrCtrlPar[8] = 999.;
	}

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return I2CStat;
}

/**************************************************************************************/
/**
  \brief Read LNA power control PIO buffer.

  Function to query LNA power control PIO buffer.
*/

int argus_lnaPowerPIO(void)
{
	if (!noDCM2) return WRONGBOX;

	unsigned char pioState;

    // check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	// set I2C sub-bus switch for power control card in backplane
	address = I2CSWITCH_BP;
	buffer[0] = PWCTL_I2CADDR;
	I2CStat = I2CSEND1;

	// Read port register to establish present state, store byte for output
	address = 0x21;  // PIO I2C address
	buffer[0] = 0x00;
	I2CStat += I2CSEND1;
	I2CREAD1;
	pioState = buffer[0];

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return (pioState);
}


/**************************************************************************************/
/**
  \brief LNA power control.

  This command turns the LNA power on and off in a safe way.  Checks for power supplies
  in range for ON, but OFF executes regardless of power supply values.

  \param  state     Power state (1=on, else off).
  \return Zero on success; else a number giving the number of failed I2C writes or,
          for power supplies out of range, in order of first failure:
            9995 for Vcc
            9996 for -Vamp
            9997 for +Vamp
            9998 for VDS
          */
int argus_lnaPower(short state)
{
	if (!noDCM2) return WRONGBOX;

	BYTE pioState;

	I2CStat = argus_readPwrADCs();
	// get power supply voltages: returns vds, -15, +15, vcc, vcal, vif, swvif, iif, tamb

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

	// check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	if (state == 1) {  // check power supply voltages before on, but skip check for off
		if (lnaPSlimitsBypass != 1) {
			if (pwrCtrlPar[3] < MINVCCV || pwrCtrlPar[3] > MAXVCCV) {
				// Disconnect I2C sub-bus
				address = I2CSWITCH_BP;
				buffer[0] = 0;
				I2CStat = I2CSEND1;
				i2cBusBusy = 0;  // release I2C bus
				return 9995;   //VCC
			}
			if (pwrCtrlPar[1] < -MAXAMPV || pwrCtrlPar[1] > -MINAMPV) {
				// Disconnect I2C sub-bus
				address = I2CSWITCH_BP;
				buffer[0] = 0;
				I2CStat = I2CSEND1;
				i2cBusBusy = 0;  // release I2C bus
				return 9996; //-15V
			}
			if (pwrCtrlPar[2] < MINAMPV || pwrCtrlPar[2] > MAXAMPV) {
				// Disconnect I2C sub-bus
				address = I2CSWITCH_BP;
				buffer[0] = 0;
				I2CStat = I2CSEND1;
				i2cBusBusy = 0;  // release I2C bus
				return 9997;   //+15
			}
			if (pwrCtrlPar[0] < MINVDSV || pwrCtrlPar[0] > MAXVDSV) {
				// Disconnect I2C sub-bus
				address = I2CSWITCH_BP;
				buffer[0] = 0;
				I2CStat = I2CSEND1;
				i2cBusBusy = 0;  // release I2C bus
				return 9996;   //VDS
			}
		}
	}

	// set I2C bus switch for power control card in backplane
	address = I2CSWITCH_BP;
	buffer[0] = PWCTL_I2CADDR;
	I2CStat = I2CSEND1;

	// Read port register to establish present state;
	// store byte for later modification
	address = 0x21;  // PIO I2C address
	buffer[0] = 0x00;
	I2CStat += I2CSEND1;
	I2CREAD1;
	pioState = buffer[0];

	// control power
	if (state==1 && lnaPwrState==0) {
		 // turn on VCC (digital) and allow it to stabilize
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState | ctlVCC;
		I2CStat += I2CSEND2;
		OSTimeDly( TICKS_PER_SECOND * 1 );
		lnaPwrState = 1;  // set state flag

		// initialize DAC values, then return to power control board
		argus_setAllBias("g", VGSTART, 1);
		argus_setAllBias("d", VDSTART, 1);
		argus_setAllBias("m", VMSTART, 1);
		address = I2CSWITCH_BP;
		buffer[0] = PWCTL_I2CADDR;
		I2CStat = I2CSEND1;

		// turn on +/- Vamp (for gates), allow stabilization time
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState | ctlpVamp | ctlnVamp;
		I2CStat += I2CSEND2;
		OSTimeDly( TICKS_PER_SECOND * 1 );

		// turn on VDS (for drains)
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState | ctlVDS;
		I2CStat += I2CSEND2;

		// turn on LED
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState & 0xff^FPLED;
		I2CStat += I2CSEND2;

	}
	else if (state==0 && lnaPwrState==1) {
		// set power supplies to safe voltages for switching
		argus_setAllBias("g", VGSTART, 1);
		argus_setAllBias("d", VDSTART, 1);
		argus_setAllBias("m", VMSTART, 1);

		// set I2C bus switch for power control card in backplane
		address = I2CSWITCH_BP;
		buffer[0] = PWCTL_I2CADDR;
		I2CStat = I2CSEND1;

		// turn off VDS (drains)
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState & 0xff^ctlVDS;
		I2CStat += I2CSEND2;
		OSTimeDly( TICKS_PER_SECOND * 0.5 );

		// turn off +/- Vamp (gates)
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState & 0xff^(ctlpVamp | ctlnVamp);
		I2CStat += I2CSEND2;
		OSTimeDly( TICKS_PER_SECOND * 0.5 );  // wait

		// turn off VCC (digital)
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState & 0xff^ctlVCC;
		I2CStat += I2CSEND2;
		lnaPwrState = 0;  // clear state flag

		// turn off LED
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState | FPLED;
		I2CStat += I2CSEND2;

	}

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return (I2CStat);
}


/****************************************************************************************/

/**
  \brief Covert cryo diode voltage to temperature

  This function converts voltage to temperature for Lakeshore 670-series cryo diodes. Coefficients
  and method from Lakeshore diode curve writeup.

  \param  v  The voltage across a Lakeshore 670-series diode
  \return    The corresponding temperature, or obvious out-of-range values for over- or under-range.
*/
float v2t_670(float v) {
 
  short int i;
  float x, temp, tc[12];

  // strutures with Chebyshev coefficients, different temperatures
  struct coeffs {
    float vl;    // low voltage
    float vh;    // high voltage
    float a[12]; // coeffs (padded with 0 if needed)
  };
  struct coeffs *ptc; // pointer to structure of type coeffs
  // coefficients for 2K to 12K
  static struct coeffs t1 = {1.294390, 1.680000, {6.429274, -7.514262,
			     -0.725882, -1.117846, -0.562041, -0.360239, 
			     -0.229751, -0.135713, -0.068203, -0.029755, 
			     0., 0.}};
  // coefficients for 12K to 24.5K
  static struct coeffs t2 = {1.11230, 1.38373, {17.244846, -7.964373,
			     0.625343, -0.105068, 0.292196, -0.344492,
			     0.271670, -0.151722, 0.121320, -0.035566,
			     0.045966, 0.}};
  // coefficients for 24.5 to 100 K
  static struct coeffs t3 = {0.909416, 1.122751, {82.017868, -59.064244,
			     -1.356615, 1.055396, 0.837341, 0.431875, 
			     0.440840, -0.061588, 0.209414, -0.120882,
			     0.055734, -0.035974}};
  // coefficients for 100K to 475K
  static struct coeffs t4 = {0.07000, 0.99799, {306.592351, -205.393808,
			     -4.695680, -2.031603, -0.071792, -0.437682,
			     0.176352, -0.182516, 0.064687, -0.027019, 
			     0.010019, 0.}};

  // pick correct set of coefficients; first out-of-range tests
  if (v > 1.680) return(-999);  // overvoltage error
  if (v < 0.070) return(999);   // undervoltage error
  if (v >= 1.339) ptc = &t1;       // for 2-12K
  else if (v >= 1.118) ptc = &t2;  // for 12-24.5K
  else if (v >= 0.954) ptc = &t3;  // for 24.5-100K
  else ptc = &t4;                  // for 100-475K

  // compute temperature with Chebyshev recursion
  x = ((v - ptc->vl) - (ptc->vh - v))/(ptc->vh - ptc->vl);
  tc[0] = 1.;
  tc[1] = x;
  temp = ptc->a[0] + ptc->a[1]*x;
  for (i=2; i<12; i++) {
    tc[i] = 2*x*tc[i-1] - tc[i-2];
    temp = temp + ptc->a[i]*tc[i];
  }


  return(temp);
}

/****************************************************************************************/

/**
  \brief Read bias card power monitor points.

  This command reads the supply monitor points on the bias cards.

  \return Zero on success, else number of failed I2C writes.
*/
int argus_readThermADCs(void)
{
	if (!noDCM2) return WRONGBOX;

	short i;
	unsigned short int rawu;
	static float offset[8] = {0};
    static float scale[8] = {0.4439, 0.4439, 0.4439, 0.4439, 0.4439, 0.4439, 3.7, 3.7};
	//float scale[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // for calibration

    // check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

    // Write to device
	// first set I2C bus switch
    address = I2CSWITCH_BP;  // select Argus backplane
    buffer[0] = THERM_I2CADDR;  // select thermometry card
	I2CStat = I2CSEND1;    // set i2c bus switch to talk to card

	//Read thermometry channels of ADC
	for (i = 0 ;  i < 6 ; i++) {
		address = thRead.i2c[i];    // ADC device address on I2C bus
		buffer[0] = thRead.add[i];  // internal address for channel
		I2CStat = I2CSEND1;  // send command for conversion
		I2CREAD2;  // read device buffer back
		if (I2CStat == 0) {
			rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
			cryoPar.cryoTemps[i] = rawu*scale[i]*4.096/65535 + offset[i]; // voltage
			cryoPar.cryoTemps[i] = v2t_670(cryoPar.cryoTemps[i]);         // temperature
			//cryoPar.cryoTemps[i] = cryoPar.cryoTemps[i]*100.;           // for testing: 1.234 V -> 123.4 K
		}
		else cryoPar.cryoTemps[i] = 99;  // error condition
	}

	//Read thermometry card aux input channels of ADC
	for (i = 6 ;  i < 8 ; i++) {
		address = thRead.i2c[i];    // ADC device address on I2C bus
		buffer[0] = thRead.add[i];  // internal address for channel
		I2CStat = I2CSEND1;  // send command for conversion
		I2CREAD2;  // read device buffer back
		if (I2CStat == 0) {
			rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
			cryoPar.auxInputs[i-6] = rawu*scale[i]*4.096/65535 + offset[i];
		}
		else cryoPar.auxInputs[i-6] = 99;  // error condition
	}

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return I2CStat;
}

/****************************************************************************************/
/**
  \brief Set LNA params from flash.

  This command sets the LNA bias values to preset values stored in flash.

  \param  *flash A pointer to a structure of type flash_t.
  \return Zero on success, else number of failed I2C writes.
*/
int argus_LNApresets(const flash_t *flash)
{
// Storage for Argus is is g1, g2, d1, d2, m1, m2, g3, g4 ... m31, m32
	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

	// check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	// Data written in control.cpp, approx line 405
	short i, j, k;
	int rtn = 0;

	for (i=0; i<NRX; i++) {
		for (j=0; j<NSTAGES; j++){  // set drains first, then gates
			k = i*NSTAGES+j;
			// need to adjust gates first to keep from running into high-v limit on drains
			rtn += argus_setLNAbias("g", i, j, flash->lnaGsets[k], 1);
			//OSTimeDly(1);   // insert for settling?
			rtn += argus_setLNAbias("d", i, j, flash->lnaDsets[k], 1);
		}
	}

	// release I2C bus
	i2cBusBusy = 0;

	return rtn;
}

/*******************************************************************/
/**
  \brief Clear I2C bus.

  Clear lock bit and open switches on main I2C bus

  \return         zero for success, or encoded number of failed I2C writes.
*/
int argus_clearBus(void)
{

	// first check the state of the I2C bus:
	i2cState[0] = 0;
	// set pins for GPIO to read
	J2[42].function (PINJ2_42_GPIO);  // configure SCL as GPIO
	J2[39].function (PINJ2_39_GPIO);  // configure SDA as GPIO
	// read pins for BOOL values
	if (J2[42]) i2cState[0] = 1;  // SCL
	if (J2[39]) i2cState[0] += 2; // SDA
	// set pins for I2C again
	J2[42].function (PINJ2_42_SCL);  // configure SCL as GPIO
	J2[39].function (PINJ2_39_SDA);  // configure SDA as GPIO

	// hardware reset, main sub-bus switch
	OSTimeDly(1); // needs a delay to make the pulse the right width, otherwise lots of jitter
	J2[28].set();
	OSTimeDly(1);
	J2[28].clr();

	// wrap up with check of the state of the I2C bus:
	i2cState[1] = 0;
	// set pins for GPIO to read
	J2[42].function (PINJ2_42_GPIO);  // configure SCL as GPIO
	J2[39].function (PINJ2_39_GPIO);  // configure SDA as GPIO
	// read pins for BOOL values
	if (J2[42]) i2cState[1] = 1;  // SCL
	if (J2[39]) i2cState[1] += 2; // SDA
	// set pins for I2C again
	J2[42].function (PINJ2_42_SCL);  // configure SCL as GPIO
	J2[39].function (PINJ2_39_SDA);  // configure SDA as GPIO

    // clear busy bit
	i2cBusBusy = 0;

	return I2CStat;
}

/**************************************************************************************/
/**
  \brief Read all Argus system ADCs.

  Read all Argus system ADCs.  This fills monitor data point structures with
  up to date values.

  \return Zero.
*/
/*------------------------------------------------------------------
  Read all ADCs
 ------------------------------------------------------------------*/
int argus_readAllSystemADCs(void)
{
	if (!noDCM2) return WRONGBOX;

	int I2CStat = 0;
	if (argus_readPwrADCs() == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readBCpsV() == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readThermADCs() == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readLNAbiasADCs("vg") == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readLNAbiasADCs("vd") == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readLNAbiasADCs("id") == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readLNAbiasADCs("vm") == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readLNAbiasADCs("im") == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	return I2CStat;
}

/****************************************************************************************/
/**
  \brief Argus LNA bias check

  This command checks request vs. measured bias for LNAs and mixers.

  Within each 16-bit word:
    First stage LNA status in MSN (most significant nibble)
    Second stage LNA in next nibble
    Mixer one in next nibble
    Mixer two in LSN

  Within each nibble:
  For LNAs, V_G error in LSB, then V_D, then I_D
    1 means gate voltage error
    4 means drain current error, but drain voltage ok
    6 means both drain voltage and current errors
  For mixers, V in LSB, I in next
    1 means mixer voltage error
    2 means mixer current error
	3 means both error

  \return Zero on no errors, else summary with all words or-ed together.
*/
int argus_biasCheck(void)
{
	if (!noDCM2) return WRONGBOX;

	int i;  // loop counter
	int ret = 0;  // return value

	// loop over channels
	// initialize, then compare with set points
	for (i=0; i<NRX; i++) {
		biasSatus[i] = 0;
		// drain i
		if ( (rxPar[i].LNAmonPts[4] < IDMIN) || (rxPar[i].LNAmonPts[4] > IDMAX) )  biasSatus[i] |= 0x4000;
		if ( (rxPar[i].LNAmonPts[5] < IDMIN) || (rxPar[i].LNAmonPts[5] > IDMAX) )  biasSatus[i] |= 0x0400;
		// drain v
		if (fabsf(rxPar[i].LNAmonPts[2] - rxPar[i].LNAsets[2]) > VDEVMAX)  biasSatus[i] |= 0x2000;
		if (fabsf(rxPar[i].LNAmonPts[3] - rxPar[i].LNAsets[3]) > VDEVMAX)  biasSatus[i] |= 0x0200;
		// gate v
		if (fabsf(rxPar[i].LNAmonPts[0] - rxPar[i].LNAsets[0]) > VDEVMAX)  biasSatus[i] |= 0x1000;
		if (fabsf(rxPar[i].LNAmonPts[1] - rxPar[i].LNAsets[1]) > VDEVMAX)  biasSatus[i] |= 0x0100;
		/* // mixer i
		if ( (rxPar[i].LNAmonPts[8] < IMMIN) || (rxPar[i].LNAmonPts[8] > IMMAX) )  biasSatus[i] |= 0x0020;
		if ( (rxPar[i].LNAmonPts[9] < IMMIN) || (rxPar[i].LNAmonPts[9] > IMMAX) )  biasSatus[i] |= 0x0002;
		// mixer v
		if (fabsf(rxPar[i].LNAmonPts[6] - rxPar[i].LNAsets[4]) > VDEVMAX*2.)  biasSatus[i] |= 0x0010;
		if (fabsf(rxPar[i].LNAmonPts[7] - rxPar[i].LNAsets[5]) > VDEVMAX*2.)  biasSatus[i] |= 0x0001;
		*/
		// accumulate return value for error summary
		ret |= (int)biasSatus[i];
	}

	return ret;
}


/****************************************************************************************/
/**
  \brief Argus power check

  This command checks power system against limits defined in header.  Also checks that LO power is present.


  \return Zero on no errors, else int reporting errors.
*/
int argus_powCheck(void)
{
	if (!noDCM2) return WRONGBOX;

	int ret = 0;  // return value

	// MSN: power supplies
	if ( (pwrCtrlPar[0] < MINVDSV) || (pwrCtrlPar[0] > MAXVDSV) ) ret |= 0x8000;    //VDS
	if ( (pwrCtrlPar[2] < MINAMPV) || (pwrCtrlPar[2] > MAXAMPV) ) ret |= 0x4000;    //+15V
	if ( (pwrCtrlPar[1] < -MAXAMPV) || (pwrCtrlPar[1] > -MINAMPV) ) ret |= 0x2000;  //-15V
	if ( (pwrCtrlPar[3] < MINVCCV) || (pwrCtrlPar[3] > MAXVCCV) ) ret |= 0x1000;    //VCC
	/*// next nibble CIF
	if ( fabsf(pwrCtrlPar[7] - CIFNOMCURR) > CIFMAXCURRDEV ) ret |= 0x0400;         //cif curr
	if ( (pwrCtrlPar[6] < MINCIFV) || (pwrCtrlPar[6] > MAXCIFV) ) ret |= 0x0200;    //cif out
	if ( (pwrCtrlPar[5] < MINCIFV) || (pwrCtrlPar[5] > MAXCIFV) ) ret |= 0x0100;    //cif in
	// next nibble warm IF
	if ( (wifPar.psv[0] < MINVCCV) || (wifPar.psv[0] > MAXVCCV) ) ret |= 0x0020;    //wif 1
	if ( (wifPar.psv[0] < MINVCCV) || (wifPar.psv[0] > MAXVCCV) ) ret |= 0x0010;    //wif 0
	*/
	// LSN: vane systems

	return ret;
}


/****************************************************************************************/
/**
  \brief Argus thermal system check

  This command checks thermal system, including cryostat pressure, against limits defined in header.


  \return Zero on no errors, else int reporting errors.
*/
int argus_thermCheck(void)
{
	if (!noDCM2) return WRONGBOX;

	int ret = 0;  // return value

	// compare with limit values
	// MSN: cryostat
	if ( (cryoPar.auxInputs[0] > 1.1) || (cryoPar.auxInputs[0] < 0.5) ) ret |= 0x8000; // pressure
	if ( cryoPar.cryoTemps[4] > MAXINTT ) ret |= 0x4000;    // 77K cold head
	if ( cryoPar.cryoTemps[0] > MAXCOLDT ) ret |= 0x2000;   // 20K cold head
	if ( cryoPar.cryoTemps[5] > MAXCOLDT  ) ret |= 0x1000;  // LNAs warm
	// next nibble: warm elex
	// check max WIF temp, chassis temp
	// next nibble: vane position
	// removed from word 2017.02.04 ret |= vaneErr;  // value defined by VANEPOSERR

	return ret;
}


/****************************************************************************************/
/**
  \brief Argus system status word

  This command constructs a word encoding the system status: freeze, vane position, etc.

  \return int reporting state.
*/
int argus_systemState(void)
{
	int ret = 0;  // return value

	// set state bits
	// LSN for freeze
	if ( freezeSys ) ret |= 0x0001;
    // next nibble for vane
    // next nibble
    // (unused)
    // MSN
    // (unused)
	return ret;
}


/****************************************************************************************/
/**
  \brief Argus test command.

  This command "succeeds" iff the sum of the arguments is at least unity.

  \param  foo  A signed integer.
  \param  bar  A floating-point value.
  \return Zero on success, else -1.
*/
int argus_test(int foo, float bar)
{
  int rtn = (foo+bar >= 1.0 ? 0 : -1);
  printf("argus_test: foo=%d, bar=%g, rtn=%d.\n", foo, bar, rtn);

  if (0) {
	  *buffer = 0x000;   // initialize buffer
	  address = 0x41;    // DAC address for VGI2
	  buffer[0] = 0x32;  // internal address for VGI2
	  buffer[1] = 0x01;  // first data byte
	  buffer[2] = 0x02;  // second data byte
	  printf("sending\n");
	  I2CStat = I2CSendBuf(address, buffer, (strlen( (const char*)buffer ) ));
	  printf("done\n");
  }

  if (0){
	  argus_readAllSystemADCs();
  }

  if (0) {
	  *buffer = 0x000;   // initialize buffer
	  buffer[0] = (BYTE)0xff;
	  //buffer[0] = (BYTE)0x21;
	  address = (BYTE)0x70;  // addresses 0x70, 0x71
	  //while (1) I2CSEND1;  // infinite loop to make warm IF switch test
  }

  return rtn;
}


/************************************************************************/

// DCM2 storage structures
// Channel-specific structure
/*struct dcm2params {
	BYTE status[NRX]; // status byte
	BYTE attenI[NRX]; // command attenuation, I channel
	BYTE attenQ[NRX]; // command attenuation, Q channel
	float powDetI[NRX]; // nominal power in dBm, I channel
	float powDetQ[NRX]; // nominal power in dBm, Q channel
	float bTemp[NRX];   // board temperature, C
};*/
struct dcm2params dcm2Apar = {  // structure for IF bank A parameters
		{9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
		{198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198},
		{198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198},
		{-99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99},
		{-99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99},
		{999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999}
};
struct dcm2params dcm2Bpar  = {  // structure for IF bank B parameters
		{9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
		{198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198},
		{198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198},
		{-99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99},
		{-99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99},
		{999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999}
};

// Vector to store on-board ADC values: Ain3, Ain2, Ain1, Ain0, MonP12, MonP8, GND, GND
float dcm2MBpar[] = {99, 99, 99, 99, 99, 99, 99, 99, 99};

// DCM2 I2C switch settings for subbus and subsubbuses
struct dcm2switches {
	// I2C switch settings for addressing DCM2 module cards
	BYTE sb[NRX];     // for subbus
	BYTE ssba[NRX];   // for subsbubus, Bank A
	BYTE ssbb[NRX];   // for subsubbus, Bank B
};

// Switch settings for DCM2 channel mapping
#if 1   // set to 1 for normal operation, 0 for all ports to connect to ch 20, band A
struct dcm2switches dcm2sw = {
	{0x10, 0x10, 0x10, 0x10, 0x08, 0x08, 0x08, 0x08, 0x04, 0x04, 0x04, 0x04,
				0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01},
	{0x08, 0x04, 0x02, 0x01, 0x08, 0x04, 0x02, 0x01, 0x08, 0x04, 0x02, 0x01,
			    0x08, 0x04, 0x02, 0x01, 0x08, 0x04, 0x02, 0x01},
	{0x80, 0x40, 0x20, 0x10, 0x80, 0x40, 0x20, 0x10, 0x80, 0x40, 0x20, 0x10,
			    0x80, 0x40, 0x20, 0x10, 0x80, 0x40, 0x20, 0x10}
};
#else
struct dcm2switches dcm2sw = {  // all channels point to 20A
	{0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
			0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01},
	{0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
			0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01},
	{0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
			0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}
};
#endif

// Flag for DCM2 board detection; 0 if present, !0 else
int noDCM2;

/*******************************************************************/
/**
  \brief Set an I2C subbus switch.

  This function sets an TCA9548A I2C subbus switch.

  \param  addr_sb command byte to set the subbus switch.
  \return Zero on success, else NB I2C error code for latest bus error.
*/
/*------------------------------------------------------------------
 * Set up I2C subbus
 ------------------------------------------------------------------*/
int openI2Csbus(BYTE addr_sb, BYTE swset_sb)
{
    // check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	address = addr_sb;        // I2C switch address
	buffer[0] = swset_sb;   // I2C channel address
	I2CStat = I2CSEND1;

	if (I2CStat) i2cBusBusy = 0;  // clear bit if write errors subbus

	return I2CStat;
}

/*******************************************************************/
/**
  \brief Open a I2C subbus switch.

  This function opens TCA9548A I2C subbus switche.

  \return Zero on success, else NB I2C error code for latest bus error.
*/
int closeI2Csbus(BYTE addr_sb)
{
	  address = addr_sb;    // I2C switch address
	  buffer[0] = 0x00;  // I2C channel address 0x00 to open all switches
	  I2CStat = I2CSEND1;

	  // release I2C bus
	  i2cBusBusy = 0;

	  return I2CStat;
}

/*******************************************************************/
/**
  \brief Set a pair of I2C subbus and subsubbus switches.

  This function sets a sequential pair of TCA9548A I2C switches, first the
  subbus, then the subsubbus.

  Checks if the I2C bus is busy and if not sets the busy bit.

  \param  addr_sb    I2C addresss of the subbus switch.
  \param  swset_sb   Switch set command byte for the subbus switch.
  \param  addr_ssb   I2C addresss of the subsubbus switch.
  \param  swset_ssb  Switch set command byte for the subsubbus switch.
  \return Zero on success, else NB I2C error code for latest bus error.
*/
int openI2Cssbus(BYTE addr_sb, BYTE swset_sb, BYTE addr_ssb, BYTE swset_ssb)
{
    // check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	address = addr_sb;      // I2C switch address
	buffer[0] = swset_sb;   // I2C switch setting
	I2CStat = I2CSEND1;

	address = addr_ssb;     // I2C switch address
	buffer[0] = swset_ssb;  // I2C switch setting
	I2CStat = I2CSEND1;

	if (I2CStat) i2cBusBusy = 0;  // clear bit if write errors on subsubbus

	return I2CStat;
}

/*******************************************************************/
/**
  \brief Close a pair of I2C subbus and subsubbus switches.

  This function closes a sequential pair of TCA9548A I2C switches, first the
  subbus, then the subsubbus.

  Then it clears the I2C bus busy bit.

  \param  addr_sb    I2C addresss of the subbus switch.
  \param  addr_ssb   I2C addresss of the subsubbus switch.
  \return Zero on success, else NB I2C error code for latest bus error.
*/
int closeI2Cssbus(BYTE addr_sb, BYTE addr_ssb)
{
	address = addr_ssb;     // I2C switch address
	buffer[0] = 0x00;       // I2C switch setting
	I2CStat = I2CSEND1;

	address = addr_sb;      // I2C switch address
	buffer[0] = 0x00;       // I2C switch setting
	I2CStat = I2CSEND1;

	// release I2C bus
	i2cBusBusy = 0;

	return I2CStat;
}

/*******************************************************************/
/**
  \brief Configure bus expander (BEX)

  This function configures a TCA6408A bus expander for which pins will be set as
  inputs and which as outputs.
  NOTE: This function must be called when externally protected for I2C bus busy

  \param  conf configuration register content byte.
  \param  addr I2C address for BEX chip
  \return Zero on success, else NB I2C error code for latest bus error.
*/
int configBEX(BYTE conf, BYTE addr)
{
	address = addr;    // I2C address for BEX chip on board
	buffer[0] = 0x03;  // configuration register
	buffer[1] = conf;  // conf = 0x01 for warm IF tester, LSB only reads
	I2CStat = I2CSEND2;

	return (I2CStat);
}

/*******************************************************************/
/**
  \brief Write to bus expander (BEX) output pins.

  This function sets values on TCA6408A bus expander output pins.
  NOTE: This function must be called when externally protected for I2C bus busy

  \param  val output content byte.
  \param  addr I2C address for BEX chip
  \return Zero on success, else NB I2C error code for latest bus error.
*/
int writeBEX(BYTE val, BYTE addr)
{
	address = addr;    // I2C address for BEX chip on board
	buffer[0] = 0x01;  // output port register
	buffer[1] = val;   // conf = 0x01 for warm IF tester, LSB only reads
	I2CStat = I2CSEND2;

	return (I2CStat);
}

/*******************************************************************/
/**
  \brief Read bus expander (BEX) input pins.

  This function reads values on TCA6408A bus expander input pins.
  NOTE: This function must be called when externally protected for I2C bus busy

  \param  addr I2C address for BEX chip
  \return byte with input value.
*/
BYTE readBEX(BYTE addr)
{
	address = addr;    // I2C address for BEX chip on board
	buffer[0] = 0x00;  // input port register
	I2CSEND1;
	I2CREAD1;

	return (buffer[0]);
}

/*******************************************************************/
/**
  \brief Read all channels of DCM2 ADC.

  This function reads values from all channels of the DCM2 LTC2309 ADC.

  \return Zero on success, else (9000 + NB I2C error code) for latest bus error.
*/
int dcm2_readMBadc(void)
{

	if (noDCM2) return WRONGBOX;  // return if no DCM2 is present

	short i;
	unsigned short int rawu;
	const float offset[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	const float scale[8] = {4.3, 4.3, 4.3, 4.3, 33.95, 4.3, 4.3, 4.3};
	//const float scale[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // for calibration
	//

	int I2CStatus = openI2Csbus(DCM2_SBADDR, DCM2PERIPH_SBADDR);  // get bus control
	if (I2CStatus) return (I2CStatus);

	//Read all channels of ADC (only 6 are connected, but keep usual structure)
	for (i = 0 ;  i < 6 ; i++) {
		address = (BYTE)0x08;    // ADC device address on I2C bus
		buffer[0] = (BYTE)pcRead.add[i];        // internal address for channel
		I2CStat = I2CSEND1;  // send command for conversion
		I2CREAD2;  // read device buffer back
		if (I2CStat == 0) {
			rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
			dcm2MBpar[i] = rawu*scale[i]*4.096/65535 + offset[i];
		}
		else dcm2MBpar[i] = 9999.;  // error condition
	}

	closeI2Csbus(DCM2_SBADDR);  // release I2C bus

	return (I2CStat);
}

/*******************************************************************/
/**
  \brief Turn on/off DCM2 amplifier power.

  This function turns on the software-controlled power supply for the DCM2.  Default is to
  turn the amplifier power supply on.

  \par inp  string: "off" or "0" for off, else on

  \return NB error code for write to BEX.
*/
int dcm2_ampPow(char *inp)
{
	if (noDCM2) return WRONGBOX;  // return if no DCM2 is present

	int I2CStatus = openI2Csbus(DCM2_SBADDR, DCM2PERIPH_SBADDR);
	if (I2CStatus) return (I2CStatus);

	if (!strcasecmp(inp, "off") || !strcasecmp(inp, "0")) {
		I2CStatus = writeBEX(readBEX(BEX_ADDR0) | DCM2_AMPPOW, BEX_ADDR0);
	} else {
		I2CStatus = writeBEX(readBEX(BEX_ADDR0) & ~DCM2_AMPPOW, BEX_ADDR0);
	}

	closeI2Csbus(DCM2_SBADDR);
	return(I2CStatus);
}

/*******************************************************************/
/**
  \brief Turn on/off DCM2 indicator LED.

  This function turns on the software-controlled power supply for the DCM2.  Default is to
  turn the amplifier power supply on.

  \par inp  string: "off" or "0" for off, else on

  \return NB error code for write to BEX.
*/
int dcm2_ledOnOff(char *inp)
{
	if (noDCM2) return WRONGBOX;  // return if no DCM2 is present

	int I2CStatus = openI2Csbus(DCM2_SBADDR, DCM2PERIPH_SBADDR);  // get bus control
	if (I2CStatus) return (I2CStatus);

	if (!strcasecmp(inp, "off") || !strcasecmp(inp, "0")) {
		I2CStatus = writeBEX(readBEX(BEX_ADDR0) | (DCM2_BD_LED | DCM2_FP_LED), BEX_ADDR0); // high for off
	} else {
		I2CStatus = writeBEX(readBEX(BEX_ADDR0) & ~(DCM2_BD_LED | DCM2_FP_LED), BEX_ADDR0);  // low for on
	}

	closeI2Csbus(DCM2_SBADDR);
	return(I2CStatus);
}

/*******************************************************************/
/**
  \brief SPI bit-bang read AD7841 10-bit temperature sensor.

  This function reads an AD7841 10-bit temperature sensor by generating SPI
  bit-bang signals through a TCA6408A parallel interface chip.

  Requires previous call to function that sets I2C bus switches to
  address the interface before use, and close after.  Also requires
  previous single call to initialize interface chip.

  \param  val  value to convert and send
  \return voltage, or (990 + NB I2C error code) for bus errors.
*/
float AD7814_SPI_bitbang(BYTE spi_clk_m, BYTE spi_dat_m, BYTE spi_csb_m, BYTE addr)
{

	/* Mask definitions are hardware dependent
	*/

	BYTE x = 0;             // Working command byte
	int I2CStat = 0;        // Comms error
 	// var and var_m are defined below

	// get command state of output pins on interface
	x = readBEX(addr);

    buffer[0] = 0x01;    // write register
	address = addr;      // I2C address for BEX chip on board
	// set up for read
	x |= spi_csb_m;       // ensure CS is high at start
	x |= spi_clk_m;      // ensure CLK is high at start
	buffer[1] = x;
	I2CStat = I2CSEND2;
	if (I2CStat) return (9000+I2CStat);

	x &= ~spi_csb_m;      // send CS low to initiate read
	buffer[1] = x;
	I2CSEND2;

	// read initial zero, one cycle
	x &= ~spi_clk_m;     // set clock low
	buffer[1] = x;
	I2CSEND2;
	x |= spi_clk_m;      // set clock high
	buffer[1] = x;
	I2CSEND2;

	// read MSB at clock high to get sign bit
	x &= ~spi_clk_m;      // set clock low
	buffer[1] = x;
	I2CSEND2;
	x |= spi_clk_m;       // set clock high
	buffer[1] = x;
	I2CSEND2;
	buffer[0] = 0x00;     // set input port register and get pin data
	I2CSEND1;
	I2CREAD1;

	int flag = 0;
	// define input variable and mask, deal with negative values
	short int val = 0;              // initialize value
	short int val_m = 0x0100;       // initialize mask
	if (buffer[0] & (unsigned short)spi_dat_m) {
		flag = 1;
		val = 0xfe00; // fill top bits of word if val < 0
	}

	// step through remainder of input word, reading at clock high
	while (val_m > 0) {
		// set clock low
		x &= ~spi_clk_m;
		buffer[0] = 0x01;
		buffer[1] = x;
		I2CSEND2;
		// set clock high
		x |= spi_clk_m;
		buffer[1] = x;
		I2CSEND2;
		// read bit
		buffer[0] = 0x00;
		I2CSEND1;
		I2CREAD1;
		// put bit in word and rotate mask for next-LSB
		if (buffer[0] & (unsigned short)spi_dat_m) val |= val_m;
		val_m >>= 1;
	}

	// all done, deselect with CS& high
	x |= spi_csb_m;
	buffer[0] = 0x01;
	buffer[1] = x;
	I2CStat = I2CSEND2;

	return ( I2CStat ? (float)(990+I2CStat) : (float)val*0.25 );
}

/*******************************************************************/
/**
  \brief Read the temperature of the DCM2 main control board.

  This function reads the temperature sensor on the DCM2 main control board.  Writes values into dcm2MBpar
  array.

  \return Zero on success, else NB I2C error code for latest bus error.
*/
int dcm2_readMBtemp(void)
{

	if (noDCM2) return WRONGBOX;  // return if no DCM2 is present

	int I2CStatus = openI2Csbus(DCM2_SBADDR, DCM2PERIPH_SBADDR);  // get bus control
	if (I2CStatus) return (I2CStatus);

	// read thermometer
	dcm2MBpar[7] = AD7814_SPI_bitbang(SPI_CLK0_M, SPI_DAT0_M, SPI_CSB1_M, BEX_ADDR0);

	I2CStat = closeI2Csbus(DCM2_SBADDR);  // release I2C bus

	return (I2CStat);
}

/*******************************************************************/
/**
  \brief Read the temperature of all DCM2 modules.

  This function reads the temperature sensor on all DCM2 modules.  Writes values into dcm2Apar and
  dcm2Bpar structures.  BEXs initialized in dcm2_init() function.

  \return Zero on success, else NB I2C error code for latest bus error.
*/
int dcm2_readAllModTemps(void)
{

	if (noDCM2) return WRONGBOX;  // return if no DCM2 is present

	// use this approach to lock bus for multiple readouts
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	int m;  // loop counter
	for (m=0; m<NRX; m++){
		// read temperatures A band
		if (!dcm2Apar.status[m]) {
			address = DCM2_SBADDR;            // I2C switch address DCM2_SBADDR for top-level switch
			buffer[0] = dcm2sw.sb[m];  // pick subbus
			I2CSEND1;
			// select, then read temperature A bank
			address = DCM2_SSBADDR;
			buffer[0] = dcm2sw.ssba[m];  // I2C subsubbus address
			I2CSEND1;
			dcm2Apar.bTemp[m] = AD7814_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, BOARD_T_CS, BEX_ADDR);
		}

		// read temperatures B band
		if (!dcm2Bpar.status[m]) {
			address = DCM2_SBADDR;            // I2C switch address DCM2_SBADDR for top-level switch
			buffer[0] = dcm2sw.sb[m];  // pick subbus
			I2CSEND1;
			address = DCM2_SSBADDR;
			buffer[0] = dcm2sw.ssbb[m];  // I2C subsubbus address
			I2CSEND1;
			dcm2Bpar.bTemp[m] = AD7814_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, BOARD_T_CS, BEX_ADDR);
		}
	}
	// close switches
	int I2CStat = closeI2Cssbus(DCM2_SBADDR, DCM2_SSBADDR);
	// release I2C bus
	i2cBusBusy = 0;

	return I2CStat;

}

/*******************************************************************/
/**
  \brief SPI bit-bang read AD7860 16-bit ADC.

  This function reads an AD7860 16-bit ADC by generating SPI
  bit-bang signals through a TCA6408A parallel interface chip.

  Requires previous call to function that sets I2C bus switches to
  address the interface before use, and close after.  Also requires
  previous single call to initialize interface chip.

  \param  val  value to convert and send
  \return voltage, or (9000 + NB I2C error code) for bus errors.
*/
float AD7860_SPI_bitbang(BYTE spi_clk_m, BYTE spi_dat_m, BYTE spi_csb_m, float vdd, BYTE addr)
{

	BYTE x ;                        // Working command byte
	int I2CStat = 0;                // Comms error counter
	short unsigned int val = 0;     // ADC value
	short unsigned int val_m;       // Mask

	// get command state of output pins on interface
	address = addr;      // I2C address for BEX chip on board
	buffer[0] = 0x01;    // output port register
	I2CStat = I2CSEND1;  // set register
	// kick out for I2C bus errors
	if (I2CStat) return (9000+I2CStat);
	I2CREAD1;            // get pin data
	x = buffer[0];       // update working byte

	// set up for read
	x |= spi_csb_m;       // ensure CS is high at start
	x |= spi_clk_m;      // ensure CLK is high at start
    buffer[0] = 0x01;    // write register
	buffer[1] = x;
	I2CSEND2;
	x &= ~spi_csb_m;      // send CS low to initiate read
	buffer[1] = x;
	I2CSEND2;

	// read three cycles of initial zeros
	val_m = 1 << (3 - 1);
	while (val_m > 0) {
		x &= ~spi_clk_m;     // set clock low
		buffer[1] = x;
		I2CSEND2;
		x |= spi_clk_m;      // set clock high
		buffer[1] = x;
		I2CSEND2;
		val_m >>= 1;
	}

	// step through 16-bit input word, msb to lsb, reading at clock high
	val_m = 1 << (16 - 1);
	while (val_m > 0) {
		// set clock low
		x &= ~spi_clk_m;
		buffer[0] = 0x01;
		buffer[1] = x;
		I2CSEND2;
		// set clock high
		x |= spi_clk_m;
		buffer[1] = x;
		I2CSEND2;
		// read bit
		buffer[0] = 0x00;
		I2CSEND1;
		I2CREAD1;
		// put bit in word and rotate mask for next-LSB
		if (buffer[0] & (unsigned short)spi_dat_m) val |= val_m;
		val_m >>= 1;
	}

	// all done. clock low, then deselect with CS& high
	buffer[0] = 0x01;
	x &= ~spi_clk_m;
	buffer[1] = x;
	x |= spi_csb_m;
	buffer[1] = x;
	I2CStat = I2CSEND2;

	return ( I2CStat ? (float)(9000+I2CStat) : (float)val*vdd/65536. );
}

/*******************************************************************/
/**
  \brief Read the power detector voltages of all DCM2 modules.

  This function reads the I&Q power detector voltages on all DCM2 modules.  Writes values into dcm2Apar and
  dcm2Bpar structures.  BEXs initialized in dcm2_init() function.

  \return Zero on success, else NB I2C error code for latest bus error.
*/
int dcm2_readAllModTotPwr(void)
{

	if (noDCM2) return WRONGBOX;  // return if no DCM2 is present

	// check that I2C bus is available, else return
	// use this approach to lock bus during multiple readouts
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	int m;  // loop counter
	for (m=0; m<NRX; m++){
		if (!dcm2Apar.status[m]) {
			// first set addresses to select A band DCM2 module
			address = DCM2_SBADDR;            // I2C switch address DCM2_SBADDR for top-level switch
			buffer[0] = dcm2sw.sb[m];  // pick subbus
			I2CSEND1;
			// select, then read powDets A bank
			address = DCM2_SSBADDR;
			buffer[0] = dcm2sw.ssba[m];  // I2C subsubbus address
			I2CSEND1;
			dcm2Apar.powDetI[m] = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, ILOG_CS, ADCVREF, BEX_ADDR);
			dcm2Apar.powDetQ[m] = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, QLOG_CS, ADCVREF, BEX_ADDR);
			dcm2Apar.powDetI[m] = (dcm2Apar.powDetI[m] < ADCVREF ? dcm2Apar.powDetI[m]*DBMSCALE + DBMOFFSET : -99.);
			dcm2Apar.powDetQ[m] = (dcm2Apar.powDetQ[m] < ADCVREF ? dcm2Apar.powDetQ[m]*DBMSCALE + DBMOFFSET : -99.);
		}

		if (!dcm2Bpar.status[m]) {
			// first set addresses to select A band DCM2 module
			address = DCM2_SBADDR;            // I2C switch address DCM2_SBADDR for top-level switch
			buffer[0] = dcm2sw.sb[m];  // pick subbus
			I2CSEND1;
			// select, configure, then read powDets B bank
			address = DCM2_SSBADDR;
			buffer[0] = dcm2sw.ssbb[m];  // I2C subsubbus address
			I2CSEND1;
			dcm2Bpar.powDetI[m] = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, ILOG_CS, ADCVREF, BEX_ADDR);
			dcm2Bpar.powDetQ[m] = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, QLOG_CS, ADCVREF, BEX_ADDR);
			dcm2Bpar.powDetI[m] = (dcm2Bpar.powDetI[m] < ADCVREF ? dcm2Bpar.powDetI[m]*DBMSCALE + DBMOFFSET : -99.);
			dcm2Bpar.powDetQ[m] = (dcm2Bpar.powDetQ[m] < ADCVREF ? dcm2Bpar.powDetQ[m]*DBMSCALE + DBMOFFSET : -99.);
		}
	}

	// close switches and release bus
	int I2CStat = closeI2Cssbus(DCM2_SBADDR, DCM2_SSBADDR);

	return I2CStat;

}

/*******************************************************************/
/**
  \brief SPI bit-bang write to 6-bit step attenuator.

  This function converts an unsigned integer to a series of I2C commands
  for a HMC624 step attenuator.

  Requires previous call to function that sets I2C bus switches to
  address the interface before use, and close after.  Also requires
  previous single call to initialize interface chip.

  \param  atten  attenuation to convert and send
  \return NB I2C error code for bus errors.

*/
int HMC624_SPI_bitbang(BYTE spi_clk_m, BYTE spi_dat_m, BYTE spi_csb_m, float atten, BYTE addr, BYTE *bits)
{

	BYTE x = 0;                       // working byte
	int I2CStat = 0;                  // comms errors
	BYTE attenBits, attenBits_m;    // binary value word and mask

	if (atten < 0.) atten = 0.;
	if (atten > MAXATTEN) atten = MAXATTEN;
    attenBits = (BYTE)round(atten*2);
 	*bits = attenBits;

	// get command state of output pins on interface
	x = readBEX(addr);

    buffer[0] = 0x01;    // write register
	address = addr;      // I2C address for BEX chip on board
	// set up for read
	x |= spi_csb_m;       // ensure CS is high at start
	x |= spi_clk_m;      // ensure CLK is high at start
	buffer[1] = x;
	I2CStat = I2CSEND2;
	if (I2CStat) return (9000+I2CStat);

	x &= ~spi_csb_m;      // send CS low to initiate read
	buffer[1] = x;
	I2CSEND2;

	// step through input word with mask; write data with clock low, then raise clock
	attenBits_m = 1 << (6 - 1);
	while (attenBits_m > 0) {
		x &= ~spi_clk_m; // set clock low
		if (attenBits & attenBits_m) { // look at bit value in word to set data bit
			x &= ~spi_dat_m;  // bit inversion: 0 is logical TRUE
		}
		else {
			x |= spi_dat_m;
		}
		buffer[1] = x;
		I2CSEND2;        // write to bus extender
		x |= spi_clk_m;  // set clock high, data will be valid here
		buffer[1] = x;
		I2CSEND2;        // write to bus extender
		attenBits_m >>= 1;     // rotate to next-MSB
	}

	// done, set CS high to terminate SPI write
	x |= spi_csb_m;
	buffer[1] = x;
	I2CStat += I2CSEND2;

	return ( I2CStat );

}

/********************************************************************/
/**
  \brief Set all DCM2 attenuators.

  This command sets the attenuators in the DCM2 modules

  \param  inp  select on a for attenuation.
  \param  m    mth receiver.
  \param  ab   A or B channel
  \param  iq   I or Q channel
  \param  atten  attenuation value.
  \return Zero on success, -1 for invalid selection, else number of I2C read fails.
*/
int dcm2_setAtten(int m, char *ab, char *iq, float atten)
{

	if (noDCM2) return WRONGBOX;  // return if no DCM2 is present

	BYTE ssb;         // subsubbus address
	BYTE attenBits;   // attenuator command bits
	                  // dcm2parPtr defined globally
	struct dcm2params *dcm2parPtr; // pointer to structure of form dcm2params

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

	// check for out-of-range channel number
	if (m > NRX) {
		return -10;
	}
	// select card for band A or B, set pointer to correct parameter structure
	if (!strcasecmp(ab, "a")) {
		ssb = dcm2sw.ssba[m];
		dcm2parPtr = &dcm2Apar;
	} else if (!strcasecmp(ab, "b")) {
		ssb = dcm2sw.ssbb[m];
		dcm2parPtr = &dcm2Bpar;
	} else {
		return -20;
	}

	if (!dcm2parPtr->status) {
		return -30;  // return if channel is blocked
	}

	// open communication to DCM2 module
	int I2CStatus = openI2Cssbus(DCM2_SBADDR, dcm2sw.sb[m], DCM2_SSBADDR, ssb); // get bus control
	if (I2CStatus) return (I2CStatus);

	// send command
	// select I or Q input on card
	if (!strcasecmp(iq, "i")) {
		I2CStat = HMC624_SPI_bitbang(SPI_CLK_M, SPI_MOSI_M, I_ATTEN_LE, atten, BEX_ADDR, &attenBits);
		if (!I2CStat) {
			dcm2parPtr->attenI[m] = attenBits;  // store command byte for atten
		} else {
			dcm2parPtr->attenI[m] = 198;
		}
	} else if (!strcasecmp(iq, "q")){
		I2CStat = HMC624_SPI_bitbang(SPI_CLK_M, SPI_MOSI_M, Q_ATTEN_LE, atten, BEX_ADDR, &attenBits);
		if (!I2CStat) {
			dcm2parPtr->attenQ[m] = attenBits;  // store command byte for atten
		} else {
			dcm2parPtr->attenQ[m] = 198;
		}
	} else {  // invalid choice for IQ channels
		closeI2Cssbus(DCM2_SBADDR, DCM2_SSBADDR);
		return (-40);
	}

	// close up and return
	return (closeI2Cssbus(DCM2_SBADDR, DCM2_SSBADDR));
}

/*******************************************************************************************/
/**
  \brief Set all DCM2 attenuators.

  This command sets the attenuators in the DCM2 modules

  \param  inp  select on a for attenuation.
  \param  m    mth receiver.
  \param  atten  attenuation value.
  \return Zero on success, -1 for invalid selection, else number of I2C read fails.
*/
int dcm2_setAllAttens(float atten)
{

	if (noDCM2) return WRONGBOX;  // return if no DCM2 is present

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

	// check that I2C bus is available, else return
	// use the explicit method here and below to simplify bus lock/unlock
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	int m;  // loop counter
	int I2CStat = 0;
	BYTE attenBits;

	// do this one atten at a time for error tracking
	for (m=0; m<NRX; m++){

		if (!dcm2Apar.status[m]) {
			// first set addresses to select a and b channels of DCM2 modules
			address = DCM2_SBADDR;        // I2C switch address DCM2_SBADDR for top-level switch
			buffer[0] = dcm2sw.sb[m];   // I2C channel address  0x01 for second-level switch
			I2CSEND1;
			address = DCM2_SSBADDR;        // I2C switch address DCM2_SSBADDR for second-level switch, band A
			buffer[0] = dcm2sw.ssba[m];
			I2CSEND1;

			I2CStat = HMC624_SPI_bitbang(SPI_CLK_M, SPI_MOSI_M, I_ATTEN_LE, atten, BEX_ADDR, &attenBits);
			if (!I2CStat) {
				dcm2Apar.attenI[m] = attenBits;  // store command bits for atten
			} else {
				dcm2Apar.attenI[m] = 198;
			}

			I2CStat = HMC624_SPI_bitbang(SPI_CLK_M, SPI_MOSI_M, Q_ATTEN_LE, atten, BEX_ADDR, &attenBits);
			if (!I2CStat) {
				dcm2Apar.attenQ[m] = attenBits;  // store command bits for atten
			} else {
				dcm2Apar.attenQ[m] = 198;
			}
		}

		if (!dcm2Bpar.status[m]) {
			// first set addresses to select a and b channels of DCM2 modules
			address = DCM2_SBADDR;        // I2C switch address DCM2_SBADDR for top-level switch
			buffer[0] = dcm2sw.sb[m];   // I2C channel address  0x01 for second-level switch
			I2CSEND1;
			address = DCM2_SSBADDR;        // I2C switch address DCM2_SSBADDR for second-level switch, band A
			buffer[0] = dcm2sw.ssbb[m];
			I2CSEND1;

			I2CStat = HMC624_SPI_bitbang(SPI_CLK_M, SPI_MOSI_M, I_ATTEN_LE, atten, BEX_ADDR, &attenBits);
			if (!I2CStat) {
				dcm2Bpar.attenI[m] = attenBits;  // store command bits for atten
			} else {
				dcm2Bpar.attenI[m] = 198;
			}

			I2CStat = HMC624_SPI_bitbang(SPI_CLK_M, SPI_MOSI_M, Q_ATTEN_LE, atten, BEX_ADDR, &attenBits);
			if (!I2CStat) {
				dcm2Bpar.attenQ[m] = attenBits;  // store command bits for atten
			} else {
				dcm2Bpar.attenQ[m] = 198;
			}
		}
	}
	    // close up and return; will show error if bus writes are a problem
		return (closeI2Cssbus(DCM2_SBADDR, DCM2_SSBADDR));
}

/********************************************************************/
/**
  \brief Block DCM2 module.

  This command writes a non-zero value to the appropriate status byte of the DCM2 parameters structure.

  \param  m    mth receiver.
  \param  ab   A or B channel
  \return Zero
*/
int dcm2_blockMod(char *ch, char *ab)
{

	if (noDCM2) return WRONGBOX;  // return if no DCM2 is present

	struct dcm2params *dcm2parPtr; // pointer to structure of form dcm2params
	int m;
	sscanf(ch, "%d", &m);          // convert channel from string to int
	m -= 1;                        // change to ones-based counting
	if (m > NRX) return -10;       // trap out-of-range channel number

	// select band A or B, set pointer to correct parameter structure
	if (!strcasecmp(ab, "a")) {
		dcm2parPtr = &dcm2Apar;
	} else if (!strcasecmp(ab, "b")){
		dcm2parPtr = &dcm2Bpar;
	} else {
		return -20;
	}

	dcm2parPtr->status[m] = 1;

	return (0);
}

/*******************************************************************************************/
/**
  \brief Send DCM2 init string.

  This command sets the attenuators in the DCM2 modules

  \return Zero.
*/
int init_dcm2(void)
{

	/*
	 * Both DCM2 main board and microC have bus switches at 0x77 and 0x73
	 * For DCM2, 0x77 is subbus, 0x73 is subsubbus from 0x77
	 * For microC, 0x77 and 0x73 are independent subbuses
	 * For DCM2,  0x80 switch setting from 0x77 connects to main board peripherals, BEX @ 0x21
	 * For microC, 0x80 switch setting from 0x77 connects to a bias card without BEXs (ADCs and DACs)
	 * So attempting to address the DCM2 main board BEX will show whether the hardware is bias or DCM2
	 */
	// DCM2 setups

	// Configure and initialize BEX on main board; test to see if DCM2 is connected
	openI2Csbus(0x77, DCM2PERIPH_SBADDR);
	noDCM2 = configBEX(BEXCONF0, BEX_ADDR0);
	writeBEX(BEXINIT0, BEX_ADDR0);
	closeI2Csbus(0x77);

	if (noDCM2==0) {
    	// Configure and initialize BEXs on DCM2 modules
    	int m;  // loop counter
    	for (m=0; m<NRX; m++){
    		// select, configure, and initialize A bank; keep track in status element
    		address = 0x77;            // I2C switch address 0x77 for top-level switch
    		buffer[0] = dcm2sw.sb[m];  // pick subbus
    		I2CSEND1;
    		address = 0x73;
    		buffer[0] = dcm2sw.ssba[m];  // I2C subsubbus address
    		I2CSEND1;
    		dcm2Apar.status[m] = (BYTE)configBEX(BEXCONF, BEX_ADDR);     // zero if BEX responds to init
    		if (!dcm2Apar.status[m]) writeBEX(BEXINIT, BEX_ADDR);  // initialize bus extender
    		J2[28].set();  // reset I2C bus switches in case a subsub bus is stuck
    		OSTimeDly(1);
    		J2[28].clr();  // enable I2C switches
    		// select, configure, and initialize B bank; keep track in status element
    		address = 0x77;            // I2C switch address 0x77 for top-level switch
    		buffer[0] = dcm2sw.sb[m];  // pick subbus
    		I2CSEND1;
    		address = 0x73;
    		buffer[0] = dcm2sw.ssbb[m];  // I2C subsubbus address
    		I2CSEND1;
    		dcm2Bpar.status[m] = (BYTE)configBEX(BEXCONF, BEX_ADDR);       // zero if BEX responds to config
    		if (!dcm2Bpar.status[m]) writeBEX(BEXINIT, BEX_ADDR);  // initialize bus extender
    		J2[28].set();  // reset I2C bus switches in case a subsub bus is stuck
    		OSTimeDly(1);
    		J2[28].clr();  // enable I2C switches
    		}
    	closeI2Cssbus(0x77, 0x73);

    	// Read out once to initialize
		dcm2_readMBadc();
		dcm2_readMBtemp();
		dcm2_readAllModTemps();
		dcm2_readAllModTotPwr();
		// Set to max atten
		dcm2_setAllAttens(MAXATTEN);

		// LED on when init is complete
		dcm2_ledOnOff("on");
    }

	return 0;
}

/*******************************************************************/
/*******************************************************************/
/*
struct saddlebagParams {
	float adcv[8];
	BYTE pll;
	BYTE ampPwr;
	char *ampStatus;};
	float v[8];
}; */
struct saddlebagParams sbPar[] = {
		  {{999., 999., 999., 999., 999., 999., 999., 999.}, 99, 99, "N/A"},
		  {{999., 999., 999., 999., 999., 999., 999., 999.}, 99, 99, "N/A"},
		  {{999., 999., 999., 999., 999., 999., 999., 999.}, 99, 99, "N/A"},
		  {{999., 999., 999., 999., 999., 999., 999., 999.}, 99, 99, "N/A"}
};

/*******************************************************************/
/**
  \brief Turn on/off Saddlebag amplifier power for specified saddlebag.

  This function turns on software-controlled power supply for a saddlebag amplifier.  Default is to
  turn the amplifier power supply on.

  Bus expander should be configured in an init file

  Function does not check for out of range saddlebag index number.

  \par inp  string: "off" or "0" for off, else on

  \return NB error code for write to BEX.
*/
int sb_ampPow(char *inp, int sbNum)
{
	if (!noDCM2) return WRONGBOX;

	BYTE swaddr[] = SADDLEBAG_SWADDR;

	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}                    // check for freeze

	int I2CStatus = openI2Cssbus(SB_SBADDR, I2CSSB_I2CADDR, SB_SSBADDR, swaddr[sbNum]);
	if (I2CStatus) return (I2CStatus);

	if (!strcasecmp(inp, "off") || !strcasecmp(inp, "0")) {
		writeBEX(readBEX(SBBEX_ADDR) & ~0x01, SBBEX_ADDR);  // pin value low
		I2CStatus = configBEX(0x02, SBBEX_ADDR);            // make control pin write
    	if (!I2CStatus) {
    		sbPar[sbNum].ampPwr = 0; // record power state as off
    	} else {
    		sbPar[sbNum].ampPwr = I2CStatus; // indeterminate power state
    	}
	} else {
		I2CStatus = configBEX(0x03, SBBEX_ADDR);            // make control pin high-Z
    	if (!I2CStatus) {
    		sbPar[sbNum].ampPwr = 1; // record power state as on
    	} else {
    		sbPar[sbNum].ampPwr = I2CStatus; // indeterminate power state
    	}
	}

	closeI2Cssbus(SB_SBADDR, SB_SSBADDR);

	// use value in sbPar.ampPwr to set sbPar.ampStatus
	switch (sbPar[sbNum].ampPwr) {
		case 0 :
			sbPar[sbNum].ampStatus = "OFF";
			break;
		case 1 :
			sbPar[sbNum].ampStatus = "on";
			break;
		default : sprintf(sbPar[sbNum].ampStatus, "ERR%d", -sbPar[sbNum].ampPwr);
	}

	return(I2CStatus);
}

/*******************************************************************/
/**
  \brief Turn on/off Saddlebag amplifier power for all saddlebags.

  This function turns on software-controlled power supply for all saddlebag amplifiers.  Default is to
  turn the amplifier power supplies on.

  \par inp  string: "off" or "0" for off, else on

  \return NB error code for write to BEX.
*/
int sb_setAllAmps(char *inp)
{
	if (!noDCM2) return WRONGBOX;

	BYTE swaddr[] = SADDLEBAG_SWADDR;

	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}    // check for freeze
	int I2CStatus = openI2Csbus(SB_SBADDR, I2CSSB_I2CADDR);  // get control of bus
	if (I2CStatus) return (I2CStatus);

	for (int i=0; i<NSBG; i++) {    // loop over saddlebags
		address = SB_SSBADDR;
		buffer[0] = swaddr[i];      // set saddlebag selection switch
		I2CSEND1;
        //configBEX(0x02|(sbPar[i].ampPwr ? 1 : 0), SBBEX_ADDR);  // configure I/O, preserving saddlebag ampl state
    	if (!strcasecmp(inp, "off") || !strcasecmp(inp, "0")) {
        	writeBEX(readBEX(SBBEX_ADDR) & ~0x01, SBBEX_ADDR);  // set pin value low
        	I2CStatus = configBEX(0x02, SBBEX_ADDR);            // make control pin write
        	if (!I2CStatus) {
        		sbPar[i].ampPwr = 0; // record power state as off
        	} else {
        		sbPar[i].ampPwr = I2CStatus; // indeterminate power state
        	}
        } else {
        	I2CStatus = configBEX(0x03, SBBEX_ADDR);            // make control pin high-Z
        	if (!I2CStatus) {
        		sbPar[i].ampPwr = 1; // record power state as on
        	} else {
        		sbPar[i].ampPwr = I2CStatus; // indeterminate power state
        	}
        }
    	// use value in sbPar.ampPwr to set sbPar.ampStatus
    	switch (sbPar[i].ampPwr) {
    		case 0 :
    			sbPar[i].ampStatus = "OFF";
    			break;
    		case 1 :
    			sbPar[i].ampStatus = "on";
    			break;
    		default : sprintf(sbPar[i].ampStatus, "ERR%d", -sbPar[i].ampPwr);
    	}
	}

	closeI2Csbus(SB_SBADDR);

	return(I2CStatus);
}


/*******************************************************************/
/**
  \brief Turn on/off Saddlebag indicator LED.

  This function turns on the software-controlled power supply for the DCM2.  Default is to
  turn the amplifier power supply on.

  Function does not check for out of range saddlebag index number.

  \par inp  string: "off" or "0" for off, else on
  \return NB error code for write to BEX.
*/

int sb_ledOnOff(char *inp, int sbNum)
{
	if (!noDCM2) return WRONGBOX;

	BYTE swaddr[] = SADDLEBAG_SWADDR;

	int I2CStatus = openI2Cssbus(SB_SBADDR, I2CSSB_I2CADDR, SB_SSBADDR, swaddr[sbNum]);  // get bus control
	if (I2CStatus) return (I2CStatus);

	configBEX(0x02|(sbPar[sbNum].ampPwr>0 ? 1 : 0), SBBEX_ADDR);  // configure I/O, preserving saddlebag ampl state

	if (!strcasecmp(inp, "off") || !strcasecmp(inp, "0")) {
		I2CStatus = writeBEX(readBEX(SBBEX_ADDR) | 0x80, SBBEX_ADDR); // high for off
	} else {
		I2CStatus = writeBEX(readBEX(SBBEX_ADDR) & ~0x81, SBBEX_ADDR); // low for on
	}

	closeI2Cssbus(SB_SBADDR, SB_SSBADDR);
	return(I2CStatus);
}

/*******************************************************************/
/**
  \brief Read saddlebag PLL bit.

  This function reads the saddlebag's PLL monitor bit.

  Function does not check for out of range saddlebag index number.

  \par inp  string: "off" or "0" for off, else on
  \return PLL bit value.
*/
int sb_readPLLmon(int sbNum)
{
	if (!noDCM2) return WRONGBOX;

	BYTE sbaddr[] = SADDLEBAG_SWADDR;

	int I2CStatus = openI2Cssbus(SB_SBADDR, I2CSSB_I2CADDR, SB_SSBADDR, sbaddr[sbNum]);
	if (I2CStatus) return BYTE(I2CStatus);

	BYTE pllState = (readBEX(SBBEX_ADDR) & 0x02) >> 1;

	closeI2Cssbus(SB_SBADDR, SB_SSBADDR);

	return(pllState);
}

/*******************************************************************/
/**
  \brief Read all channels of the saddlebag ADC.

  This function reads values from all channels of the DCM2 LTC2309 ADC.

   Function does not check for out of range saddlebag index number.

  \return Zero on success, else NB I2C error code for latest bus error.
*/
int sb_readADC(int sbNum)
{
	if (!noDCM2) return WRONGBOX;

	short i;
	unsigned short int rawu;
	BYTE sbaddr[] = SADDLEBAG_SWADDR;

	// Scale and offset for ADC channels
	// order: +12V, -8V, fan 1, fan 2, temp 1, temp 2, temp 3, temp 4
	const float offset[8] = {0, 0, 0, 0, -50., -50., -50., -50.};
	const float scale[8] = {10., -10., 60., 60., 100., 100., 100., 100.};
	//const float offset[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // for calibration
	//const float scale[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};  // for calibration, in mV

	// get control of I2C bus
	int I2CStatus = openI2Cssbus(SB_SBADDR, I2CSSB_I2CADDR, SB_SSBADDR, sbaddr[sbNum]);
	if (I2CStatus) return (I2CStatus);

	//Read all channels of ADC
	for (i = 0 ;  i < 8 ; i++) {
		address = SBADC_ADDR;               // ADC device address on I2C bus
		buffer[0] = (BYTE)pcRead.add[i];    // internal address for channel
		I2CStat = I2CSEND1;                 // send command for conversion
		I2CREAD2;                           // read device buffer back
		if (I2CStat == 0) {
			rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
			sbPar[sbNum].adcv[i] = rawu*scale[i]*4.096/65535 + offset[i];
		}
		else sbPar[sbNum].adcv[i] = 9999.;  // error condition
	}

	// release I2C bus
	closeI2Cssbus(SB_SBADDR, SB_SSBADDR);

	return (I2CStat);
}

/*******************************************************************/
/**
  \brief Bias system initialization.

  This function initializes the bias system either to make a clean start on boot,
  to properly update the parameters file on boot, or both.

  \return Nothing.
*/

void init_bias(void)
{
		// shut down LNA power if it is on
		// may take a while if lnaPwrState is high but PIOs aren't initialized,
		// or maybe not
		if (lnaPwrState==1) {
			argus_lnaPower(0);
		}

		/** Initialize devices on main I2C bus **/
		//// Power control card
		address = I2CSWITCH_BP;
		buffer[0] = PWCTL_I2CADDR;

		I2CSEND1;
		// set default value in BEX
		address = 0x21;
		buffer[0] = 0x01;
		buffer[1] = 0x20;  // all relays off low; FPLED is off high
		I2CSEND2;
		// configure BEX pins for I/O
		address = 0x21;
		buffer[0] = 0x03;
		buffer[1] = 0xc0;
		I2CSEND2;
		//// Clean up: disconnect I2C sub-buses
		address = I2CSWITCH_BP;
		buffer[0] = 0;
		I2CStat = I2CSEND1;
}

/**
  \brief Saddlebag initialization.

  This function initializes saddlebag boards either to make a clean start on boot,
  to properly update the parameters file on boot, or both.
  - Ensures amps are on
  - Blinks LED or simply turns it on if it were off.

  \return Nothing.
*/

void init_saddlebags(void)
{

	BYTE swaddr[] = SADDLEBAG_SWADDR;
	int i;
	int I2CStat;

	for (i=0; i<NSBG; i++){
		openI2Cssbus(SB_SBADDR, I2CSSB_I2CADDR, SB_SSBADDR, swaddr[i]);  // get bus control
		I2CStat = configBEX(0x03, SBBEX_ADDR);  // configure I/O, amplifiers on
		sbPar[i].ampPwr = (I2CStat ? I2CStat : 1);
		writeBEX(0x80, SBBEX_ADDR);   // turn off LED (if on)
		closeI2Cssbus(SB_SBADDR, SB_SSBADDR);
		// use value in sbPar.ampPwr to set sbPar.ampStatus
		switch (sbPar[i].ampPwr) {
			case 0 :
				sbPar[i].ampStatus = "OFF";
				break;
			case 1 :
				sbPar[i].ampStatus = "on";
				break;
			default : sprintf(sbPar[i].ampStatus, "ERR%d", -sbPar[i].ampPwr);
		}
	}
	OSTimeDly(2);                // perceptible off time for blink
	for (i=0; i<NSBG; i++) {
		openI2Cssbus(SB_SBADDR, I2CSSB_I2CADDR, SB_SSBADDR, swaddr[i]);  // get bus control
		writeBEX(0x00, SBBEX_ADDR);   // turn on LED
		closeI2Cssbus(SB_SBADDR, SB_SSBADDR);
	}
}

/**************************************************************************/
/**
  \brief Initialize hardware.

  This routine is called automatically at boot (for Argus hardware).

  \param  flash  User flash data structure.
*/
void argus_init(const flash_t *flash)
{

	// get voltage divider value from flash
	printf("argus_init: flash vgdiv %f\n", flash->gvdiv);
	gvdiv = flash->gvdiv;  // gvdiv is initialized as a global variable
	if (gvdiv < 0. || gvdiv > 1.) gvdiv = 1.e6;  // protect against uninitialized flash value

	// start I2C interface
	I2CInit( 0xaa, 0x1a );   // Initialize I2C and set NB device slave address and I2C clock
	// Second argument is clock divisor for I2C bus, freqdiv
	// 0x16 for 97.6 kHz (fastest)
	// 0x17 for 78.1 kHz
	// 0x3b for 73.2 kHz
	// 0x18 for 65.1 kHz
	// 0x19 for 58.6 kHz
	// 0x1a for 48.8 kHz
	// 0x1c for 32.6 kHz
	// 0x1f for 19.5 kHz (slowest)
	//Echo setup
	printf("argus_init: flash serNo = %d, hwType = %d, valid = %d.\n",
	  			  flash->serialNo, flash->hw, flash->valid);

	// initialize I2C switch reset line and pulse reset for microC (irrelevant for earlier versions)
	J2[28].function (PINJ2_28_GPIO);  // configure pin J2-28 for GPIO
	J2[28].clr();  // set pin low to enable I2C switches
	OSTimeDly(1);  // delay to make pulse come out right
	J2[28].set();  // reset
	OSTimeDly(1);
	J2[28].clr();  // enable I2C switches


	init_dcm2();       // initialize DCM2 box, determine whether a DCM2 is connected
	if (noDCM2) {      // if not, initialze bias box
		init_bias();       // initialize bias system and backplane
		init_saddlebags(); // initialize saddlebag interface boards
	}


    // release I2C bus
	i2cBusBusy = 0;

}



