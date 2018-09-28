#ifndef ARGUS_H
#define ARGUS_H
/*
   \file
   \author Andy Harris
   \brief  Argus backend declarations.

   $Id: argus.h,v 1.5 2014/06/04 18:35:05 harris Exp $
*/
#include "zpec.h"
#include "argusHardwareStructs.h"

#ifdef __cplusplus
  // Any C++ declarations can go here...
extern struct receiverParams rxPar[];
extern struct cryostatParams cryoPar;
extern struct biasCardParams bcPar[];
extern struct warmIFparams wifPar;
//extern struct muBoxParams muBoxPar;
extern struct calSysParams calSysPar;
extern float pwrCtrlPar[];
extern struct chSet *chSetPtr;
extern struct chRead *chReadPtr;
// dcm2 defs
extern float dcm2MBpar[];
extern struct dcm2params dcm2Apar;
extern struct dcm2params dcm2Bpar;
// saddlebag defs
extern struct saddlebagParams sbPar[];
// vane defs
extern struct vaneParams vanePar;

extern "C" {
#endif

/* All C declarations in this region. */

extern int lnaPwrState;  // LNA power supply state
extern int sbAmpState;   // Saddlebag amplifiers power state
extern unsigned char lnaPSlimitsBypass; // bypass LNA power supply limits when = 1
extern unsigned char cifPSlimitsBypass; // bypass cold IF power supply limits when = 1
extern unsigned char lnaLimitsBypass;   // bypass soft limits on LNA bias when = 1
extern unsigned char stopVaneOnStall;   // bypass timeout on vane stall = 0
extern float gvdiv;                     // Gate voltage divider factor
extern float vaneOffset;                // Vane offset voltage for angle calculation
extern float vaneV2Deg;                 // Vane volts to degrees
extern unsigned char i2cBusBusy;        // I2C bus is busy when = 1
extern unsigned int busLockCtr;         // I2C successful bus lock request counter
extern unsigned int busNoLockCtr;       // I2C unsuccessful bus lock request counter
extern unsigned char freezeSys;         // freeze system state when = 1
extern unsigned int freezeCtr;          // freeze request counter
extern unsigned int thawCtr;            // thaw request counter
extern unsigned int freezeErrCtr;       // freeze error counter (access request while frozen)
extern short unsigned int biasSatus[NRX]; // receiver status word, see argus_rxCheck(void)
extern int i2cState[];                  // I2C bus SCL (0/1) and SDA (0/2) values
extern int foundLNAbiasSys;                      // 0 when DCM2 board is detected

extern void argus_init(const flash_t *flash);
extern int  argus_test(int foo, float bar);
extern int  argus_setLNAbias(char *term, int m, int n, float v, unsigned char busyOverride);
extern int  argus_setAllBias(char *inp, float v, unsigned char busyOverride);
extern int  argus_lnaPower(short state);
extern int  argus_cifPower(short state);
extern int  argus_readLNAbiasADCs(char *sw);
extern int  argus_readPwrADCs(void);
extern int  argus_readBCpsV(void);
extern int  argus_readThermADCs(void);
extern int  argus_lnaPowerPIO(void);
extern int  argus_openSubbusC(BYTE addr);
extern int  argus_closeSubbusC(void);
extern int  argus_clearBus(void);
extern int  argus_readAllSystemADCs(void);
extern int  argus_biasCheck(void);
extern int  argus_powCheck(void);
extern int  argus_thermCheck(void);
extern int  argus_ifCheck(void);
extern int  argus_systemState(void);

extern int  dcm2_setAtten(int m, char *ab, char *iq, float atten);
extern int  dcm2_setAllAttens(float atten);
extern int  dcm2_ampPow(char *inp);
extern int  dcm2_ledOnOff(char *inp);
extern int  dcm2_readMBadc(void);
extern int  dcm2_readMBtemp(void);
extern int  dcm2_readAllModTemps(void);
extern int  dcm2_readAllModTotPwr(void);
extern int  dcm2_blockMod(char *ch, char *ab);
extern int  dcm2_setPow(int m, char *ab, char *iq, float pow);
extern int  dcm2_setAllPow(float pow);
extern int  init_dcm2(void);

extern int  sb_ampPow(char *inp, int sbNum);
extern int  sb_ledOnOff(char *inp, int sbNum);
extern int  sb_readADC(int sbNum);
extern int  sb_readPLLmon(int sbNum);
extern int  sb_setAllAmps(char *inp);
extern void init_saddlebags(void);

extern int  vane_obscal(char *inp);
extern int  vane_readADC(int adcN);
extern void init_vane(void);

extern int  comap_presets(const flash_t *flash);

/**************************************************************************/


#ifdef __cplusplus
}  // extern "C"
#endif

#endif  /* ARGUS_H */
