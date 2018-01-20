/**
  \file
  \author Andy Harris
  \brief Argus control command infrastructure.

  $Id: argus_control.cpp,v 1.2 2014/06/04 18:35:21 harris Exp $
*/

#include "argus.h"
#include "control.h"
#include "math.h"

// names for cryostat test points
char *cnames[] = {"T0", "T1", "T2", "T3", "T4", "T5", "Pressure"};
/*char *cnames[] = {"20K cold head: ",
		          "NC:            ",
		          "20K plate:     ",
		          "70 K plate:    ",
		          "70 K cold head:",
		          "Card cage:     ",
		          "Pressure:      "}; */

// names for saddlebag test points
char *sbnames[] = {"+12V   [V]",
		           "-8V    [V]",
		           "Fan 1 [Hz]",
		           "Fan 2 [Hz]",
		           "Temp 1 [C]",
		           "Temp 2 [C]",
		           "Temp 3 [C]",
		           "Temp 4 [C]",
				   "PLL lock  ",
                   "Amp on    "};

// decimal points for display in exexArgusMonPts
int d1 = 1, d2 = 2;

/********************************************************************************/
/**
  \brief Argus test command.

  This method is a template for testing control command infrastructure.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusTest(return_type status, argument_type arg)
{
  static const char *usage =
  "ARG1 [ARG2]\r\n"
  "  Do something with arguments ARG1 and (optional) ARG2.\r\n"
  "  ARG1  The first  argument (integer).\r\n"
  "  ARG2  The second argument (float; default: 1.0).\r\n";

  if (!arg.help) {
    int   arg1 = 0;
    float arg2 = 1.0;
    if (arg.str) {
      // Command called with one or more arguments.

      int narg = sscanf(arg.str, "%d%f", &arg1, &arg2);
      if (narg == 0) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusTest);
      } else {
        // Execute the command.
        int rtn = argus_test(arg1, arg2);
        sprintf(status, "%sargus_test(%d, %g) returned status %d.\r\n",
                         (rtn==0 ? statusOK : statusERR), arg1, arg2, rtn);
      }
    } else {
      // Command called without arguments. In this example, that's an error.
      longHelp(status, usage, &Correlator::execArgusTest);
    }
  } else {
    longHelp(status, usage, &Correlator::execArgusTest);
  }
}

/**
  \brief Argus init command.

  This method runs the hardware initialization function as a standalone rather than at boot.
  This will
  - pulse the I2C reset line to reset switches
  - turn off the LNAs gracefully if they are on
  - initialize all the parallel IO converters
  - and so on

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusInit(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Initialize hardware without a reboot.\r\n";

  if (!arg.help) {
	  flash_t flashData;
	  zpec_readFlash(&flashData);
	  argus_init(&flashData);  // initialize hardware
	  argus_init(&flashData);  // often seems to need a second reset
	  argus_init(&flashData);  // and one more for good measure
	  sprintf(status, "%sHardware initialized\r\n", statusOK);
  } else {
    longHelp(status, usage, &Correlator::execArgusInit);
  }
}

/**
  \brief Argus freeze command.

  This method sets a bit to freeze the system state, generally meant for during integrations.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusFreeze(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Freeze system state (see thaw) to prevent changes to settings.\r\n";


  if (!arg.help && !arg.str) {
	  freezeSys = 1;
	  freezeCtr += 1;
	  sprintf(status, "%sfreezeSys = %u\r\n", statusOK, freezeSys);
  } else {
	    longHelp(status, usage, &Correlator::execArgusFreeze);
  }
}

/**
  \brief Argus thaw command.

  This method clears a bit to unfreeze the system state, generally meant for not during integrations.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusThaw(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Thaw system state (see freeze) to permit changes to settings.\r\n";

  if (!arg.help && !arg.str) {
	  freezeSys = 0;
	  thawCtr += 1;
	  sprintf(status, "%sfreezeSys = %u\r\n", statusOK, freezeSys);
  } else {
    	longHelp(status, usage, &Correlator::execArgusThaw);
  }
}

/**
  \brief Argus instrument health.

  This method produces an instrument health readout.
  Values returned on the first line should all be zeros indicating nominal operation.  These check
  words are in order of power systems, output IF power, thermal system, then LNA bias cards.
  Information on succeeding lines is in a form easier for humans to decode.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusRxHealth(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Get instrument health error summary, results to screen.\r\n"
  "  Zero value words indicate no errors.\r\n";

  if (!arg.help) {
	  OSTimeDly(CMDDELAY);
	  int rtnStatus = argus_systemState();
	  argus_readAllSystemADCs();
	  int rtnPow = argus_powCheck();
	  int rtnIF = argus_ifCheck();
	  int rtnTherm = argus_thermCheck();
	  int rtnRx = argus_biasCheck();
	  sprintf(status, "%sState and error flags:\r\n"
			  "System status 0x%04x\r\n"
			  "Power errors 0x%04x\r\n"
			  "IF output power errors 0x%04x\r\n"
			  "Thermal errors 0x%04x\r\n"
			  "LNA bias error state 0x%04x\r\n"
			  "Individual receiver bias errors:\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n\r\n",
              (!freezeSys ? statusOK : statusERR),
              rtnStatus, rtnPow, rtnIF, rtnTherm, rtnRx,
              biasSatus[0], biasSatus[1], biasSatus[2],
              biasSatus[3], biasSatus[4], biasSatus[5], biasSatus[6], biasSatus[7], biasSatus[8],
              biasSatus[9], biasSatus[10], biasSatus[11], biasSatus[12], biasSatus[13], biasSatus[14],
              biasSatus[15]);
  } else {
    longHelp(status, usage, &Correlator::execArgusRxHealth);
  }
}

/**
  \brief Set engineering options.

  Set engineering options such as power supply limit bypass for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusEngr(return_type status, argument_type arg)
{
  static const char *usage =
  "[KEYWORD [VALUE]]\r\n"
  "  Set engineering mode functions.\r\n"
  "    KEYWORD         VALUE:\r\n"
  "    bypassLNApsLims  x   magic number x to bypass LNA power supply limits.\r\n"
//  "    bypassCIFpsLim  x   magic number x bypass cold IF power supply limits.\r\n"
  "    bypassLNAlims   y   magic number y to bypass soft limits on LNA biases.\r\n"
//  "    stopVaneOnStall x   0 to ignore vane auto-stop when vane stalled.\r\n"
//  "    sendVane        z   send vane drive hardware integer z.\r\n"
  "    dec             n   n decimal places for MON LNA, MON MIX, MON SETS display\r\n"
  "    clearBus            clear I2C bus busy bit, open main bus switches.\r\n"
  "    clrCtr              clear counters for I2C bus and freeze/thaw.\r\n"
		  ;

  if (!arg.help) {
    if (arg.str) {
      // Command called with one or more arguments.
   	  char kw[15] = {0};
   	  int val;
      int narg = sscanf(arg.str, "%s%d", kw, &val);

      if (narg == 2) {
        // Execute the command.
      	if (!strcasecmp(kw, "bypassLNApsLims")) lnaPSlimitsBypass = (val == 37 ? 1 : 0);
//      	else if (!strcasecmp(kw, "bypassCIFpsLim")) cifPSlimitsBypass = (val == 37 ? 1 : 0);
      	else if (!strcasecmp(kw, "bypassLNAlims"))  lnaLimitsBypass = (val == 74 ? 1 : 0);
      	//else if (!strcasecmp(kw, "stopVaneOnStall"))  lnaLimitsBypass = (val == 0 ? 0 : 1);
      	/*else if (!strcasecmp(kw, "sendVane")) {
      		OSTimeDly(CMDDELAY);
      		if (!argus_openSubbusC(VANE_I2CADDR)) {
      			//argus_setVaneBits((BYTE)(val-200));
      			argus_closeSubbusC();
      		}
      	}*/
      	else if (!strcasecmp(kw, "dec")) {
    		if (val > 2) {
    			d1 = d2 = val;
    		} else {
    			d1 = 1;
    			d2 = 2;
    		}
      	}
    	else longHelp(status, usage, &Correlator::execArgusEngr);
      	sprintf(status,"\r");
      	}
      else if (narg == 1) {
    	  if (!strcasecmp(kw, "clearBus")) {
    		  OSTimeDly(CMDDELAY);
    		  int rtn = argus_clearBus();
    		  sprintf(status, "%sclearBus found status %d, SDA/SCL before and after 0x%x, 0x%x.\r\n",
    				  (rtn==0 ? statusOK : statusERR), rtn, i2cState[0], i2cState[1]);
    	  }
    	  else if (!strcasecmp(kw, "clrCtr")) {
        	  busLockCtr = 0;
        	  busNoLockCtr = 0;
        	  freezeCtr = 0;
        	  thawCtr = 0;
        	  freezeErrCtr = 0;
              sprintf(status,"\r");
          }
    	  else longHelp(status, usage, &Correlator::execArgusEngr);
      }
      else {
      		longHelp(status, usage, &Correlator::execArgusEngr);
      	}
    } else {
    	OSTimeDly(CMDDELAY);
    	sprintf(status, "%sEngineering:\r\n"
    		"  i2cBusBusy = %d, freeze = %u\r\n"
            "  successful and unsuccessful I2C bus lock requests since clrCtr = %u and %u\r\n"
            "  freeze and thaw requests since clrCtr = %u and %u, denials while frozen = %u\r\n"
    		"  bypassLNApsLim = %d\r\n"
       		"  bypassLNAlims = %d\r\n"
     		"  decimal points: %d, %d\r\n"
       		"  power control PIO byte = 0x%02x\r\n"
    		"  version %s\r\n",
    		statusOK, i2cBusBusy, freezeSys,
    		busLockCtr, busNoLockCtr, freezeCtr, thawCtr, freezeErrCtr,
    		lnaPSlimitsBypass, lnaLimitsBypass,
    		d1, d2, argus_lnaPowerPIO(), VER);
    }
  } else {
	  longHelp(status, usage, &Correlator::execArgusEngr);
  }
}

/**
  \brief Argus setting limits.

  Return values of bias setting limits when queried.  This keeps the GBT Manager
  synchronized with the firmware limits.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusLimits(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Return bias setting limits in order:\r\n"
  "  VDGMAX, VGMIN, VGMAX, VDMIN, VDMAX, VMMIN, VMMAX [V], IDMIN, IDMAX, IMMIN, IMMAX [mA],\r\n"
  "  MAXATTEN [dB]\r\n"
		  ;

  if (!arg.help) {
	  sprintf(status, "%s %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %f \r\n",
	      				statusOK, VDGMAX, VGMIN, VGMAX, VDMIN, VDMAX, VMMIN, VMMAX,
	      				IDMIN, IDMAX, IMMIN, IMMAX, MAXATTEN);
  } else {
    longHelp(status, usage, &Correlator::execArgusLimits);
  }
}

/**
  \brief Argus individual drain bias control.

  Set or read a single LNA drain bias for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusDrain(return_type status, argument_type arg)
{
  static const char *usage =
  "[M N V]\r\n"
  "  Set an LNA drain voltage.\r\n"
  "  M is the Mth receiver to set.\r\n"
  "  N is the Nth stage within receiver to set.\r\n"
  "  V is the voltage in V to set.\r\n"
		  ;

  if (!arg.help) {
    int m, n;
    float v = 0.0;
    if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%d%d%f", &m, &n, &v);
      if (narg < 3) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusDrain);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX && n > 0 && n <= NSTAGES){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = argus_setLNAbias("d", m-1, n-1, v, 0);
    		if (rtn == -10) {
        		sprintf(status, "%sLNA cards are not powered, returned status %d.\r\n",
        				statusERR, rtn);
    		} else {
    			sprintf(status, "%sargus_setLNAbias(%d, %d, %f, 0) returned status %d.\r\n",
    				(rtn==0 ? statusOK : statusERR), m, n, v, rtn);
    		}
    	} else {
    		sprintf(status, "%sReceiver or stage number out of range\r\n", statusERR);
    	}
     }
  } else {
      // Command called without arguments; read drain values
      longHelp(status, usage, &Correlator::execArgusDrain);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execArgusDrain);
  }
}

/**
  \brief Argus individual gate bias control.

  Set or read a single LNA gate bias for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusGate(return_type status, argument_type arg)
{
  static const char *usage =
  "[M N V]\r\n"
  "  Set an LNA gate voltage.\r\n"
  "  M is the Mth receiver to set.\r\n"
  "  N is the Nth stage within receiver to set.\r\n"
  "  V is the voltage in V to set.\r\n"
		  ;

  if (!arg.help) {
    int m, n;
    float v = 0.0;
    if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%d%d%f", &m, &n, &v);
      if (narg < 3) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusGate);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX && n > 0 && n <= NSTAGES){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = argus_setLNAbias("g", m-1, n-1, v, 0);
    		if (rtn == -10) {
        		sprintf(status, "%sLNA cards are not powered, returned status %d.\r\n",
        				statusERR, rtn);
    		} else {
    			sprintf(status, "%sargus_setLNAbias(%d, %d, %f, 0) returned status %d.\r\n",
    					(rtn==0 ? statusOK : statusERR), m, n, v, rtn);
    		}
    	} else {
    		sprintf(status, "%sReceiver or stage number out of range\r\n", statusERR);
    	}
     }
  } else {
      // Command called without arguments; read gate values
      longHelp(status, usage, &Correlator::execArgusGate);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execArgusGate);
  }
}


/**
  \brief Argus individual receiver attenuator control.

  Set a single receiver's warm IF attenuation for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusAtten(return_type status, argument_type arg)
{
  static const char *usage =
  "[M AB IQ dB]\r\n"
  "  Set a receiver warm IF attenuation.\r\n"
  "  M is the Mth receiver to set.\r\n"
  "  AB is either A or B IF bank.\r\n"
  "  IQ is either I or Q.\r\n"
  "  dB is the attenuation in dB to set.\r\n"
		  ;

  if (!arg.help) {
    int m;
    char ab[4], iq[4];
    float atten;

   if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%d%1s%1s%f", &m, ab, iq, &atten);
      if (narg < 4) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusAtten);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = dcm2_setAtten(m-1, ab, iq, atten);
   			sprintf(status, "%sdcm2_setAtten(%d, %s, %s, %f) returned status %d.\r\n",
    					(rtn==0 ? statusOK : statusERR), m, ab, iq, atten, rtn);
    	} else {
    		sprintf(status, "%sReceiver number out of range\r\n", statusERR);
    	}
     }
  } else {
      // Command called without arguments; read gate values
      longHelp(status, usage, &Correlator::execArgusAtten);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execArgusAtten);
  }
}

/**
  \brief Argus individual receiver sideband control.

  Set a single receiver's warm IF sideband for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusSB(return_type status, argument_type arg)
{
  static const char *usage =
  "[M S]\r\n"
  "  Set a receiver sideband.\r\n"
  "  M is the Mth receiver to set.\r\n"
  "  S is the sideband: 0 for LSB, 1 for USB.\r\n"
		  ;

  if (!arg.help) {
    int m, s;
   if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%d%d", &m, &s);
      if (narg < 2) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusSB);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		/* int rtn = argus_setWIFswitches("s", m-1, s, 0); // NEEDS WORK ???
   			sprintf(status, "%sargus_setWIFswitches(s, %d, %d, 0) returned status %d.\r\n",
    					(rtn==0 ? statusOK : statusERR), m, s, rtn); */
    	} else {
    		sprintf(status, "%sReceiver number out of range\r\n", statusERR);
    	}
     }
  } else {
      // Command called without arguments; read gate values
      longHelp(status, usage, &Correlator::execArgusSB);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execArgusSB);
  }
}

/**
  \brief Argus: set all gate, drain biases and attenuations to a common value.

  Set all gate, drain biases and attenuations to a common value.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusSetAll(return_type status, argument_type arg)
{
  static const char *usage =
  "[KEYWORD VALUE]\r\n"
  "  Set choice of LNA gate/drain bias voltages or \r\n"
  "  receiver warm IF attenuations to a common value.\r\n"
  "  Keywords are:\r\n"
  "    G  gate [V].\r\n"
  "    D  drain [V].\r\n"
  "    A  attenuation [dB].\r\n"
  "    S  saddlebag amp power [1/0].\r\n"
  "  Value V or dB is the set value.\r\n"
		  ;

  if (!arg.help) {
    float v = 0.0;
    char inp[2] = {0};

    if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%1s%f", inp, &v);
      if (narg < 2) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusSetAll);
      } else if (!strcmp(inp, "a")) {
    	// Set atten
   		OSTimeDly(CMDDELAY);
        int rtn = dcm2_setAllAttens(v);
		sprintf(status, "%sdcm2_setAllAttens(%f) returned status %d.\r\n",
					(rtn==0 ? statusOK : statusERR), v, rtn);
      } else if (!strcmp(inp, "s")) {
      	// Set saddlebag amplifier state  /// zzz need to change from 1/0 to on/off
     		OSTimeDly(CMDDELAY);
          int rtn = sb_setAllAmps((int)v);
  		sprintf(status, "%ssb_setAllAmps(%d) returned status %d.\r\n",
  					(rtn==0 ? statusOK : statusERR), (int)v, rtn);
        } else {
        // Set G, D, M biases
    	OSTimeDly(CMDDELAY);
    	int rtn = argus_setAllBias(inp, v, 0);
    	if (rtn == -10) {
        	sprintf(status, "%sLNA cards are not powered, returned status %d.\r\n",
        			statusERR, rtn);
    	} else {
    		sprintf(status, "%sargus_setAllBias(%s, %f) returned status %d.\r\n",
    				(rtn==0 ? statusOK : statusERR), inp, v, rtn);
    	}
      }

  } else {
      // Command called without arguments
      longHelp(status, usage, &Correlator::execArgusSetAll);
     }
  } else {
    longHelp(status, usage, &Correlator::execArgusSetAll);
  }
}

/**
  \brief Argus cryostat monitoring.

  Monitor temperatures and aux inputs from the cryo board.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusCryo(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Read all cryostat monitor points, return values to screen.\r\n"
		  ;

  if (!arg.help) {
	OSTimeDly(CMDDELAY);
   	int rtn = argus_readThermADCs();
    sprintf(status, "%sCryostat:\r\n%s:%8.1f K\r\n%s:%8.1f K\r\n%s:%8.1f K\r\n"
    			    "%s:%8.1f K\r\n%s:%8.1f K\r\n%s:%8.1f K\r\n%s:%8.1e Torr (%4.3f V)\r\n",
    		(rtn==0 ? statusOK : statusERR), cnames[0], cryoPar.cryoTemps[0], cnames[1], cryoPar.cryoTemps[1],
    		cnames[2], cryoPar.cryoTemps[2], cnames[3], cryoPar.cryoTemps[3], cnames[4], cryoPar.cryoTemps[4],
    		cnames[5], cryoPar.cryoTemps[5], cnames[6],
    		(cryoPar.auxInputs[0] > 1. ? powf(10., cryoPar.auxInputs[0]-6.) : 0.),
    		cryoPar.auxInputs[0]);
  } else {
	longHelp(status, usage, &Correlator::execArgusCryo);
  }

}

/**
  \brief Use Argus LNA presets.

  Use Argus LNA presets from flash memory.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusPresets(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Set LNA biases to values stored in memory.\r\n"
  "  (see FLASH command to set).\r\n "
		  ;

  if (!arg.help) {
	flash_t flashData;
	zpec_readFlash(&flashData);
	OSTimeDly(CMDDELAY);
	int rtn = argus_LNApresets(&flashData);
    sprintf(status, "%sSet LNA to preset bias values, status %d\r\n",
    		(rtn==0 ? statusOK : statusERR), rtn);
  } else {
	longHelp(status, usage, &Correlator::execArgusPresets);
  }

}

/**
  \brief Argus LNA power control.

  Turn LNA power on and off for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusPwrCtrl(return_type status, argument_type arg)
{
  static const char *usage =
  "[STATE]\r\n"
  "  Sequence LNA power on or off, query LNA power supply.\r\n"
  "  STATE  ON or 1 to sequence LNA power on.\r\n"
  "         OFF or 0 to sequence LNA power off.\r\n"
  "  No argument returns power supply voltages at power control card.\r\n"
		  ;

  if (!arg.help) {
	  if (arg.str) {  // argument present, set new state.
		  char state[5] = {0};

      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%4s", state);
      if (narg < 1) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusPwrCtrl);
      } else {
        // Execute the command.
      	if (!strcmp(state, "1") || !strcasecmp(state, "ON")) {
      		OSTimeDly(CMDDELAY);
      		int rtn = argus_lnaPower(1);
            sprintf(status, "%sLNA power commanded on, status %d.\r\n",
                             (rtn==0 ? statusOK : statusERR), rtn);
      	}
      	else if (!strcmp(state, "0") || !strcasecmp(state, "OFF")) {
      		OSTimeDly(CMDDELAY);
      		int rtn = argus_lnaPower(0);
            sprintf(status, "%sLNA power commanded off, status %d.\r\n",
                             (rtn==0 ? statusOK : statusERR), rtn);
      	}
      	else {
      		longHelp(status, usage, &Correlator::execArgusPwrCtrl);
      	}
      }
    } else {
      // Command called without arguments; write LNA state
    	int rtn = 0;
    	if (lnaPwrState) {
    		OSTimeDly(CMDDELAY);
    		rtn += argus_readPwrADCs();
    		rtn += argus_readLNAbiasADCs("vg");
    		rtn += argus_readLNAbiasADCs("vd");
    		rtn += argus_readLNAbiasADCs("id");

    		sprintf(status, "%sLNA power state %s.\r\nSupplies: +15V: %5.2f V; "
				  "-15V: %5.2f V; +5V: %5.2f V\r\n"
				  "Voltages in [V], currents in [mA]\r\n\r\n"
	      			  "          1               2               3               4\r\n"
	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	      			  "          5               6               7               8\r\n"
	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	      			  "          9               10              11              12\r\n"
	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	      			  "          13              14              15              16\r\n"
	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	      			  "          17              18              19              20\r\n"
	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n",
	      			  (rtn==0 ? statusOK : statusERR), (lnaPwrState==1 ? "ON" : "OFF"),
	      			  pwrCtrlPar[2], pwrCtrlPar[1], pwrCtrlPar[0],  //pv, nv, vds
	      			  d2, rxPar[0].LNAmonPts[0], d2, rxPar[0].LNAmonPts[1], d2, rxPar[1].LNAmonPts[0], d2, rxPar[1].LNAmonPts[1],
	      			  d2, rxPar[2].LNAmonPts[0], d2, rxPar[2].LNAmonPts[1], d2, rxPar[3].LNAmonPts[0], d2, rxPar[3].LNAmonPts[1],
	      			  d2, rxPar[0].LNAmonPts[2], d2, rxPar[0].LNAmonPts[3], d2, rxPar[1].LNAmonPts[2], d2, rxPar[1].LNAmonPts[3],
	      			  d2, rxPar[2].LNAmonPts[2], d2, rxPar[2].LNAmonPts[3], d2, rxPar[3].LNAmonPts[2], d2, rxPar[3].LNAmonPts[3],
	      			  d1, rxPar[0].LNAmonPts[4], d1, rxPar[0].LNAmonPts[5], d1, rxPar[1].LNAmonPts[4], d1, rxPar[1].LNAmonPts[5],
	      			  d1, rxPar[2].LNAmonPts[4], d1, rxPar[2].LNAmonPts[5], d1, rxPar[3].LNAmonPts[4], d1, rxPar[3].LNAmonPts[5],

	      			  d2, rxPar[4].LNAmonPts[0], d2, rxPar[4].LNAmonPts[1], d2, rxPar[5].LNAmonPts[0], d2, rxPar[5].LNAmonPts[1],
	      			  d2, rxPar[6].LNAmonPts[0], d2, rxPar[6].LNAmonPts[1], d2, rxPar[7].LNAmonPts[0], d2, rxPar[7].LNAmonPts[1],
	      			  d2, rxPar[4].LNAmonPts[2], d2, rxPar[4].LNAmonPts[3], d2, rxPar[5].LNAmonPts[2], d2, rxPar[5].LNAmonPts[3],
	      			  d2, rxPar[6].LNAmonPts[2], d2, rxPar[6].LNAmonPts[3], d2, rxPar[7].LNAmonPts[2], d2, rxPar[7].LNAmonPts[3],
	      			  d1, rxPar[4].LNAmonPts[4], d1, rxPar[4].LNAmonPts[5], d1, rxPar[5].LNAmonPts[4], d1, rxPar[5].LNAmonPts[5],
	      			  d1, rxPar[6].LNAmonPts[4], d1, rxPar[6].LNAmonPts[5], d1, rxPar[7].LNAmonPts[4], d1, rxPar[7].LNAmonPts[5],

	      			  d2, rxPar[8].LNAmonPts[0],  d2, rxPar[8].LNAmonPts[1],  d2, rxPar[9].LNAmonPts[0],  d2, rxPar[9].LNAmonPts[1],
	      			  d2, rxPar[10].LNAmonPts[0], d2, rxPar[10].LNAmonPts[1], d2, rxPar[11].LNAmonPts[0], d2, rxPar[11].LNAmonPts[1],
	      			  d2, rxPar[8].LNAmonPts[2],  d2, rxPar[8].LNAmonPts[3],  d2, rxPar[9].LNAmonPts[2],  d2, rxPar[9].LNAmonPts[3],
	      			  d2, rxPar[10].LNAmonPts[2], d2, rxPar[10].LNAmonPts[3], d2, rxPar[11].LNAmonPts[2], d2, rxPar[11].LNAmonPts[3],
	      			  d1, rxPar[8].LNAmonPts[4],  d1, rxPar[8].LNAmonPts[5],  d1, rxPar[9].LNAmonPts[4],  d1, rxPar[9].LNAmonPts[5],
	      			  d1, rxPar[10].LNAmonPts[4], d1, rxPar[10].LNAmonPts[5], d1, rxPar[11].LNAmonPts[4], d1, rxPar[11].LNAmonPts[5],

	      			  d2, rxPar[12].LNAmonPts[0], d2, rxPar[12].LNAmonPts[1], d2, rxPar[13].LNAmonPts[0], d2, rxPar[13].LNAmonPts[1],
	      			  d2, rxPar[14].LNAmonPts[0], d2, rxPar[14].LNAmonPts[1], d2, rxPar[15].LNAmonPts[0], d2, rxPar[15].LNAmonPts[1],
	      			  d2, rxPar[12].LNAmonPts[2], d2, rxPar[12].LNAmonPts[3], d2, rxPar[13].LNAmonPts[2], d2, rxPar[13].LNAmonPts[3],
	      			  d2, rxPar[14].LNAmonPts[2], d2, rxPar[14].LNAmonPts[3], d2, rxPar[15].LNAmonPts[2], d2, rxPar[15].LNAmonPts[3],
	      			  d1, rxPar[12].LNAmonPts[4], d1, rxPar[12].LNAmonPts[5], d1, rxPar[13].LNAmonPts[4], d1, rxPar[13].LNAmonPts[5],
	      			  d1, rxPar[14].LNAmonPts[4], d1, rxPar[14].LNAmonPts[5], d1, rxPar[15].LNAmonPts[4], d1, rxPar[15].LNAmonPts[5],

	      			  d2, rxPar[16].LNAmonPts[0], d2, rxPar[16].LNAmonPts[1], d2, rxPar[17].LNAmonPts[0], d2, rxPar[17].LNAmonPts[1],
	      			  d2, rxPar[18].LNAmonPts[0], d2, rxPar[18].LNAmonPts[1], d2, rxPar[19].LNAmonPts[0], d2, rxPar[19].LNAmonPts[1],
	      			  d2, rxPar[16].LNAmonPts[2], d2, rxPar[16].LNAmonPts[3], d2, rxPar[17].LNAmonPts[2], d2, rxPar[17].LNAmonPts[3],
	      			  d2, rxPar[18].LNAmonPts[2], d2, rxPar[18].LNAmonPts[3], d2, rxPar[19].LNAmonPts[2], d2, rxPar[19].LNAmonPts[3],
	      			  d1, rxPar[16].LNAmonPts[4], d1, rxPar[16].LNAmonPts[5], d1, rxPar[17].LNAmonPts[4], d1, rxPar[17].LNAmonPts[5],
	      			  d1, rxPar[18].LNAmonPts[4], d1, rxPar[16].LNAmonPts[5], d1, rxPar[19].LNAmonPts[4], d1, rxPar[19].LNAmonPts[5]);
 		  } else {
 	    		rtn = argus_readPwrADCs();
		   		sprintf(status, "%sLNA power state %s.\r\nSupplies: +15V: %5.2f V; "
 						  "-15V: %5.2f V; +5V: %5.2f V\r\n",
 		    		  (rtn==0 ? statusOK : statusERR), (lnaPwrState==1 ? "ON" : "OFF"),
 		    		  pwrCtrlPar[2], pwrCtrlPar[1], pwrCtrlPar[0]);
 		  }
    }
  } else {
    longHelp(status, usage, &Correlator::execArgusPwrCtrl);
  }
}

/**
  \brief Read and display Argus monitor points.

  Read and display Argus monitor points.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusMonPts(return_type status, argument_type arg)
{
  static const char *usage =
		  "[KEYWORD]\r\n"
		  "  Read and display monitor and set points.\r\n"
		  "  KEYWORD:\r\n"
		  "    LNA for measured LNA bias values.\r\n"
		  "    SETS for LNA requested set points.\r\n"
		  "    POW for power supply values and card power monitor points.\r\n"
		  "    CRYO for cryostat monitor points.\r\n"
		  "    PRESETS for stored bias values.\r\n"
		  "  Empty keyword gives LNA bias values.\r\n"
		  ;

  int rtn = 0;
  if (!arg.help) {    // if no flag for help continue
	  if (arg.str) {  // if argument is present set new state.
		  char state[5] = {0};
      // Command called with an argument.
	      int narg = sscanf(arg.str, "%4s", state);
	      if (narg == 1) {
	    	  if (!strcasecmp(state, "lna")) {
	     		  if (lnaPwrState) {
	     			  OSTimeDly(CMDDELAY);
	     			  rtn += argus_readLNAbiasADCs("vg");
	    	      	  rtn += argus_readLNAbiasADCs("vd");
	    	      	  rtn += argus_readLNAbiasADCs("id");
	    	      	  rtn += argus_readPwrADCs();
	    	      	  sprintf(status, "%sLNA power state %s.\r\nSupplies: +15V: %5.2f V; "
	    	      			  "-15V: %5.2f V; +5V: %5.2f V\r\n"
	    	      			  "Voltages in [V], currents in [mA]\r\n\r\n"
	    	      			  "          1               2               3               4\r\n"
	    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "          5               6               7               8\r\n"
	    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "          9               10              11              12\r\n"
	    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "          13              14              15              16\r\n"
	    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "          17              18              19              20\r\n"
	    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n",
	    	      			  (rtn==0 ? statusOK : statusERR), (lnaPwrState==1 ? "ON" : "OFF"),
	    	      			  pwrCtrlPar[2], pwrCtrlPar[1], pwrCtrlPar[0],  //pv, nv, vds
	    	      			  d2, rxPar[0].LNAmonPts[0], d2, rxPar[0].LNAmonPts[1], d2, rxPar[1].LNAmonPts[0], d2, rxPar[1].LNAmonPts[1],
	    	      			  d2, rxPar[2].LNAmonPts[0], d2, rxPar[2].LNAmonPts[1], d2, rxPar[3].LNAmonPts[0], d2, rxPar[3].LNAmonPts[1],
	    	      			  d2, rxPar[0].LNAmonPts[2], d2, rxPar[0].LNAmonPts[3], d2, rxPar[1].LNAmonPts[2], d2, rxPar[1].LNAmonPts[3],
	    	      			  d2, rxPar[2].LNAmonPts[2], d2, rxPar[2].LNAmonPts[3], d2, rxPar[3].LNAmonPts[2], d2, rxPar[3].LNAmonPts[3],
	    	      			  d1, rxPar[0].LNAmonPts[4], d1, rxPar[0].LNAmonPts[5], d1, rxPar[1].LNAmonPts[4], d1, rxPar[1].LNAmonPts[5],
	    	      			  d1, rxPar[2].LNAmonPts[4], d1, rxPar[2].LNAmonPts[5], d1, rxPar[3].LNAmonPts[4], d1, rxPar[3].LNAmonPts[5],

	    	      			  d2, rxPar[4].LNAmonPts[0], d2, rxPar[4].LNAmonPts[1], d2, rxPar[5].LNAmonPts[0], d2, rxPar[5].LNAmonPts[1],
	    	      			  d2, rxPar[6].LNAmonPts[0], d2, rxPar[6].LNAmonPts[1], d2, rxPar[7].LNAmonPts[0], d2, rxPar[7].LNAmonPts[1],
	    	      			  d2, rxPar[4].LNAmonPts[2], d2, rxPar[4].LNAmonPts[3], d2, rxPar[5].LNAmonPts[2], d2, rxPar[5].LNAmonPts[3],
	    	      			  d2, rxPar[6].LNAmonPts[2], d2, rxPar[6].LNAmonPts[3], d2, rxPar[7].LNAmonPts[2], d2, rxPar[7].LNAmonPts[3],
	    	      			  d1, rxPar[4].LNAmonPts[4], d1, rxPar[4].LNAmonPts[5], d1, rxPar[5].LNAmonPts[4], d1, rxPar[5].LNAmonPts[5],
	    	      			  d1, rxPar[6].LNAmonPts[4], d1, rxPar[6].LNAmonPts[5], d1, rxPar[7].LNAmonPts[4], d1, rxPar[7].LNAmonPts[5],

	    	      			  d2, rxPar[8].LNAmonPts[0],  d2, rxPar[8].LNAmonPts[1],  d2, rxPar[9].LNAmonPts[0],  d2, rxPar[9].LNAmonPts[1],
	    	      			  d2, rxPar[10].LNAmonPts[0], d2, rxPar[10].LNAmonPts[1], d2, rxPar[11].LNAmonPts[0], d2, rxPar[11].LNAmonPts[1],
	    	      			  d2, rxPar[8].LNAmonPts[2],  d2, rxPar[8].LNAmonPts[3],  d2, rxPar[9].LNAmonPts[2],  d2, rxPar[9].LNAmonPts[3],
	    	      			  d2, rxPar[10].LNAmonPts[2], d2, rxPar[10].LNAmonPts[3], d2, rxPar[11].LNAmonPts[2], d2, rxPar[11].LNAmonPts[3],
	    	      			  d1, rxPar[8].LNAmonPts[4],  d1, rxPar[8].LNAmonPts[5],  d1, rxPar[9].LNAmonPts[4],  d1, rxPar[9].LNAmonPts[5],
	    	      			  d1, rxPar[10].LNAmonPts[4], d1, rxPar[10].LNAmonPts[5], d1, rxPar[11].LNAmonPts[4], d1, rxPar[11].LNAmonPts[5],

	    	      			  d2, rxPar[12].LNAmonPts[0], d2, rxPar[12].LNAmonPts[1], d2, rxPar[13].LNAmonPts[0], d2, rxPar[13].LNAmonPts[1],
	    	      			  d2, rxPar[14].LNAmonPts[0], d2, rxPar[14].LNAmonPts[1], d2, rxPar[15].LNAmonPts[0], d2, rxPar[15].LNAmonPts[1],
	    	      			  d2, rxPar[12].LNAmonPts[2], d2, rxPar[12].LNAmonPts[3], d2, rxPar[13].LNAmonPts[2], d2, rxPar[13].LNAmonPts[3],
	    	      			  d2, rxPar[14].LNAmonPts[2], d2, rxPar[14].LNAmonPts[3], d2, rxPar[15].LNAmonPts[2], d2, rxPar[15].LNAmonPts[3],
	    	      			  d1, rxPar[12].LNAmonPts[4], d1, rxPar[12].LNAmonPts[5], d1, rxPar[13].LNAmonPts[4], d1, rxPar[13].LNAmonPts[5],
	    	      			  d1, rxPar[14].LNAmonPts[4], d1, rxPar[14].LNAmonPts[5], d1, rxPar[15].LNAmonPts[4], d1, rxPar[15].LNAmonPts[5],

	    	      			  d2, rxPar[16].LNAmonPts[0], d2, rxPar[16].LNAmonPts[1], d2, rxPar[17].LNAmonPts[0], d2, rxPar[17].LNAmonPts[1],
	    	      			  d2, rxPar[18].LNAmonPts[0], d2, rxPar[18].LNAmonPts[1], d2, rxPar[19].LNAmonPts[0], d2, rxPar[19].LNAmonPts[1],
	    	      			  d2, rxPar[16].LNAmonPts[2], d2, rxPar[16].LNAmonPts[3], d2, rxPar[17].LNAmonPts[2], d2, rxPar[17].LNAmonPts[3],
	    	      			  d2, rxPar[18].LNAmonPts[2], d2, rxPar[18].LNAmonPts[3], d2, rxPar[19].LNAmonPts[2], d2, rxPar[19].LNAmonPts[3],
	    	      			  d1, rxPar[16].LNAmonPts[4], d1, rxPar[16].LNAmonPts[5], d1, rxPar[17].LNAmonPts[4], d1, rxPar[17].LNAmonPts[5],
	    	      			  d1, rxPar[18].LNAmonPts[4], d1, rxPar[16].LNAmonPts[5], d1, rxPar[19].LNAmonPts[4], d1, rxPar[19].LNAmonPts[5]);
	     		  } else {
	     			  	  sprintf(status, "%sNo report: LNA power is not on.\r\n", statusERR);
	     		  }
	    	  } else if (!strcasecmp(state, "sets")) {
	    		  if (1) {     //(lnaPwrState != 0) {
	    	      	  sprintf(status, "%sSet values:\r\n"
	    	      			  "Voltages in [V], currents in [mA]\r\n\r\n"
	    	      			  "         1               2               3               4\r\n"
	    	      			  "G: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "D: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "         5               6               7               8\r\n"
	    	      			  "G: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "D: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "         9               10              11              12\r\n"
	    	      			  "G: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "D: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "         13              14              15              16\r\n"
	    	      			  "G: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "D: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "         17              18              19              20\r\n"
	    	      			  "G: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "D: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n",

	    	      			  statusOK,

	    	      			  d2, rxPar[0].LNAsets[0], d2, rxPar[0].LNAsets[1], d2, rxPar[1].LNAsets[0], d2, rxPar[1].LNAsets[1],
	    	      			  d2, rxPar[2].LNAsets[0], d2, rxPar[2].LNAsets[1], d2, rxPar[3].LNAsets[0], d2, rxPar[3].LNAsets[1],
	    	      			  d2, rxPar[0].LNAsets[2], d2, rxPar[0].LNAsets[3], d2, rxPar[1].LNAsets[2], d2, rxPar[1].LNAsets[3],
	    	      			  d2, rxPar[2].LNAsets[2], d2, rxPar[2].LNAsets[3], d2, rxPar[3].LNAsets[2], d2, rxPar[3].LNAsets[3],

	    	      			  d2, rxPar[4].LNAsets[0], d2, rxPar[4].LNAsets[1], d2, rxPar[5].LNAsets[0], d2, rxPar[5].LNAsets[1],
	    	      			  d2, rxPar[6].LNAsets[0], d2, rxPar[6].LNAsets[1], d2, rxPar[7].LNAsets[0], d2, rxPar[7].LNAsets[1],
	    	      			  d2, rxPar[4].LNAsets[2], d2, rxPar[4].LNAsets[3], d2, rxPar[5].LNAsets[2], d2, rxPar[5].LNAsets[3],
	    	      			  d2, rxPar[6].LNAsets[2], d2, rxPar[6].LNAsets[3], d2, rxPar[7].LNAsets[2], d2, rxPar[7].LNAsets[3],

	    	      			  d2, rxPar[8].LNAsets[0], d2, rxPar[8].LNAsets[1], d2, rxPar[9].LNAsets[0], d2, rxPar[9].LNAsets[1],
	    	      			  d2, rxPar[10].LNAsets[0], d2, rxPar[10].LNAsets[1], d2, rxPar[11].LNAsets[0], d2, rxPar[11].LNAsets[1],
	    	      			  d2, rxPar[8].LNAsets[2], d2, rxPar[8].LNAsets[3], d2, rxPar[9].LNAsets[2], d2, rxPar[9].LNAsets[3],
	    	      			  d2, rxPar[10].LNAsets[2], d2, rxPar[10].LNAsets[3], d2, rxPar[11].LNAsets[2], d2, rxPar[11].LNAsets[3],

	    	      			  d2, rxPar[12].LNAsets[0], d2, rxPar[12].LNAsets[1], d2, rxPar[13].LNAsets[0], d2, rxPar[13].LNAsets[1],
	    	      			  d2, rxPar[14].LNAsets[0], d2, rxPar[14].LNAsets[1], d2, rxPar[15].LNAsets[0], d2, rxPar[15].LNAsets[1],
	    	      			  d2, rxPar[12].LNAsets[2], d2, rxPar[12].LNAsets[3], d2, rxPar[13].LNAsets[2], d2, rxPar[13].LNAsets[3],
	    	      			  d2, rxPar[14].LNAsets[2], d2, rxPar[14].LNAsets[3], d2, rxPar[15].LNAsets[2], d2, rxPar[15].LNAsets[3],

	    	      			  d2, rxPar[16].LNAsets[0], d2, rxPar[16].LNAsets[1], d2, rxPar[17].LNAsets[0], d2, rxPar[17].LNAsets[1],
	    	      			  d2, rxPar[18].LNAsets[0], d2, rxPar[18].LNAsets[1], d2, rxPar[19].LNAsets[0], d2, rxPar[19].LNAsets[1],
	    	      			  d2, rxPar[16].LNAsets[2], d2, rxPar[16].LNAsets[3], d2, rxPar[17].LNAsets[2], d2, rxPar[17].LNAsets[3],
	    	      			  d2, rxPar[18].LNAsets[2], d2, rxPar[18].LNAsets[3], d2, rxPar[19].LNAsets[2], d2, rxPar[19].LNAsets[3]);
	   	    		  } else {
	    			  sprintf(status, "%sNo report: LNA power is not on.\r\n", statusERR);
	              }
	    	 /* } else if (!strcasecmp(state, "wif")) {  /// NEEDS WORK ?????
      			  OSTimeDly(CMDDELAY);
	    	      rtn = argus_readWIF();         // update total power and temperature in status table
	    	      rtn += argus_readWIFpsADCs();  // update power supply voltage in status table
	    	      sprintf(status, "%sWarm IF:\r\n"
	    	    		  "Supply voltages: %5.2f V, %5.2f V\r\n"
	    	    		  "Ch    TotPow       Atten    SB    Card T\r\n"
	    	    		  " 1   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 2   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 3   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 4   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 5   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 6   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 7   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 8   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 9   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "10   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "11   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "12   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "13   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "14   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "15   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "16   %8.4f V    %2d dB    %1d    %5.1f C\r\n",
	    	    		  (rtn==0 ? statusOK : statusERR), wifPar.psv[0], wifPar.psv[1],
	    	    		  wifPar.totPow[0], wifPar.atten[0], wifPar.sb[0], wifPar.cardTemp[0],
	    	    		  wifPar.totPow[1], wifPar.atten[1], wifPar.sb[1], wifPar.cardTemp[1],
	    	    		  wifPar.totPow[2], wifPar.atten[2], wifPar.sb[2], wifPar.cardTemp[2],
	    	    		  wifPar.totPow[3], wifPar.atten[3], wifPar.sb[3], wifPar.cardTemp[3],
	    	    		  wifPar.totPow[4], wifPar.atten[4], wifPar.sb[4], wifPar.cardTemp[4],
	    	    		  wifPar.totPow[5], wifPar.atten[5], wifPar.sb[5], wifPar.cardTemp[5],
	    	    		  wifPar.totPow[6], wifPar.atten[6], wifPar.sb[6], wifPar.cardTemp[6],
	    	    		  wifPar.totPow[7], wifPar.atten[7], wifPar.sb[7], wifPar.cardTemp[7],
	    	    		  wifPar.totPow[8], wifPar.atten[8], wifPar.sb[8], wifPar.cardTemp[8],
	    	    		  wifPar.totPow[9], wifPar.atten[9], wifPar.sb[9], wifPar.cardTemp[9],
	    	    		  wifPar.totPow[10], wifPar.atten[10], wifPar.sb[10], wifPar.cardTemp[10],
	    	    		  wifPar.totPow[11], wifPar.atten[11], wifPar.sb[11], wifPar.cardTemp[11],
	    	    		  wifPar.totPow[12], wifPar.atten[12], wifPar.sb[12], wifPar.cardTemp[12],
	    	    		  wifPar.totPow[13], wifPar.atten[13], wifPar.sb[13], wifPar.cardTemp[13],
	    	    		  wifPar.totPow[14], wifPar.atten[14], wifPar.sb[14], wifPar.cardTemp[14],
	    	    		  wifPar.totPow[15], wifPar.atten[15], wifPar.sb[15], wifPar.cardTemp[15]);
	    	    		  */
	    	  } else if (!strcasecmp(state, "cryo")) {
	    		  OSTimeDly(CMDDELAY);
	    		  rtn = argus_readThermADCs();
	    		  sprintf(status, "%sCryostat:\r\n%s:%8.1f K\r\n%s:%8.1f K\r\n%s:%8.1f K\r\n"
	    		    			   "%s:%8.1f K\r\n%s:%8.1f K\r\n%s:%8.1f K\r\n%s:%8.1e Torr (%4.3f V)\r\n",
	    		    	    (rtn==0 ? statusOK : statusERR), cnames[0], cryoPar.cryoTemps[0], cnames[1], cryoPar.cryoTemps[1],
	    		    		cnames[2], cryoPar.cryoTemps[2], cnames[3], cryoPar.cryoTemps[3], cnames[4], cryoPar.cryoTemps[4],
	    		    		cnames[5], cryoPar.cryoTemps[5],
	    		    		cnames[6], (cryoPar.auxInputs[0] > 1 ? powf(10., cryoPar.auxInputs[0]-6.) : 0.),
	    		    		cryoPar.auxInputs[0]);
	    	  } else if (!strcasecmp(state, "pow")) {
	    		  OSTimeDly(CMDDELAY);
	    		  rtn = argus_readPwrADCs();
	    		  rtn += argus_readBCpsV();
	    		  sprintf(status, "%sPower control card:\r\n"
	    				  "Analog +15V:     %5.2f V;  -15V:     %5.2f V\r\n"
	    				  "Digital +5V:     %5.2f V;  Drains:    %5.2f V\r\n"
	    				  "Chassis temp.:   %5.1f C\r\n\r\n"
	    				  "Bias card power monitor points in [V]:\r\n"
	    				  "         Card A         Card B          Card C         Card D         Card E\r\n"
    	      			  "+15: % 4.2f % 4.2f, % 4.2f, % 4.2f, % 4.2f % 4.2f, % 4.2f % 4.2f, % 4.2f % 4.2f\r\n"
    	      			  "-15: % 4.2f % 4.2f, % 4.2f, % 4.2f, % 4.2f % 4.2f, % 4.2f % 4.2f, % 4.2f % 4.2f\r\n"
    	      			  "VCC: % 6.2f % 6.2f, % 6.2f, % 6.2f, % 6.2f % 6.2f, % 6.2f % 6.2f, % 6.2f % 6.2f\r\n"
    	      			  "VDS: % 6.2f % 6.2f, % 6.2f, % 6.2f, % 6.2f % 6.2f, % 6.2f % 6.2f, % 6.2f % 6.2f\r\n\r\n",
	    				  (rtn==0 ? statusOK : statusERR), pwrCtrlPar[2], pwrCtrlPar[1],
	    				  pwrCtrlPar[3], pwrCtrlPar[0],
	    				  pwrCtrlPar[8],
	    				  bcPar[0].v[0], bcPar[0].v[1], bcPar[1].v[0], bcPar[1].v[1],
	    				  bcPar[2].v[0], bcPar[2].v[1], bcPar[3].v[0], bcPar[3].v[1],
	    				  bcPar[4].v[0], bcPar[4].v[1],
	    				  bcPar[0].v[2], bcPar[0].v[3], bcPar[1].v[2], bcPar[1].v[3],
	    				  bcPar[2].v[2], bcPar[2].v[3], bcPar[3].v[2], bcPar[3].v[3],
	    				  bcPar[4].v[2], bcPar[4].v[3],
	    				  bcPar[0].v[4], bcPar[0].v[5], bcPar[1].v[4], bcPar[1].v[5],
	    				  bcPar[2].v[4], bcPar[2].v[5], bcPar[3].v[4], bcPar[3].v[5],
	    				  bcPar[4].v[4], bcPar[4].v[5],
	    				  bcPar[0].v[6], bcPar[0].v[7], bcPar[1].v[6], bcPar[1].v[7],
	    				  bcPar[2].v[6], bcPar[2].v[7], bcPar[3].v[6], bcPar[3].v[7],
	    				  bcPar[4].v[6], bcPar[4].v[7] );
    	  } else if (!strcasecmp(state, "pres")) {
    		  flash_t flashData;
    		  zpec_readFlash(&flashData);
	      	  sprintf(status, "%sStored bias values.  Voltages in [V]\r\n\r\n"
	      			  "          1               2               3               4\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  //"A : %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  "          5               6               7               8\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  //"A : %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  "          9               10              11              12\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  //"A : %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  "          13              14              15              16\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  //"A : %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  "          17              18              19              20\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n",
	      			  //"A : %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n",
	      			  statusOK,
	      			  flashData.lnaGsets[0], flashData.lnaGsets[1], flashData.lnaGsets[2], flashData.lnaGsets[3],
	      			  flashData.lnaGsets[4], flashData.lnaGsets[5], flashData.lnaGsets[6], flashData.lnaGsets[7],
	      			  flashData.lnaDsets[0], flashData.lnaDsets[1], flashData.lnaDsets[2], flashData.lnaDsets[3],
	      			  flashData.lnaDsets[4], flashData.lnaDsets[5], flashData.lnaDsets[6], flashData.lnaDsets[7],
	      			  //(float)flashData.atten[0]/2., (float)flashData.atten[1]/2., (float)flashData.atten[2]/2., (float)flashData.atten[3]/2.,
	      			  //(float)flashData.atten[4]/2., (float)flashData.atten[5]/2., (float)flashData.atten[6]/2., (float)flashData.atten[7]/2.,

	      			  flashData.lnaGsets[8], flashData.lnaGsets[9], flashData.lnaGsets[10], flashData.lnaGsets[11],
	      			  flashData.lnaGsets[12], flashData.lnaGsets[13], flashData.lnaGsets[14], flashData.lnaGsets[15],
	      			  flashData.lnaDsets[8], flashData.lnaDsets[9], flashData.lnaDsets[10], flashData.lnaDsets[11],
	      			  flashData.lnaDsets[12], flashData.lnaDsets[13], flashData.lnaDsets[14], flashData.lnaDsets[15],
	      			  //(float)flashData.atten[8]/2., (float)flashData.atten[9]/2., (float)flashData.atten[10]/2., (float)flashData.atten[11]/2.,
	      			  //(float)flashData.atten[12]/2., (float)flashData.atten[13]/2., (float)flashData.atten[14]/2., (float)flashData.atten[15]/2.,
			  
	      			  flashData.lnaGsets[16], flashData.lnaGsets[17], flashData.lnaGsets[18], flashData.lnaGsets[19],
	      			  flashData.lnaGsets[20], flashData.lnaGsets[21], flashData.lnaGsets[22], flashData.lnaGsets[23],
	      			  flashData.lnaDsets[16], flashData.lnaDsets[17], flashData.lnaDsets[18], flashData.lnaDsets[19],
	      			  flashData.lnaDsets[20], flashData.lnaDsets[21], flashData.lnaDsets[22], flashData.lnaDsets[23],
	      			  //(float)flashData.atten[16]/2., (float)flashData.atten[17]/2., (float)flashData.atten[18]/2., (float)flashData.atten[19]/2.,
	      			  //(float)flashData.atten[20]/2., (float)flashData.atten[21]/2., (float)flashData.atten[22]/2., (float)flashData.atten[23]/2.,
			  
	      			  flashData.lnaGsets[24], flashData.lnaGsets[25], flashData.lnaGsets[26], flashData.lnaGsets[27],
	      			  flashData.lnaGsets[28], flashData.lnaGsets[29], flashData.lnaGsets[30], flashData.lnaGsets[31],
	      			  flashData.lnaDsets[24], flashData.lnaDsets[25], flashData.lnaDsets[26], flashData.lnaDsets[27],
	      			  flashData.lnaDsets[28], flashData.lnaDsets[29], flashData.lnaDsets[30], flashData.lnaDsets[31],
	      			  //(float)flashData.atten[24]/2., (float)flashData.atten[25]/2., (float)flashData.atten[26]/2., (float)flashData.atten[27]/2.,
	      			  //(float)flashData.atten[28]/2., (float)flashData.atten[29]/2., (float)flashData.atten[30]/2., (float)flashData.atten[31]/2.,

	      			  flashData.lnaGsets[32], flashData.lnaGsets[33], flashData.lnaGsets[34], flashData.lnaGsets[35],
	      			  flashData.lnaGsets[36], flashData.lnaGsets[37], flashData.lnaGsets[38], flashData.lnaGsets[39],
	      			  flashData.lnaDsets[32], flashData.lnaDsets[33], flashData.lnaDsets[34], flashData.lnaDsets[35],
	      			  flashData.lnaDsets[36], flashData.lnaDsets[37], flashData.lnaDsets[38], flashData.lnaDsets[39]);
	      			  //(float)flashData.atten[32]/2., (float)flashData.atten[33]/2., (float)flashData.atten[34]/2., (float)flashData.atten[35]/2.,
	      			  //(float)flashData.atten[36]/2., (float)flashData.atten[37]/2., (float)flashData.atten[38]/2., (float)flashData.atten[39]/2.);

		      } else { // no valid argument; list options
		    	  longHelp(status, usage, &Correlator::execArgusMonPts);
		      }
	      }
	  } else {	 // no argument given; show lna mon points
		  if (lnaPwrState) {
			  OSTimeDly(CMDDELAY);
			  rtn += argus_readPwrADCs();
			  rtn += argus_readLNAbiasADCs("vg");
			  rtn += argus_readLNAbiasADCs("vd");
			  rtn += argus_readLNAbiasADCs("id");

			  sprintf(status, "%sLNA power state %s.\r\nSupplies: +15V: %5.2f V; "
			"-15V: %5.2f V; +5V: %5.2f V\r\n"
					  "Voltages in [V], currents in [mA]\r\n\r\n"
    	      			  "          1               2               3               4\r\n"
    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
    	      			  "          5               6               7               8\r\n"
    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
    	      			  "          9               10              11              12\r\n"
    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
    	      			  "          13              14              15              16\r\n"
    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
    	      			  "          17              18              19              20\r\n"
    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n",
    	      			  (rtn==0 ? statusOK : statusERR), (lnaPwrState==1 ? "ON" : "OFF"),
    	      			  pwrCtrlPar[2], pwrCtrlPar[1], pwrCtrlPar[0],  //pv, nv, vds
    	      			  d2, rxPar[0].LNAmonPts[0], d2, rxPar[0].LNAmonPts[1], d2, rxPar[1].LNAmonPts[0], d2, rxPar[1].LNAmonPts[1],
    	      			  d2, rxPar[2].LNAmonPts[0], d2, rxPar[2].LNAmonPts[1], d2, rxPar[3].LNAmonPts[0], d2, rxPar[3].LNAmonPts[1],
    	      			  d2, rxPar[0].LNAmonPts[2], d2, rxPar[0].LNAmonPts[3], d2, rxPar[1].LNAmonPts[2], d2, rxPar[1].LNAmonPts[3],
    	      			  d2, rxPar[2].LNAmonPts[2], d2, rxPar[2].LNAmonPts[3], d2, rxPar[3].LNAmonPts[2], d2, rxPar[3].LNAmonPts[3],
    	      			  d1, rxPar[0].LNAmonPts[4], d1, rxPar[0].LNAmonPts[5], d1, rxPar[1].LNAmonPts[4], d1, rxPar[1].LNAmonPts[5],
    	      			  d1, rxPar[2].LNAmonPts[4], d1, rxPar[2].LNAmonPts[5], d1, rxPar[3].LNAmonPts[4], d1, rxPar[3].LNAmonPts[5],

    	      			  d2, rxPar[4].LNAmonPts[0], d2, rxPar[4].LNAmonPts[1], d2, rxPar[5].LNAmonPts[0], d2, rxPar[5].LNAmonPts[1],
    	      			  d2, rxPar[6].LNAmonPts[0], d2, rxPar[6].LNAmonPts[1], d2, rxPar[7].LNAmonPts[0], d2, rxPar[7].LNAmonPts[1],
    	      			  d2, rxPar[4].LNAmonPts[2], d2, rxPar[4].LNAmonPts[3], d2, rxPar[5].LNAmonPts[2], d2, rxPar[5].LNAmonPts[3],
    	      			  d2, rxPar[6].LNAmonPts[2], d2, rxPar[6].LNAmonPts[3], d2, rxPar[7].LNAmonPts[2], d2, rxPar[7].LNAmonPts[3],
    	      			  d1, rxPar[4].LNAmonPts[4], d1, rxPar[4].LNAmonPts[5], d1, rxPar[5].LNAmonPts[4], d1, rxPar[5].LNAmonPts[5],
    	      			  d1, rxPar[6].LNAmonPts[4], d1, rxPar[6].LNAmonPts[5], d1, rxPar[7].LNAmonPts[4], d1, rxPar[7].LNAmonPts[5],

    	      			  d2, rxPar[8].LNAmonPts[0],  d2, rxPar[8].LNAmonPts[1],  d2, rxPar[9].LNAmonPts[0],  d2, rxPar[9].LNAmonPts[1],
    	      			  d2, rxPar[10].LNAmonPts[0], d2, rxPar[10].LNAmonPts[1], d2, rxPar[11].LNAmonPts[0], d2, rxPar[11].LNAmonPts[1],
    	      			  d2, rxPar[8].LNAmonPts[2],  d2, rxPar[8].LNAmonPts[3],  d2, rxPar[9].LNAmonPts[2],  d2, rxPar[9].LNAmonPts[3],
    	      			  d2, rxPar[10].LNAmonPts[2], d2, rxPar[10].LNAmonPts[3], d2, rxPar[11].LNAmonPts[2], d2, rxPar[11].LNAmonPts[3],
    	      			  d1, rxPar[8].LNAmonPts[4],  d1, rxPar[8].LNAmonPts[5],  d1, rxPar[9].LNAmonPts[4],  d1, rxPar[9].LNAmonPts[5],
    	      			  d1, rxPar[10].LNAmonPts[4], d1, rxPar[10].LNAmonPts[5], d1, rxPar[11].LNAmonPts[4], d1, rxPar[11].LNAmonPts[5],

    	      			  d2, rxPar[12].LNAmonPts[0], d2, rxPar[12].LNAmonPts[1], d2, rxPar[13].LNAmonPts[0], d2, rxPar[13].LNAmonPts[1],
    	      			  d2, rxPar[14].LNAmonPts[0], d2, rxPar[14].LNAmonPts[1], d2, rxPar[15].LNAmonPts[0], d2, rxPar[15].LNAmonPts[1],
    	      			  d2, rxPar[12].LNAmonPts[2], d2, rxPar[12].LNAmonPts[3], d2, rxPar[13].LNAmonPts[2], d2, rxPar[13].LNAmonPts[3],
    	      			  d2, rxPar[14].LNAmonPts[2], d2, rxPar[14].LNAmonPts[3], d2, rxPar[15].LNAmonPts[2], d2, rxPar[15].LNAmonPts[3],
    	      			  d1, rxPar[12].LNAmonPts[4], d1, rxPar[12].LNAmonPts[5], d1, rxPar[13].LNAmonPts[4], d1, rxPar[13].LNAmonPts[5],
    	      			  d1, rxPar[14].LNAmonPts[4], d1, rxPar[14].LNAmonPts[5], d1, rxPar[15].LNAmonPts[4], d1, rxPar[15].LNAmonPts[5],

    	      			  d2, rxPar[16].LNAmonPts[0], d2, rxPar[16].LNAmonPts[1], d2, rxPar[17].LNAmonPts[0], d2, rxPar[17].LNAmonPts[1],
    	      			  d2, rxPar[18].LNAmonPts[0], d2, rxPar[18].LNAmonPts[1], d2, rxPar[19].LNAmonPts[0], d2, rxPar[19].LNAmonPts[1],
    	      			  d2, rxPar[16].LNAmonPts[2], d2, rxPar[16].LNAmonPts[3], d2, rxPar[17].LNAmonPts[2], d2, rxPar[17].LNAmonPts[3],
    	      			  d2, rxPar[18].LNAmonPts[2], d2, rxPar[18].LNAmonPts[3], d2, rxPar[19].LNAmonPts[2], d2, rxPar[19].LNAmonPts[3],
    	      			  d1, rxPar[16].LNAmonPts[4], d1, rxPar[16].LNAmonPts[5], d1, rxPar[17].LNAmonPts[4], d1, rxPar[17].LNAmonPts[5],
    	      			  d1, rxPar[18].LNAmonPts[4], d1, rxPar[16].LNAmonPts[5], d1, rxPar[19].LNAmonPts[4], d1, rxPar[19].LNAmonPts[5]);
     		  } else {
    			  sprintf(status, "%sNo report: LNA power is not on.\r\n", statusERR);
    		  }
	      }
  } else { // help string requested
	  longHelp(status, usage, &Correlator::execArgusMonPts);
  }
 }


/**
  \brief COMAP jlna command.

  This method returns the LNA monitor points in JSON format.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execCOMAPjlna(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Return LNA monitor point values in JSON format.\r\n";

  if (!arg.help && !arg.str) {
/*	    short i, j, k;
		for (i=0; i<NRX; i++) {
	      for (j=0; j<NSTAGES; j++){
	    	  k = i*NSTAGES + j;  // index within rxPar.lnaXsets vector
	    	  // gates, if value is within limits
	          flashData.lnaGsets[k] = rxPar[i].LNAsets[j];
	    	  // drains, if value is within limits
	    		  flashData.lnaDsets[k] = rxPar[i].LNAsets[j+NSTAGES];
	      }
	    }
*/
	  sprintf(status, "%sStub for jlna.\r\n", statusOK);
  } else {
    	longHelp(status, usage, &Correlator::execCOMAPjlna);
  }
}

/**
  \brief COMAP jcryo command.

  This method returns the LNA monitor points in JSON format.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execCOMAPjcryo(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Return cryostat monitor point values in JSON format.\r\n";

  if (!arg.help && !arg.str) {
	  int rtn = argus_readThermADCs();
	  sprintf(status, "{\"cryostat\":{\"dataOK\":%s, \"temps\":"
			  "[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], \"press\":[%.1f]}}\r\n",
	    	    (rtn==0 ? "true" : "false"), cryoPar.cryoTemps[0], cryoPar.cryoTemps[1],
	    		cryoPar.cryoTemps[2], cryoPar.cryoTemps[3], cryoPar.cryoTemps[4],
	    		cryoPar.cryoTemps[5],
	    		(cryoPar.auxInputs[0] > 1 ? powf(10., cryoPar.auxInputs[0]-6.) : 0.));
  } else {
    	longHelp(status, usage, &Correlator::execCOMAPjcryo);
  }
}

/**
  \brief DCM2 control.

  This method controls DCM2 and readouts.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execDCM2(return_type status, argument_type arg)
{
	  static const char *usage =
	  "[KEYWORD VALUE [VALUE]]\r\n"
      "  DCM2 commands.\r\n"
      "    KEYWORD   VALUE    VALUE:\r\n"
	  "    amps      on/off             turns amplifier power on/off\r\n"
	  "    led       on/off             turns led on/off\r\n"
	  "    block     ch_no    A/B       blocks DCM2 channel, band A or B\r\n" ;

  int rtn = 0;

  if (!arg.help) {
	  if (arg.str) {
	  // Command called with one or more arguments.
	  char kw[10] = {0};
	  char val[4] = {0};
	  char val2[4] = {0};
	  int narg = sscanf(arg.str, "%9s %3s %3s", kw, val, val2);

	  if (narg == 2) {
	      // Execute the command.
	      if (!strcasecmp(kw, "amps")) {
  		    rtn = dcm2_ampPow(val);
	        sprintf(status, "%sdcm2_ampPow(%s) returned with status %d\r\n",
					(!rtn ? statusOK : statusERR), val, rtn);
	      } else if (!strcasecmp(kw, "led")) {
	  		    rtn = dcm2_ledOnOff(val);
		        sprintf(status, "%sdcm2_ledOnOff(%s) returned with status %d\r\n",
						(!rtn ? statusOK : statusERR), val, rtn);
	      } else {
		      longHelp(status, usage, &Correlator::execDCM2);
	      }
	  } else if (narg == 3){
		  if (!strcasecmp(kw, "block")) {
			  rtn = dcm2_blockMod(val, val2);
			  sprintf(status, "%sdcm2_blockMod(%s, %s) returned with status %d\r\n",
			  		  (!rtn ? statusOK : statusERR), val, val2, rtn);
		  } else {
			  longHelp(status, usage, &Correlator::execDCM2);
		  }
	  } else {
		  longHelp(status, usage, &Correlator::execDCM2);
	  }
	} else {
      rtn = dcm2_readMBadc();
      rtn += dcm2_readMBtemp();
      rtn += dcm2_readAllModTemps();
      rtn += dcm2_readAllModTotPwr();

      // write output: header, channel reports, then an extra line

      char outStr[2048] = {0};
      int n = 0;
      int i;
      n = sprintf(&outStr[0],
    		  "%sDCM2 parameters:\r\n"
    		  //"%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f \r\n"
    		  "DCM2 7 & 12 V supply voltages: %.1f V, %.1f V, fanout board temp.: %.1f C\r\n"
    		  "4 GHz PLL: %s, 8 GHz PLL: %s\r\n"
    		  "Individual DCM2 modules:\r\n"
    		  "                 Band A               |           Band B\r\n"
    		  "      Bl AttI AttQ  TPwI  TPwQ   T[C] |Bl AttI AttQ  TPwI  TPwQ   T[C]\r\n",
    		  (!rtn ? statusOK : statusERR),
    		  //dcm2MBpar[0], dcm2MBpar[1], dcm2MBpar[2], dcm2MBpar[3], dcm2MBpar[4], dcm2MBpar[5], dcm2MBpar[6], dcm2MBpar[7],
    		  dcm2MBpar[5], dcm2MBpar[4], dcm2MBpar[7],
    		  (dcm2MBpar[2] > PLLLOCKTHRESH ? "locked" : "***UNLOCKED***"),
    		  (dcm2MBpar[3] > PLLLOCKTHRESH ? "locked" : "***UNLOCKED***"));
      for (i=0; i<NRX; i++) {
    	  n += sprintf(&outStr[n],
		     "Ch %2d: %d %4.1f %4.1f %5.1f %5.1f %6.2f | %d %4.1f %4.1f %5.1f %5.1f %6.2f\r\n",
		     i+1, dcm2Apar.status[i], 
		     (float)dcm2Apar.attenI[i]/2., (float)dcm2Apar.attenQ[i]/2.,
		     dcm2Apar.powDetI[i], dcm2Apar.powDetQ[i], 
		     dcm2Apar.bTemp[i],
		     dcm2Bpar.status[i], 
		     (float)dcm2Bpar.attenI[i]/2., (float)dcm2Bpar.attenQ[i]/2.,
		     dcm2Bpar.powDetI[i], dcm2Bpar.powDetQ[i], 
		     dcm2Bpar.bTemp[i]);
      }
      n += sprintf(&outStr[n], "\r\n");
      sprintf(status, outStr);  // send it out
	}
  } else {
	  longHelp(status, usage, &Correlator::execDCM2);
  }
}

/**
  \brief Saddlebag control.

  This method controls the saddlebag card contols and readouts.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execSaddlebag(return_type status, argument_type arg)
{
	  static const char *usage =
	  "[KEYWORD VALUE [VALUE]]\r\n"
      "  Saddlebag commands.\r\n"
      "    KEYWORD  VALUE  VALUE:\r\n"
	  "    amps     m      on/off    turns amplifier power for saddlebag m on/off\r\n"
	  "    led      m      on/off    turns led for saddlebag m on/off\r\n";

  int rtn = 0;

  if (!arg.help) {
	  if (arg.str) {
	  // Command called with one or more arguments.
	  char kw[10] = {0};
	  int val;
	  char val2[4] = {0};
	  int narg = sscanf(arg.str, "%9s %d %3s", kw, &val, val2);

	  if (narg == 3) {
	      // Check for valid saddlebag index number; this is the only place this happens
		  if (val > NSBG || val < 0) {
			  sprintf(status, "*** Error: invalid saddlebag number ***\r\n");
		  }
		  val -= 1; // convert from human to index

	      if (!strcasecmp(kw, "amps")) {
	    	  rtn = sb_ampPow(val2, val);
	    	  sprintf(status, "%ssb_ampPow(%s) returned with status %d\r\n",
	    			  (!rtn ? statusOK : statusERR), val2, rtn);
	      } else if (!strcasecmp(kw, "led")) {
	    	  rtn = sb_ledOnOff(val2, val);
	    	  sprintf(status, "%ssb_ledOnOff(%s) returned with status %d\r\n",
	    			  (!rtn ? statusOK : statusERR), val2, rtn);
	      } else {
	    	  longHelp(status, usage, &Correlator::execSaddlebag);
	      }
	  } else {
		  longHelp(status, usage, &Correlator::execSaddlebag);
	  }
	} else {
	  rtn = 0;
	  int i;

      for (i=0; i<4; i++) {
    	  rtn += sb_readADC(i);
    	  sbPar[i].pll = sb_readPLLmon(i);
      }
      sprintf(status, "%sSaddlebags:   (status %d)\r\n"
    		  "%s: %.1f, %.1f, %.1f, %.1f\r\n"
    		  "%s: %.1f, %.1f, %.1f, %.1f\r\n"
    		  "%s: %.1f, %.1f, %.1f, %.1f\r\n"
    		  "%s: %.1f, %.1f, %.1f, %.1f\r\n"
    		  "%s: %.1f, %.1f, %.1f, %.1f\r\n"
    		  "%s: %.1f, %.1f, %.1f, %.1f\r\n"
    		  "%s: %.1f, %.1f, %.1f, %.1f\r\n"
    		  "%s: %.1f, %.1f, %.1f, %.1f\r\n"
    		  "%s: %u, %u, %u, %u\r\n"
    		  "%s: %u, %u, %u, %u\r\n",
    	  	  (!rtn ? statusOK : statusERR), rtn,
    	  	  sbnames[0], sbPar[0].adcv[0], sbPar[1].adcv[0], sbPar[2].adcv[0], sbPar[3].adcv[0],
    	  	  sbnames[1], sbPar[0].adcv[1], sbPar[1].adcv[1], sbPar[2].adcv[1], sbPar[3].adcv[1],
    	  	  sbnames[2], sbPar[0].adcv[2], sbPar[1].adcv[2], sbPar[2].adcv[2], sbPar[3].adcv[2],
    	  	  sbnames[3], sbPar[0].adcv[3], sbPar[1].adcv[3], sbPar[2].adcv[3], sbPar[3].adcv[3],
    	  	  sbnames[4], sbPar[0].adcv[4], sbPar[1].adcv[4], sbPar[2].adcv[4], sbPar[3].adcv[4],
    	  	  sbnames[5], sbPar[0].adcv[5], sbPar[1].adcv[5], sbPar[2].adcv[5], sbPar[3].adcv[5],
    	  	  sbnames[6], sbPar[0].adcv[6], sbPar[1].adcv[6], sbPar[2].adcv[6], sbPar[3].adcv[6],
    	  	  sbnames[7], sbPar[0].adcv[7], sbPar[1].adcv[7], sbPar[2].adcv[7], sbPar[3].adcv[7],
    	  	  sbnames[8], sbPar[0].pll, sbPar[1].pll, sbPar[2].pll, sbPar[3].pll,
    	  	  sbnames[9], sbPar[0].ampPwr, sbPar[1].ampPwr, sbPar[2].ampPwr, sbPar[3].ampPwr);
	}
  } else {
	  longHelp(status, usage, &Correlator::execSaddlebag);
  }
}

/*************************************************************************************/
/**
  \brief Argus lock test command.

  This method tests the lockout scheme.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/

void Correlator::execArgusLock(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n";

  int rtn;

  if (!arg.help) {
	  int busy = 1;  // i2cBusBusy value for all tests
	  int lnaps = 0; // LNA power state; set to 1 to reach bus lock tests for LNA, 0 for CIF

	  i2cBusBusy = busy;
	  rtn = argus_readAllSystemADCs();
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readAllSystemADCs()\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readPwrADCs();
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readPwrADCs();\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readBCpsV();
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readBCpsV();\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readThermADCs();
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readThermADCs();\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readLNAbiasADCs("vg");
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readLNAbiasADCs(vg);\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readLNAbiasADCs("vd");
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readLNAbiasADCs(vd);\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readLNAbiasADCs("id");
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readLNAbiasADCs(id);\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readLNAbiasADCs("vm");
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readLNAbiasADCs(vm);\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readLNAbiasADCs("im");
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readLNAbiasADCs(im);\r\n", i2cBusBusy, rtn);

	  lnaPwrState = lnaps;
	  i2cBusBusy = busy;
	  rtn = argus_setLNAbias("d", 2, 1, .5, 0);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_setLNAbias(d, 2, 1, .5, 0)\r\n", i2cBusBusy, rtn);

	  lnaPwrState = lnaps;
	  i2cBusBusy = busy;
	  rtn = argus_setLNAbias("m", 2, 1, .5, 0);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_setLNAbias(m, 2, 1, .5, 0)\r\n", i2cBusBusy, rtn);

	  lnaPwrState = lnaps;
	  i2cBusBusy = busy;
	  rtn = argus_setAllBias("d", 0.5, 0);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_setAllBias(d, 0.5, 0)\r\n", i2cBusBusy, rtn);

	  lnaPwrState = lnaps;
	  i2cBusBusy = busy;
	  rtn = argus_lnaPower(1);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_lnaPower(1)\r\n", i2cBusBusy, rtn);

	  sprintf(status, "# I2C bus lock test results output to UART0.\r\n");

  } else {
    longHelp(status, usage, &Correlator::execArgusLock);
  }
}
