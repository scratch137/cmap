/**
  \file
  \author Andy Harris
  \brief Argus control command infrastructure.

  $Id: argus_control.cpp,v 1.2 2014/06/04 18:35:21 harris Exp $
*/

#include "argus.h"
#include "control.h"
#include "math.h"

// temporary strings for JSON output work
char outStr[8192] = {0};
char str0[512] = {0};
char str1[512] = {0};
char str2[512] = {0};
char str3[512] = {0};
char str4[512] = {0};
char str5[512] = {0};
char str6[512] = {0};
char str7[512] = {0};
char str8[512] = {0};
char str9[512] = {0};

// names for cryostat test points
//char *cnames[] = {"T0", "T1", "T2", "T3", "T4", "T5", "Pressure"};
char *cnames[] = {"T_stage_1 ",
		          "T_stage_2 ",
		          "T_bulkhead",
		          "T_plate_2 ",
		          "T_pixel_1 ",
		          "T_pixel_2 ",
		          "Pressure  "};

// names for saddlebag test points; names in JSaddlebag should match these
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

// names for vane test points; names in JSaddlebag should match these
char *vnames[] = {"Vin    [V]",
                   "NC        ",
                   "NC        ",
                   "NC        ",
		           "Angle     ",
		           "T_load [C]",
		           "T_outs [C]",
		           "T_shrd [C]",
				   "Vane pos. ",
                   "          "};

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
  \brief Argus freeze command, JSON return.

  This method sets a bit to freeze the system state, generally meant for during integrations.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJArgusFreeze(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Freeze system state (see thaw) to prevent changes to settings.\r\n";


  if (!arg.help && !arg.str) {
	  freezeSys = 1;
	  freezeCtr += 1;
	  sprintf(status, "{\"freeze\": {\"cmdOK\":true}}\r\n");
  } else {
	    longHelp(status, usage, &Correlator::execJArgusFreeze);
  }
}

/**
  \brief Argus thaw command.

  This method clears a bit to unfreeze the system state, generally meant for not during integrations.
  JSON return

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
  \brief Argus thaw command, JSON return.

  This method clears a bit to unfreeze the system state, generally meant for not during integrations.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJArgusThaw(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Thaw system state (see freeze) to permit changes to settings.\r\n";

  if (!arg.help && !arg.str) {
	  freezeSys = 0;
	  thawCtr += 1;
	  sprintf(status, "{\"thaw\": {\"cmdOK\":true}}\r\n");
  } else {
    	longHelp(status, usage, &Correlator::execJArgusThaw);
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
	  int rtnTherm = argus_thermCheck();
	  int rtnRx = argus_biasCheck();
	  sprintf(status, "%sState and error flags:\r\n"
			  "System status 0x%04x\r\n"
			  //"Power errors 0x%04x\r\n"
			  "IF output power errors 0x%04x\r\n"
			  "Thermal errors 0x%04x\r\n"
			  "LNA bias error state 0x%04x\r\n"
			  "Individual receiver bias errors:\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n\r\n",
              (!freezeSys ? statusOK : statusERR),
              rtnStatus, rtnPow, rtnTherm, rtnRx,
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
  "    bypassLNAlims   y   magic number y to bypass soft limits on LNA biases.\r\n"
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
      	else if (!strcasecmp(kw, "bypassLNAlims"))  lnaLimitsBypass = (val == 74 ? 1 : 0);
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
     	if (foundLNAbiasSys) {
     		sprintf(status, "%sEngineering report, Front-end system:\r\n"
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
     				lnaPSlimitsBypass, lnaLimitsBypass, d1, d2, argus_lnaPowerPIO(), VER);
    	} else {
    		sprintf(status, "%sEngineering report, DCM2 system:\r\n"
    				"  i2cBusBusy = %d, freeze = %u\r\n"
    				"  successful and unsuccessful I2C bus lock requests since clrCtr = %u and %u\r\n"
    				"  freeze and thaw requests since clrCtr = %u and %u, denials while frozen = %u\r\n"
    				"  version %s\r\n",
    				statusOK, i2cBusBusy, freezeSys,
    				busLockCtr, busNoLockCtr, freezeCtr, thawCtr, freezeErrCtr, VER);
    	}
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
  "  VDGMAX, VGMIN, VGMAX, VDMIN, VDMAX [V], IDMIN, IDMAX [mA], MAXATTEN [dB]\r\n"
		  ;

  if (!arg.help) {
	  sprintf(status, "%s %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %f \r\n",
	      				statusOK, VDGMAX, VGMIN, VGMAX, VDMIN, VDMAX,
	      				IDMIN, IDMAX, MAXATTEN);
  } else {
    longHelp(status, usage, &Correlator::execArgusLimits);
  }
}

/**
  \brief Argus setting limits, JSON version.

  Return values of bias setting limits when queried.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJArgusLimits(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Return bias setting limits in JSON format:\r\n"
  "  Voltages in V, currents in mA, attenuation in dB. \r\n"
		  ;

  if (!arg.help) {
	  sprintf(status, "{\"biasLimits\": {\"cmdOK\": true, \"vdmax\":[%.1f], \"vgminmax\":[%.1f,%.1f], "
			  	  "\"vdminmax\":[%.1f,%.1f], \"idminmax\":[%.1f,%.1f], \"maxatten\":[%.1f]}}\r\n",
			  	  VDGMAX, VGMIN, VGMAX, VDMIN, VDMAX, IDMIN, IDMAX, MAXATTEN);
  } else {
    longHelp(status, usage, &Correlator::execJArgusLimits);
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
  \brief Argus individual drain bias control, JSON response.

  Set or read a single LNA drain bias for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJArgusDrain(return_type status, argument_type arg)
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
        longHelp(status, usage, &Correlator::execJArgusDrain);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX && n > 0 && n <= NSTAGES){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = argus_setLNAbias("d", m-1, n-1, v, 0);
    		if (rtn == -10) {
        		sprintf(status, "{\"biasD\":{\"cmdOK\":false}}\r\n"); //LNA cards are not powered
    		} else {
        		sprintf(status, "{\"biasD\":{\"cmdOK\":%s}}\r\n", (rtn==0 ? "true" : "false"));
    		}
    	} else {
    		sprintf(status, "{\"biasD\":{\"cmdOK\":false}}\r\n"); //Receiver or stage number out of range
    	}
     }
  } else {
      // Command called without arguments; read drain values
      longHelp(status, usage, &Correlator::execJArgusDrain);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execJArgusDrain);
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
  \brief Argus individual gate bias control, JSON version.

  Set or read a single LNA gate bias for Argus system.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJArgusGate(return_type status, argument_type arg)
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
        longHelp(status, usage, &Correlator::execJArgusGate);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX && n > 0 && n <= NSTAGES){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = argus_setLNAbias("g", m-1, n-1, v, 0);
    		if (rtn == -10) {
        		sprintf(status, "{\"biasG\":{\"cmdOK\":false}}\r\n"); // LNA cards are not powered
    		} else {
        		sprintf(status, "{\"biasG\":{\"cmdOK\":%s}}\r\n", (rtn==0 ? "true" : "false"));
    		}
    	} else {
    		sprintf(status, "{\"biasG\":{\"cmdOK\":false}}\r\n"); //Receiver or stage number out of range
    	}
     }
  } else {
      // Command called without arguments; read gate values
      longHelp(status, usage, &Correlator::execJArgusGate);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execJArgusGate);
  }
}


/**
  \brief COMAP individual receiver attenuator control.

  Set a single receiver's warm IF attenuation for COMAP.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execCOMAPatten(return_type status, argument_type arg)
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
        longHelp(status, usage, &Correlator::execCOMAPatten);
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
  } else {  // no argument: return atten vals?
      longHelp(status, usage, &Correlator::execCOMAPatten);
     }
  } else {
    longHelp(status, usage, &Correlator::execCOMAPatten);
  }
}

/**
  \brief COMAP individual receiver attenuator control, JSON version.

  Set a single receiver's warm IF attenuation for COMAP.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJCOMAPatten(return_type status, argument_type arg)
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
        longHelp(status, usage, &Correlator::execJCOMAPatten);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = dcm2_setAtten(m-1, ab, iq, atten);
   			sprintf(status, "{\"dcm2atten\": {\"cmdOK\":%s}}\r\n", (rtn==0 ? "true" : "false"));
    	} else {
   			sprintf(status, "{\"dcm2atten\": {\"cmdOK\":false}}\r\n");
    	}
     }
  } else {
      // Command called without arguments; return atten values?
      longHelp(status, usage, &Correlator::execJCOMAPatten);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execJCOMAPatten);
  }
}

/**
  \brief COMAP individual receiver attenuator control.

  Set a single receiver's warm IF attenuation for COMAP.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execCOMAPpow(return_type status, argument_type arg)
{
  static const char *usage =
  "[M AB IQ dB]\r\n"
  "  Set a receiver warm IF attenuation.\r\n"
  "  M is the Mth receiver to set.\r\n"
  "  AB is either A or B IF bank.\r\n"
  "  IQ is either I or Q.\r\n"
  "  dB is the power level dB to set.\r\n"
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
        longHelp(status, usage, &Correlator::execCOMAPpow);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = dcm2_setPow(m-1, ab, iq, atten);
   			sprintf(status, "%sdcm2_setPow(%d, %s, %s, %f) returned status %d.\r\n",
    					(rtn==0 ? statusOK : statusERR), m, ab, iq, atten, rtn);
    	} else {
    		sprintf(status, "%sReceiver number out of range\r\n", statusERR);
    	}
     }
  } else {  // no argument: return atten vals?
      longHelp(status, usage, &Correlator::execCOMAPpow);
     }
  } else {
    longHelp(status, usage, &Correlator::execCOMAPpow);
  }
}

/**
  \brief COMAP individual receiver attenuator control, JSON version.

  Set a single receiver's warm IF attenuation for COMAP.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJCOMAPpow(return_type status, argument_type arg)
{
  static const char *usage =
  "[M AB IQ dB]\r\n"
  "  Set a receiver warm IF attenuation.\r\n"
  "  M is the Mth receiver to set.\r\n"
  "  AB is either A or B IF bank.\r\n"
  "  IQ is either I or Q.\r\n"
  "  dB is the power level in dB to set.\r\n"
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
        longHelp(status, usage, &Correlator::execJCOMAPpow);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = dcm2_setPow(m-1, ab, iq, atten);
   			sprintf(status, "{\"dcm2pow\": {\"cmdOK\":%s}}\r\n", (rtn==0 ? "true" : "false"));
    	} else {
   			sprintf(status, "{\"dcm2pow\": {\"cmdOK\":false}}\r\n");
    	}
     }
  } else {
      // Command called without arguments; return atten values?
      longHelp(status, usage, &Correlator::execJCOMAPpow);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execJCOMAPpow);
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
  "    P  DCM2 power levels [dBm].\r\n"
  "    S  saddlebag amp power [on/off].\r\n"
  "  Value is the set value in V or dB, or ON or OFF, as appropriate.\r\n"
		  ;

  if (!arg.help) {
    float v = 0.0;
    char inp[10] = {0};
    char act[10] = {0};

    if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%s %s", inp, act);
      if (narg < 2) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusSetAll);
      } else if (!strcmp(inp, "a")) {
    	// Set atten
   		OSTimeDly(CMDDELAY);
   		sscanf(act, "%f", &v);
        int rtn = dcm2_setAllAttens(v);
		sprintf(status, "%sdcm2_setAllAttens(%f) returned status %d.\r\n",
					(rtn==0 ? statusOK : statusERR), v, rtn);
      } else if (!strcmp(inp, "p")) {
    	// Set atten
   		OSTimeDly(CMDDELAY);
   		sscanf(act, "%f", &v);
        int rtn = dcm2_setAllPow(v);
		sprintf(status, "%sdcm2_setAllPow(%f) returned status %d.\r\n",
					(rtn==0 ? statusOK : statusERR), v, rtn);
      } else if (!strcmp(inp, "s")) {
      	// Set saddlebag amplifier state  /// zzz need to change from 1/0 to on/off
     	OSTimeDly(CMDDELAY);
        int rtn = sb_setAllAmps(act);
  		sprintf(status, "%ssb_setAllAmps(%d) returned status %d.\r\n",
  					(rtn==0 ? statusOK : statusERR), (int)v, rtn);
        } else {
        // Set G, D, M biases
    	OSTimeDly(CMDDELAY);
   		sscanf(act, "%f", &v);
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
  \brief Argus: set all gate, drain biases and attenuations to a common value, JSON response.

  Set all gate, drain biases and attenuations to a common value.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJArgusSetAll(return_type status, argument_type arg)
{
  static const char *usage =
  "[KEYWORD VALUE]\r\n"
  "  Set choice of LNA gate/drain bias voltages or \r\n"
  "  receiver warm IF attenuations to a common value.\r\n"
  "  Keywords are:\r\n"
  "    G  gate [V].\r\n"
  "    D  drain [V].\r\n"
  "    A  attenuation [dB].\r\n"
  "    P  DCM2 power levels [dBm].\r\n"
  "    S  saddlebag amp power [on/off].\r\n"
  "  Value is the set value in V or dB, or ON or OFF, as appropriate.\r\n"
		  ;

  if (!arg.help) {
    float v = 0.0;
    char inp[10] = {0};
    char act[10] = {0};

    if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%s %s", inp, act);
      if (narg < 2) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execJArgusSetAll);
      } else if (!strcmp(inp, "a")) {
    	// Set atten
   		OSTimeDly(CMDDELAY);
   		sscanf(act, "%f", &v);
        int rtn = dcm2_setAllAttens(v);
		sprintf(status, "{\"allA\": {\"cmdOK\":%s}}\r\n", (rtn==0 ? "true" : "false"));
      } else if (!strcmp(inp, "p")) {
    	// Set atten
   		OSTimeDly(CMDDELAY);
   		sscanf(act, "%f", &v);
        int rtn = dcm2_setAllPow(v);
		sprintf(status, "{\"allP\": {\"cmdOK\":%s}}\r\n", (rtn==0 ? "true" : "false"));
      } else if (!strcmp(inp, "s")) {
      	// Set saddlebag amplifier state  /// zzz need to change from 1/0 to on/off
     	OSTimeDly(CMDDELAY);
        int rtn = sb_setAllAmps(act);
		sprintf(status, "{\"allS\": {\"cmdOK\":%s}}\r\n", (rtn==0 ? "true" : "false"));
        } else {
        // Set G, D biases
    	OSTimeDly(CMDDELAY);
   		sscanf(act, "%f", &v);
   		int rtn = argus_setAllBias(inp, v, 0);
    	if (rtn == -10) {
    		sprintf(status, "{\"all%c\": {\"cmdOK\":false}}\r\n", toupper(inp[0]));  //LNA cards are not powered
    	} else {
    		sprintf(status, "{\"all%c\": {\"cmdOK\":%s}}\r\n",
    				toupper(inp[0]), (rtn==0 ? "true" : "false"));
    	}
      }

  } else {
      // Command called without arguments
      longHelp(status, usage, &Correlator::execJArgusSetAll);
     }
  } else {
    longHelp(status, usage, &Correlator::execJArgusSetAll);
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
  \brief COMAP jcryo command.

  This method returns the LNA monitor points in JSON format.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJCOMAPcryo(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Return cryostat monitor point values in JSON format.\r\n";

  if (!arg.help && !arg.str) {
	  int rtn = argus_readThermADCs();
	  sprintf(status, "{\"cryostat\":{\"cmdOK\":%s, \"temps\":"
			  "[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f], \"press\":[%.6f]}}\r\n",
			  	(rtn==0 ? "true" : "false"), cryoPar.cryoTemps[0], cryoPar.cryoTemps[1],
	    		cryoPar.cryoTemps[2], cryoPar.cryoTemps[3], cryoPar.cryoTemps[4],
	    		cryoPar.cryoTemps[5],
	    		(cryoPar.auxInputs[0] > 1 ? powf(10., cryoPar.auxInputs[0]-6.) : 0.));
  } else {
    	longHelp(status, usage, &Correlator::execJCOMAPcryo);
  }
}

/**
  \brief Use COMAP LNA and atten presets.

  Use COMAP LNA and atten presets from flash memory.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execCOMAPpresets(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Set LNA bias or DCM2 attenuations to values stored in memory.\r\n"
  "  (see FLASH command to set).\r\n "
		  ;

  if (!arg.help) {
	flash_t flashData;
	zpec_readFlash(&flashData);
	OSTimeDly(CMDDELAY);
	int rtn = comap_presets(&flashData);
    sprintf(status, "%sSetting parameters to stored values, status %d\r\n",
    		(rtn==0 ? statusOK : statusERR), rtn);
  } else {
	longHelp(status, usage, &Correlator::execCOMAPpresets);
  }

}

/**
  \brief Use Argus LNA presets, JSON return.

  Use Argus LNA presets from flash memory.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJCOMAPpresets(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Set LNA bias or DCM2 attenuations to values stored in memory.\r\n"
  "  (see FLASH command to set).\r\n "
		  ;

  if (!arg.help) {
	flash_t flashData;
	zpec_readFlash(&flashData);
	OSTimeDly(CMDDELAY);
	int rtn = comap_presets(&flashData);
    sprintf(status, "\"presets\": {\"cmdOK\":%s}}\r\n",
    		(rtn==0 ? "true" : "false"));
  } else {
	longHelp(status, usage, &Correlator::execJCOMAPpresets);
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
	      			  d1, rxPar[18].LNAmonPts[4], d1, rxPar[18].LNAmonPts[5], d1, rxPar[19].LNAmonPts[4], d1, rxPar[19].LNAmonPts[5]);
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
  \brief COMAP LNA power monitor and control.  JSON returns.

  Turn LNA power on and off, provide monitoring, for COMAP.  JSON returns.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJCOMAPlna(return_type status, argument_type arg)
{
  static const char *usage =
  "[STATE]\r\n"
  "  Sequence LNA power on or off, query LNA power supply.\r\n"
  "  STATE  ON or 1 to sequence LNA power on.\r\n"
  "         OFF or 0 to sequence LNA power off.\r\n"
  "  No argument returns monitor point data.\r\n"
		  ;

  if (!arg.help) {
	  if (arg.str) {  // argument present, set new state.
		  char state[5] = {0};

      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%4s", state);
      if (narg < 1) {
        // Too few arguments; return help string.
        sprintf(status, "{\"lna\": {\"cmdOK\":false}}\r\n");
      } else {
        // Execute the command.
      	if (!strcmp(state, "1") || !strcasecmp(state, "ON")) {
      		OSTimeDly(CMDDELAY);
      		int rtn = argus_lnaPower(1);
            sprintf(status, "{\"lna\": {\"cmdOK\":%s, \"LNAon\": [%.1f]}}\r\n",
            		(rtn==0 ? "true" : "false"), (lnaPwrState && !rtn ? 1.0 : 0.0));
      	}
      	else if (!strcmp(state, "0") || !strcasecmp(state, "OFF")) {
      		OSTimeDly(CMDDELAY);
      		int rtn = argus_lnaPower(0);
            sprintf(status, "{\"lna\": {\"cmdOK\":%s, \"LNAon\": [%.1f]}}\r\n",
            		(rtn==0 ? "true" : "false"), (lnaPwrState && !rtn ? 1.0 : 0.0));
     	}
      	else {
      		longHelp(status, usage, &Correlator::execJCOMAPlna);
      	}
      }

    } else {
      // Command called without arguments; write LNA state

    	int rtn = 0;

    	if (lnaPwrState) {
    		OSTimeDly(CMDDELAY);
    		rtn = argus_readPwrADCs();
    		rtn += argus_readLNAbiasADCs("vg");
    		rtn += argus_readLNAbiasADCs("vd");
    		rtn += argus_readLNAbiasADCs("id");
    	}

		int i, n0, n1, n2, n3, n4, n5;
    	int n = sprintf(outStr, "{\"lna\": {\"cmdOK\":%s, \"LNAon\": [%.1f], "
					"\"powSupp\": [%.1f,%.1f,%.1f], \"Tchassis\": [%.2f], ",
					(rtn==0 && lnaPwrState==1 ? "true" : "false"), (lnaPwrState && !rtn ? 1.0 : 0.0),
					pwrCtrlPar[2], pwrCtrlPar[1], pwrCtrlPar[0], pwrCtrlPar[8]);

	    if (lnaPwrState) {

    		n0 = sprintf(str0, "\"vg1\":[%.3f", rxPar[0].LNAmonPts[0]);
    		n1 = sprintf(str1, "\"vd1\":[%.3f", rxPar[0].LNAmonPts[2]);
    		n2 = sprintf(str2, "\"id1\":[%.3f", rxPar[0].LNAmonPts[4]);
    		n3 = sprintf(str3, "\"vg2\":[%.3f", rxPar[0].LNAmonPts[1]);
    		n4 = sprintf(str4, "\"vd2\":[%.3f", rxPar[0].LNAmonPts[3]);
    		n5 = sprintf(str5, "\"id2\":[%.3f", rxPar[0].LNAmonPts[5]);
    		for (i=1; i<JNRX; i++){
        		n0 += sprintf(&str0[n0], ",%.3f", rxPar[i].LNAmonPts[0]);
        		n1 += sprintf(&str1[n1], ",%.3f", rxPar[i].LNAmonPts[2]);
        		n2 += sprintf(&str2[n2], ",%.3f", rxPar[i].LNAmonPts[4]);
        		n3 += sprintf(&str3[n3], ",%.3f", rxPar[i].LNAmonPts[1]);
        		n4 += sprintf(&str4[n4], ",%.3f", rxPar[i].LNAmonPts[3]);
        		n5 += sprintf(&str5[n5], ",%.3f", rxPar[i].LNAmonPts[5]);
    		}
    		n0 += sprintf(&str0[n0], "]");
    		n1 += sprintf(&str1[n1], "]");
    		n2 += sprintf(&str2[n2], "]");
    		n3 += sprintf(&str3[n3], "]");
    		n4 += sprintf(&str4[n4], "]");
    		n5 += sprintf(&str5[n5], "]");

 		  } else {
 			rtn = argus_readPwrADCs();
 			n0 = sprintf(str0, "\"vg1\":[99.0");
 			n1 = sprintf(str1, "\"vd1\":[99.0");
 			n2 = sprintf(str2, "\"id1\":[99.0");
 			n3 = sprintf(str3, "\"vg2\":[99.0");
 			n4 = sprintf(str4, "\"vd2\":[99.0");
 			n5 = sprintf(str5, "\"id2\":[99.0");
 			for (i=1; i<JNRX; i++){
 				n0 += sprintf(&str0[n0], ",99.0");
 				n1 += sprintf(&str1[n1], ",99.0");
 				n2 += sprintf(&str2[n2], ",99.0");
 				n3 += sprintf(&str3[n3], ",99.0");
 				n4 += sprintf(&str4[n4], ",99.0");
 				n5 += sprintf(&str5[n5], ",99.0");
  	    	}
 			n0 += sprintf(&str0[n0], "]");
 			n1 += sprintf(&str1[n1], "]");
 			n2 += sprintf(&str2[n2], "]");
 			n3 += sprintf(&str3[n3], "]");
 			n4 += sprintf(&str4[n4], "]");
 			n5 += sprintf(&str5[n5], "]");
		  }
    	n += sprintf(&outStr[n], "%s, %s, %s, %s, %s, %s}}\r\n", str0, str1, str2, str3, str4, str5);
 		sprintf(status, outStr);
    }
  } else {
    longHelp(status, usage, &Correlator::execJCOMAPlna);
  }
}

/**
  \brief COMAP LNA power monitor and control: read set points.  JSON returns.

  Turn LNA power on and off, provide monitoring, for COMAP.  JSON returns.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJCOMAPsets(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Query LNA bias set points.\r\n"
		  ;

  int i, n, n0, n1, n2, n3;

  if (!arg.help) {
	  if (lnaPwrState) {

		  n = sprintf(outStr, "{\"lnasets\": {\"cmdOK\":true, \"LNAon\": [1.0], ");

		  n0 = sprintf(str0, "\"vg1\":[%.3f", rxPar[0].LNAsets[0]);
		  n1 = sprintf(str1, "\"vd1\":[%.3f", rxPar[0].LNAsets[2]);
		  n2 = sprintf(str2, "\"vg2\":[%.3f", rxPar[0].LNAsets[1]);
		  n3 = sprintf(str3, "\"vd2\":[%.3f", rxPar[0].LNAsets[3]);
		  for (i=1; i<JNRX; i++){
			  n0 += sprintf(&str0[n0], ",%.3f", rxPar[i].LNAsets[0]);
			  n1 += sprintf(&str1[n1], ",%.3f", rxPar[i].LNAsets[2]);
			  n2 += sprintf(&str2[n2], ",%.3f", rxPar[i].LNAsets[1]);
			  n3 += sprintf(&str3[n3], ",%.3f", rxPar[i].LNAsets[3]);
		  }
		  n0 += sprintf(&str0[n0], "]");
		  n1 += sprintf(&str1[n1], "]");
		  n2 += sprintf(&str2[n2], "]");
		  n3 += sprintf(&str3[n3], "]");

	  } else {
		  n = sprintf(outStr, "{\"lnasets\": {\"cmdOK\":true, \"LNAon\": [0.0], ");

		  n0 = sprintf(str0, "\"vg1\":[99.0");
		  n1 = sprintf(str1, "\"vd1\":[99.0");
		  n2 = sprintf(str2, "\"vg2\":[99.0");
		  n3 = sprintf(str3, "\"vd2\":[99.0");
		  for (i=1; i<JNRX; i++){
			  n0 += sprintf(&str0[n0], ",99.0");
			  n1 += sprintf(&str1[n1], ",99.0");
			  n2 += sprintf(&str2[n2], ",99.0");
			  n3 += sprintf(&str3[n3], ",99.0");
		  }
		  n0 += sprintf(&str0[n0], "]");
		  n1 += sprintf(&str1[n1], "]");
		  n2 += sprintf(&str2[n2], "]");
		  n3 += sprintf(&str3[n3], "]");

	  }
	  n += sprintf(&outStr[n], "%s, %s, %s, %s}}\r\n", str0, str1, str2, str3);
	  sprintf(status, outStr);

  } else {
	  longHelp(status, usage, &Correlator::execJCOMAPsets);
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
	    	      			  d1, rxPar[18].LNAmonPts[4], d1, rxPar[18].LNAmonPts[5], d1, rxPar[19].LNAmonPts[4], d1, rxPar[19].LNAmonPts[5]);
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
    		  if (foundLNAbiasSys) {
    			  sprintf(status, "%sStored bias values in [V]\n\r\n"
	      			  "          1               2               3               4\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  "          5               6               7               8\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  "          9               10              11              12\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  "          13              14              15              16\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  "          17              18              19              20\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n",
	      			  statusOK,
	      			  flashData.lnaGsets[0], flashData.lnaGsets[1], flashData.lnaGsets[2], flashData.lnaGsets[3],
	      			  flashData.lnaGsets[4], flashData.lnaGsets[5], flashData.lnaGsets[6], flashData.lnaGsets[7],
	      			  flashData.lnaDsets[0], flashData.lnaDsets[1], flashData.lnaDsets[2], flashData.lnaDsets[3],
	      			  flashData.lnaDsets[4], flashData.lnaDsets[5], flashData.lnaDsets[6], flashData.lnaDsets[7],

	      			  flashData.lnaGsets[8], flashData.lnaGsets[9], flashData.lnaGsets[10], flashData.lnaGsets[11],
	      			  flashData.lnaGsets[12], flashData.lnaGsets[13], flashData.lnaGsets[14], flashData.lnaGsets[15],
	      			  flashData.lnaDsets[8], flashData.lnaDsets[9], flashData.lnaDsets[10], flashData.lnaDsets[11],
	      			  flashData.lnaDsets[12], flashData.lnaDsets[13], flashData.lnaDsets[14], flashData.lnaDsets[15],
			  
	      			  flashData.lnaGsets[16], flashData.lnaGsets[17], flashData.lnaGsets[18], flashData.lnaGsets[19],
	      			  flashData.lnaGsets[20], flashData.lnaGsets[21], flashData.lnaGsets[22], flashData.lnaGsets[23],
	      			  flashData.lnaDsets[16], flashData.lnaDsets[17], flashData.lnaDsets[18], flashData.lnaDsets[19],
	      			  flashData.lnaDsets[20], flashData.lnaDsets[21], flashData.lnaDsets[22], flashData.lnaDsets[23],
			  
	      			  flashData.lnaGsets[24], flashData.lnaGsets[25], flashData.lnaGsets[26], flashData.lnaGsets[27],
	      			  flashData.lnaGsets[28], flashData.lnaGsets[29], flashData.lnaGsets[30], flashData.lnaGsets[31],
	      			  flashData.lnaDsets[24], flashData.lnaDsets[25], flashData.lnaDsets[26], flashData.lnaDsets[27],
	      			  flashData.lnaDsets[28], flashData.lnaDsets[29], flashData.lnaDsets[30], flashData.lnaDsets[31],

	      			  flashData.lnaGsets[32], flashData.lnaGsets[33], flashData.lnaGsets[34], flashData.lnaGsets[35],
	      			  flashData.lnaGsets[36], flashData.lnaGsets[37], flashData.lnaGsets[38], flashData.lnaGsets[39],
	      			  flashData.lnaDsets[32], flashData.lnaDsets[33], flashData.lnaDsets[34], flashData.lnaDsets[35],
	      			  flashData.lnaDsets[36], flashData.lnaDsets[37], flashData.lnaDsets[38], flashData.lnaDsets[39]);
    		  } else {
        		  sprintf(status, "%sStored A-I/Q and B-I/Q atten values in [dB]\r\n\r\n"
    	      		  "             1               2               3               4\r\n"
    	      		  "A I,Q: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      		  "B I,Q: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
    	      		  "             5               6               7               8\r\n"
    	      		  "A I,Q: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      		  "B I,Q: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
    	      		  "             9               10              11              12\r\n"
    	      		  "A I,Q: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      		  "B I,Q: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
    	      		  "             13              14              15              16\r\n"
    	      		  "A I,Q: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      		  "B I,Q: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
    	      		  "             17              18              19              20\r\n"
    	      		  "A I,Q: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      		  "B I,Q: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n",
    	      		  statusOK,
    	      		  (float)flashData.attenAI[0]/2., (float)flashData.attenAQ[0]/2.,
    	      		  (float)flashData.attenAI[1]/2., (float)flashData.attenAQ[1]/2.,
    	      		  (float)flashData.attenAI[2]/2., (float)flashData.attenAQ[2]/2.,
    	      		  (float)flashData.attenAI[3]/2., (float)flashData.attenAQ[3]/2.,
    	      		  (float)flashData.attenBI[0]/2., (float)flashData.attenBQ[0]/2.,
    	      		  (float)flashData.attenBI[1]/2., (float)flashData.attenBQ[1]/2.,
    	      		  (float)flashData.attenBI[2]/2., (float)flashData.attenBQ[2]/2.,
    	      		  (float)flashData.attenBI[3]/2., (float)flashData.attenBQ[3]/2.,

    	      		  (float)flashData.attenAI[4]/2., (float)flashData.attenAQ[4]/2.,
   	      			  (float)flashData.attenAI[5]/2., (float)flashData.attenAQ[5]/2.,
   	      			  (float)flashData.attenAI[6]/2., (float)flashData.attenAQ[6]/2.,
   	      			  (float)flashData.attenAI[7]/2., (float)flashData.attenAQ[7]/2.,
   	      			  (float)flashData.attenBI[4]/2., (float)flashData.attenBQ[4]/2.,
   	      			  (float)flashData.attenBI[5]/2., (float)flashData.attenBQ[5]/2.,
   	      			  (float)flashData.attenBI[6]/2., (float)flashData.attenBQ[6]/2.,
   	      			  (float)flashData.attenBI[7]/2., (float)flashData.attenBQ[7]/2.,

   	      			  (float)flashData.attenAI[8]/2., (float)flashData.attenAQ[8]/2.,
   	      			  (float)flashData.attenAI[9]/2., (float)flashData.attenAQ[9]/2.,
   	      			  (float)flashData.attenAI[10]/2., (float)flashData.attenAQ[10]/2.,
   	      			  (float)flashData.attenAI[11]/2., (float)flashData.attenAQ[11]/2.,
   	      			  (float)flashData.attenBI[8]/2., (float)flashData.attenBQ[8]/2.,
   	      			  (float)flashData.attenBI[9]/2., (float)flashData.attenBQ[9]/2.,
   	      			  (float)flashData.attenBI[10]/2., (float)flashData.attenBQ[10]/2.,
   	      			  (float)flashData.attenBI[11]/2., (float)flashData.attenBQ[11]/2.,

   	      			  (float)flashData.attenAI[12]/2., (float)flashData.attenAQ[12]/2.,
   	      			  (float)flashData.attenAI[13]/2., (float)flashData.attenAQ[13]/2.,
   	      			  (float)flashData.attenAI[14]/2., (float)flashData.attenAQ[14]/2.,
   	      			  (float)flashData.attenAI[15]/2., (float)flashData.attenAQ[15]/2.,
   	      			  (float)flashData.attenBI[12]/2., (float)flashData.attenBQ[12]/2.,
   	      			  (float)flashData.attenBI[13]/2., (float)flashData.attenBQ[13]/2.,
   	      			  (float)flashData.attenBI[14]/2., (float)flashData.attenBQ[14]/2.,
   	      			  (float)flashData.attenBI[15]/2., (float)flashData.attenBQ[15]/2.,

   	      			  (float)flashData.attenAI[16]/2., (float)flashData.attenAQ[16]/2.,
   	      			  (float)flashData.attenAI[17]/2., (float)flashData.attenAQ[17]/2.,
   	      			  (float)flashData.attenAI[18]/2., (float)flashData.attenAQ[18]/2.,
   	      			  (float)flashData.attenAI[19]/2., (float)flashData.attenAQ[19]/2.,
   	      			  (float)flashData.attenBI[16]/2., (float)flashData.attenBQ[16]/2.,
   	      			  (float)flashData.attenBI[17]/2., (float)flashData.attenBQ[17]/2.,
   	      			  (float)flashData.attenBI[18]/2., (float)flashData.attenBQ[18]/2.,
   	      			  (float)flashData.attenBI[19]/2., (float)flashData.attenBQ[19]/2.);
   		  	  }
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
    	      			  d1, rxPar[18].LNAmonPts[4], d1, rxPar[18].LNAmonPts[5], d1, rxPar[19].LNAmonPts[4], d1, rxPar[19].LNAmonPts[5]);
     		  } else {
    			  sprintf(status, "%sNo report: LNA power is not on.\r\n", statusERR);
    		  }
	      }
  } else { // help string requested
	  longHelp(status, usage, &Correlator::execArgusMonPts);
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
      "  DCM2 commands; no value returns status.\r\n"
      "    KEYWORD   VALUE    VALUE:\r\n"
	  "    amps      on/off             turns amplifier power on/off\r\n"
	  "    led       on/off             turns led on/off\r\n"
	  "    block     ch_no    A/B       blocks DCM2 channel, band A or B\r\n"
	  "  No argument returns monitor point data.\r\n"
			  ;

  int rtn = 0;

  if (!arg.help) {
	  if (arg.str) {
	  // Command called with one or more arguments.
	  char kw[10] = {0};
	  char val[4] = {0};
	  char onoff[4] = {0};
	  int narg = sscanf(arg.str, "%9s %3s %3s", kw, val, onoff);

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
			  rtn = dcm2_blockMod(val, onoff);
			  sprintf(status, "%sdcm2_blockMod(%s, %s) returned with status %d\r\n",
			  		  (!rtn ? statusOK : statusERR), val, onoff, rtn);
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

      int n = 0;
      int i;
      n = sprintf(&outStr[0],
    		  "%sDCM2 parameters:    (status %d)\r\n"
    		  //"%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f \r\n"
    		  "DCM2 7 & 12 V supply voltages: %.1f V, %.1f V, fanout board temp.: %.1f C\r\n"
    		  "4 GHz PLL: %s, 8 GHz PLL: %s\r\n"
    		  "Individual DCM2 modules:\r\n"
    		  "                   Band A                 |             Band B\r\n"
    		  "      Bl AttI AttQ    TPwI    TPwQ   T[C] |Bl AttI AttQ    TPwI    TPwQ   T[C]\r\n",
    		  (!rtn ? statusOK : statusERR), rtn,
    		  //dcm2MBpar[0], dcm2MBpar[1], dcm2MBpar[2], dcm2MBpar[3], dcm2MBpar[4], dcm2MBpar[5], dcm2MBpar[6], dcm2MBpar[7],
    		  dcm2MBpar[5], dcm2MBpar[4], dcm2MBpar[7],
    		  (dcm2MBpar[2] > PLLLOCKTHRESH && dcm2MBpar[2] < 5 ? "locked" : "***UNLOCKED***"),
    		  (dcm2MBpar[3] > PLLLOCKTHRESH && dcm2MBpar[3] < 5 ? "locked" : "***UNLOCKED***"));
      for (i=0; i<NRX; i++) {
    	  n += sprintf(&outStr[n],
		     "Ch %2d: %d %4.1f %4.1f %7.3f %7.3f %6.2f | %d %4.1f %4.1f %7.3f %7.3f %6.2f\r\n",
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
  \brief DCM2 control.

  This method controls DCM2 and readouts with JSON responses.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJDCM2(return_type status, argument_type arg)
{
	  static const char *usage =
	  "[KEYWORD VALUE [VALUE]]\r\n"
      "  JSON format DCM2 commands; no value returns status.\r\n"
      "    KEYWORD   VALUE    VALUE:\r\n"
	  "    amps      on/off             turns amplifier power on/off\r\n"
	  "    led       on/off             turns led on/off\r\n"
	  "    block     ch_no    A/B       blocks DCM2 channel, band A or B\r\n"
      "  No argument returns monitor point data.\r\n"
			  ;

  int rtn = 0;

  if (!arg.help) {
	  if (arg.str) {
	  // Command called with one or more arguments.
	  char kw[10] = {0};
	  char val[4] = {0};
	  char onoff[4] = {0};
	  int narg = sscanf(arg.str, "%9s %3s %3s", kw, val, onoff);

	  if (narg == 2) {
	      // Execute the command.
	      if (!strcasecmp(kw, "amps")) {
  		    rtn = dcm2_ampPow(val);
	    	  sprintf(status, "{\"dcm2\": {\"cmdOK\":%s}}\r\n", (!rtn ? "true" : "false"));
	      } else if (!strcasecmp(kw, "led")) {
	  		    rtn = dcm2_ledOnOff(val);
		    	  sprintf(status, "{\"dcm2\": {\"cmdOK\":%s}}\r\n", (!rtn ? "true" : "false"));
	      } else {
		      longHelp(status, usage, &Correlator::execJDCM2);
	      }
	  } else if (narg == 3){
		  if (!strcasecmp(kw, "block")) {
			  rtn = dcm2_blockMod(val, onoff);
	    	  sprintf(status, "{\"dcm2\": {\"cmdOK\":%s}}\r\n", (!rtn ? "true" : "false"));
		  } else {
			  longHelp(status, usage, &Correlator::execJDCM2);
		  }
	  } else {
		  longHelp(status, usage, &Correlator::execJDCM2);
	  }
	} else {
      rtn = dcm2_readMBadc();
      rtn += dcm2_readMBtemp();
      rtn += dcm2_readAllModTemps();
      rtn += dcm2_readAllModTotPwr();

      // write output: build up JSON string

      int n, n0, n1, n2, n3, n4, n5;
      int i;

      n = sprintf(&outStr[0], "{\"dcm2\": {\"cmdOK\":%s, \"psVolts\":[%.1f,%.1f], "
    		  "\"temp\":[%.1f], \"pllLock\":[%.1f,%.1f], ",
    		  (!rtn ? "true" : "false"),
    		  dcm2MBpar[5], dcm2MBpar[4], dcm2MBpar[7],
    		  (dcm2MBpar[2] > PLLLOCKTHRESH && dcm2MBpar[2] < 5 ? 1.0 : 0.0),
    		  (dcm2MBpar[3] > PLLLOCKTHRESH && dcm2MBpar[3] < 5 ? 1.0 : 0.0));

      n0 = sprintf(&str0[0], "\"Astatus\":[%.1f", (float)dcm2Apar.status[0]);
      n1 = sprintf(&str1[0], "\"AattenI\":[%.1f", (float)dcm2Apar.attenI[0]/2.);
      n2 = sprintf(&str2[0], "\"AattenQ\":[%.1f", (float)dcm2Apar.attenQ[0]/2.);
      n3 = sprintf(&str3[0], "\"ApowI\":[%.3f", dcm2Apar.powDetI[0]);
      n4 = sprintf(&str4[0], "\"ApowQ\":[%.3f", dcm2Apar.powDetQ[0]);
      n5 = sprintf(&str5[0], "\"Atemp\":[%.2f", dcm2Apar.bTemp[0]);
      for (i=1; i<JNRX; i++) {
    	  n0 += sprintf(&str0[n0], ",%.1f", (float)dcm2Apar.status[i]);
    	  n1 += sprintf(&str1[n1], ",%.1f", (float)dcm2Apar.attenI[i]/2.);
    	  n2 += sprintf(&str2[n2], ",%.1f", (float)dcm2Apar.attenQ[i]/2.);
    	  n3 += sprintf(&str3[n3], ",%.3f", dcm2Apar.powDetI[i]);
    	  n4 += sprintf(&str4[n4], ",%.3f", dcm2Apar.powDetQ[i]);
    	  n5 += sprintf(&str5[n5], ",%.2f", dcm2Apar.bTemp[i]);
      }
	  n0 += sprintf(&str0[n0], "]");
	  n1 += sprintf(&str1[n1], "]");
	  n2 += sprintf(&str2[n2], "]");
	  n3 += sprintf(&str3[n3], "]");
	  n4 += sprintf(&str4[n4], "]");
	  n5 += sprintf(&str5[n5], "]");

	  n += sprintf(&outStr[n], "%s, %s, %s, %s, %s, %s, ", str0, str1, str2, str3, str4, str5);

      n0 = sprintf(&str0[0], "\"Bstatus\":[%.1f", (float)dcm2Bpar.status[0]);
      n1 = sprintf(&str1[0], "\"BattenI\":[%.1f", (float)dcm2Bpar.attenI[0]/2.);
      n2 = sprintf(&str2[0], "\"BattenQ\":[%.1f", (float)dcm2Bpar.attenQ[0]/2.);
      n3 = sprintf(&str3[0], "\"BpowI\":[%.3f", dcm2Bpar.powDetI[0]);
      n4 = sprintf(&str4[0], "\"BpowQ\":[%.3f", dcm2Bpar.powDetQ[0]);
      n5 = sprintf(&str5[0], "\"Btemp\":[%.2f", dcm2Bpar.bTemp[0]);
      for (i=1; i<JNRX; i++) {
    	  n0 += sprintf(&str0[n0], ",%.1f", (float)dcm2Bpar.status[i]);
    	  n1 += sprintf(&str1[n1], ",%.1f", (float)dcm2Bpar.attenI[i]/2.);
    	  n2 += sprintf(&str2[n2], ",%.1f", (float)dcm2Bpar.attenQ[i]/2.);
    	  n3 += sprintf(&str3[n3], ",%.3f", dcm2Bpar.powDetI[i]);
    	  n4 += sprintf(&str4[n4], ",%.3f", dcm2Bpar.powDetQ[i]);
    	  n5 += sprintf(&str5[n5], ",%.2f", dcm2Bpar.bTemp[i]);
      }
	  n0 += sprintf(&str0[n0], "]");
	  n1 += sprintf(&str1[n1], "]");
	  n2 += sprintf(&str2[n2], "]");
	  n3 += sprintf(&str3[n3], "]");
	  n4 += sprintf(&str4[n4], "]");
	  n5 += sprintf(&str5[n5], "]");

	  n += sprintf(&outStr[n], "%s, %s, %s, %s, %s, %s}}", str0, str1, str2, str3, str4, str5);
	  sprintf(status, "%s", outStr);
	}
  } else {
	  longHelp(status, usage, &Correlator::execJDCM2);
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
	  "    amp      m      on/off    turns amplifier power for saddlebag m on/off\r\n"
	  "    led      m      on/off    turns led for saddlebag m on/off\r\n"
	  "  No argument returns monitor point data.\r\n"
			  ;

  int rtn = 0;

  if (!arg.help) {
	  if (arg.str) {
	  // Command called with one or more arguments.
	  char kw[10] = {0};
	  int nSbg;
	  char onoff[4] = {0};
	  int narg = sscanf(arg.str, "%9s %d %3s", kw, &nSbg, onoff);

	  if (narg == 3) {
	      // Check for valid saddlebag index number; this is the only place this happens ZZZ doesn't catch error
		  if (nSbg > NSBG || nSbg < 1) nSbg = NSBG+1;  // go to a null device
		  nSbg -= 1; // convert from human to index

	      if (!strcasecmp(kw, "amp")) {
	    	  rtn = sb_ampPow(onoff, nSbg);
	    	  sprintf(status, "%ssb_ampPow(%s) for amp %d returned with status %d\r\n",
	    			  (!rtn ? statusOK : statusERR), onoff, nSbg+1, rtn);
	      } else if (!strcasecmp(kw, "led")) {
	    	  rtn = sb_ledOnOff(onoff, nSbg);
	    	  sprintf(status, "%ssb_ledOnOff(%s) for LED %d returned with status %d\r\n",
	    			  (!rtn ? statusOK : statusERR), onoff, nSbg+1, rtn);
	      } else {
	    	  longHelp(status, usage, &Correlator::execSaddlebag);
	      }
	  } else {
		  longHelp(status, usage, &Correlator::execSaddlebag);
	  }
	} else {
	  rtn = 0;
	  int i;

      for (i=0; i<NSBG; i++) {
    	  rtn += sb_readADC(i);
    	  sbPar[i].pll = sb_readPLLmon(i);
      }
      sprintf(status, "%sSaddlebags:   (status %d)\r\n"
    		  "               1      2      3      4\r\n"
    		  "%s: %6.1f %6.1f %6.1f %6.1f\r\n"
    		  "%s: %6.1f %6.1f %6.1f %6.1f\r\n"
    		  "%s: %6.1f %6.1f %6.1f %6.1f\r\n"
    		  "%s: %6.1f %6.1f %6.1f %6.1f\r\n"
    		  "%s: %6.1f %6.1f %6.1f %6.1f\r\n"
    		  "%s: %6.1f %6.1f %6.1f %6.1f\r\n"
    		  "%s: %6.1f %6.1f %6.1f %6.1f\r\n"
    		  "%s: %6.1f %6.1f %6.1f %6.1f\r\n"
    		  "%s: %6s %6s %6s %6s\r\n"
    		  "%s: %6s %6s %6s %6s\r\n\r\n",
    	  	  (!rtn ? statusOK : statusERR), rtn,
    	  	  sbnames[0], sbPar[0].adcv[0], sbPar[1].adcv[0], sbPar[2].adcv[0], sbPar[3].adcv[0],
    	  	  sbnames[1], sbPar[0].adcv[1], sbPar[1].adcv[1], sbPar[2].adcv[1], sbPar[3].adcv[1],
    	  	  sbnames[2], sbPar[0].adcv[2], sbPar[1].adcv[2], sbPar[2].adcv[2], sbPar[3].adcv[2],
    	  	  sbnames[3], sbPar[0].adcv[3], sbPar[1].adcv[3], sbPar[2].adcv[3], sbPar[3].adcv[3],
    	  	  sbnames[4], sbPar[0].adcv[4], sbPar[1].adcv[4], sbPar[2].adcv[4], sbPar[3].adcv[4],
    	  	  sbnames[5], sbPar[0].adcv[5], sbPar[1].adcv[5], sbPar[2].adcv[5], sbPar[3].adcv[5],
    	  	  sbnames[6], sbPar[0].adcv[6], sbPar[1].adcv[6], sbPar[2].adcv[6], sbPar[3].adcv[6],
    	  	  sbnames[7], sbPar[0].adcv[7], sbPar[1].adcv[7], sbPar[2].adcv[7], sbPar[3].adcv[7],
    	  	  sbnames[8], (sbPar[0].pll ? "lock" : "UNLOCK"), (sbPar[1].pll ? "lock" : "UNLOCK"),
    	  	  (sbPar[2].pll ? "lock" : "UNLOCK"), (sbPar[3].pll ? "lock" : "UNLOCK"),
    	  	  sbnames[9], sbPar[0].ampStatus, sbPar[1].ampStatus, sbPar[2].ampStatus, sbPar[3].ampStatus);
	}
  } else {
	  longHelp(status, usage, &Correlator::execSaddlebag);
  }
}

/**
  \brief JSON saddlebag control.

  This method controls the saddlebag card contols and readouts using JSON formatting.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJSaddlebag(return_type status, argument_type arg)
{
	  static const char *usage =
	  "[KEYWORD VALUE [VALUE]]\r\n"
      "  Saddlebag commands in JSON.  No argument returns status. \r\n"
      "    KEYWORD  VALUE  VALUE:\r\n"
	  "    amp      m      on/off    turns amplifier power for saddlebag m on/off\r\n"
	  "    led      m      on/off    turns led for saddlebag m on/off\r\n"
	  "  No argument returns monitor point data.\r\n"
			  ;

	  int rtn = 0;

	  if (!arg.help) {
		  if (arg.str) {
		  // Command called with one or more arguments.
		  char kw[10] = {0};
		  int nSbg;
		  char onoff[4] = {0};
		  int narg = sscanf(arg.str, "%9s %d %3s", kw, &nSbg, onoff);

		  if (narg == 3) {
		      // Check for valid saddlebag index number; this is the only place this happens ZZZ doesn't catch error
			  if (nSbg > NSBG || nSbg < 1) nSbg = NSBG+1;  // go to a null device
			  nSbg -= 1; // convert from human to index

		      if (!strcasecmp(kw, "amp")) {
		    	  rtn = sb_ampPow(onoff, nSbg);
		    	  sprintf(status, "{\"sbag\": {\"cmdOK\":%s}}\r\n", (!rtn ? "true" : "false"));
		      } else if (!strcasecmp(kw, "led")) {
		    	  rtn = sb_ledOnOff(onoff, nSbg);
		    	  sprintf(status, "{\"sbag\": {\"cmdOK\":%s}}\r\n", (!rtn ? "true" : "false"));
		      } else {
		    	  longHelp(status, usage, &Correlator::execSaddlebag);
		      }
		  } else {
			  longHelp(status, usage, &Correlator::execJSaddlebag);
		  }
		} else {
		  rtn = 0;
	      int n, n0, n1, n2, n3, n4, n5, n6, n7, n8, n9;
	      int i;

	      for (i=0; i<NSBG; i++) {  // read out ADCs and check PLL status
	    	  rtn += sb_readADC(i);
	    	  sbPar[i].pll = sb_readPLLmon(i);
	      }

	      // Assemble JSON return string
	      n = sprintf(&outStr[0], "{\"sbag\": {\"cmdOK\":%s, ",
	    		  (!rtn ? "true" : "false"));
	      n0 = sprintf(&str0[0], "\"ps12v\":[%.1f", sbPar[0].adcv[0]);
	      n1 = sprintf(&str1[0], "\"ps-8v\":[%.1f", sbPar[0].adcv[1]);
	      n2 = sprintf(&str2[0], "\"fanspeed1\":[%.1f", sbPar[0].adcv[2]);
	      n3 = sprintf(&str3[0], "\"fanspeed2\":[%.1f", sbPar[0].adcv[3]);
	      n4 = sprintf(&str4[0], "\"temp1\":[%.1f", sbPar[0].adcv[4]);
	      n5 = sprintf(&str5[0], "\"temp2\":[%.1f", sbPar[0].adcv[5]);
	      n6 = sprintf(&str6[0], "\"temp3\":[%.1f", sbPar[0].adcv[6]);
	      n7 = sprintf(&str7[0], "\"temp4\":[%.1f", sbPar[0].adcv[7]);
	      n8 = sprintf(&str8[0], "\"pllLock\":[%.1f", (sbPar[0].pll==1 ? 1. : 0.));
	      n9 = sprintf(&str9[0], "\"ampOn\":[%.1f", (sbPar[0].ampPwr==1 ? 1. : 0.));
	      for (i=1; i<NSBG; i++) {
	    	  n0 += sprintf(&str0[n0], ",%.1f", sbPar[i].adcv[0]);
	    	  n1 += sprintf(&str1[n1], ",%.1f", sbPar[i].adcv[1]);
	    	  n2 += sprintf(&str2[n2], ",%.1f", sbPar[i].adcv[2]);
	    	  n3 += sprintf(&str3[n3], ",%.1f", sbPar[i].adcv[3]);
	    	  n4 += sprintf(&str4[n4], ",%.1f", sbPar[i].adcv[4]);
	    	  n5 += sprintf(&str5[n5], ",%.1f", sbPar[i].adcv[5]);
	    	  n6 += sprintf(&str6[n6], ",%.1f", sbPar[i].adcv[6]);
	    	  n7 += sprintf(&str7[n7], ",%.1f", sbPar[i].adcv[7]);
	    	  n8 += sprintf(&str8[n8], ",%.1f", (sbPar[i].pll==1 ? 1. : 0.));
	    	  n9 += sprintf(&str9[n9], ",%.1f", (sbPar[i].ampPwr==1 ? 1. : 0.));
	      }
    	  n0 += sprintf(&str0[n0], "]");
    	  n1 += sprintf(&str1[n1], "]");
    	  n2 += sprintf(&str2[n2], "]");
    	  n3 += sprintf(&str3[n3], "]");
    	  n4 += sprintf(&str4[n4], "]");
    	  n5 += sprintf(&str5[n5], "]");
    	  n6 += sprintf(&str6[n6], "]");
    	  n7 += sprintf(&str7[n7], "]");
    	  n8 += sprintf(&str8[n8], "]");
    	  n9 += sprintf(&str9[n9], "]");

    	  n += sprintf(&outStr[n], "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s}}\r\n", str0, str1, str2, str3, str4,
    			  str5, str6, str7, str8, str9);
    	  sprintf(status, "%s", outStr);
		}
	  } else {
		  longHelp(status, usage, &Correlator::execJSaddlebag);
	  }
	}

/*************************************************************************************/
/**
  \brief Vane control.

  This method controls the vane control card and readouts.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execVane(return_type status, argument_type arg)
{
	  static const char *usage =
	  "[Command]\r\n"
      "  Vane commands:\r\n"
      "    OBS moves ambient vane out of the beam.\r\n"
	  "    CAL moves ambient vane into calibration position.\r\n"
      "    MAN switches off both relays for manual control.\r\n"
	  "  No argument returns monitor point data.\r\n"
			  ;

  int rtn = 0;

  if (!arg.help) {
	  if (arg.str) {
	  // Command called with one or more arguments.
	  char kw[10] = {0};
	  int narg = sscanf(arg.str, "%3s", kw);

	  if (narg == 1) {
	      if (!strcasecmp(kw, "obs") || !strcasecmp(kw, "cal") || !strcasecmp(kw, "man")) {
	    	  rtn = vane_obscal(kw);
	    	  vane_readADC();
	          sprintf(status, "%sVane position is %s    (status %d):\r\n"
	        		  "  V_supp =   %5.3f [V]\r\n"
	        		  "  Angle =    %5.1f [deg] (%5.3f [V])\r\n"
	        		  "  T_vane =   %5.3f [C]\r\n"
	        		  "  T_amb =    %5.3f [C]\r\n"
	        		  "  T_shroud = %5.3f [C]\r\n\r\n",
	        	  	  (!rtn ? statusOK : statusERR), vanePar.vanePos, rtn,
	        	  	 vanePar.adcv[0],  vanePar.vaneAngleDeg,  vanePar.adcv[4], vanePar.adcv[5],
	        	  	 vanePar.adcv[6],  vanePar.adcv[7]);
	      } else {
	    	  longHelp(status, usage, &Correlator::execVane);
	      }
	  } else {
		  longHelp(status, usage, &Correlator::execVane);
	  }
	} else {
		vane_readADC();

	  // check vane position
	  /*if (rtn) {                // I2C bus error
		  vanePar.vaneFlag = 4;
		  vanePar.vanePos = "ERROR";
	  } else if (vanePar.vaneFlag > 1 && vanePar.vaneFlag < 8) {
		 // nothing; don't change output if stall or other error reported
	  } else if (fabs(vanePar.vaneAngleDeg) < VANECALERRANGLE) {
		  vanePar.vaneFlag = 1; // record command position as cal, in beam
		  vanePar.vanePos = "CAL";
	  } else if (fabsf(vanePar.vaneAngleDeg - VANESWINGANGLE) < VANEOBSERRANGLE) {
		  vanePar.vaneFlag = 0; // record command position as obs, out of beam
		  vanePar.vanePos = "OBS";
	  } else {                  // report if not in position and no known error
		  vanePar.vaneFlag = 8;
		  vanePar.vanePos = "UNKNOWN";
	  }*/

	  sprintf(status, "%sVane position is %s    (status %d):\r\n"
    		  "  V_supp =   %5.3f [V]\r\n"
    		  "  Angle =    %5.1f [deg] (%5.3f [V])\r\n"
    		  "  T_vane =   %5.3f [C]\r\n"
    		  "  T_amb =    %5.3f [C]\r\n"
    		  "  T_shroud = %5.3f [C]\r\n\r\n",
    	  	  (!rtn ? statusOK : statusERR), vanePar.vanePos, rtn,
    	  	 vanePar.adcv[0],  vanePar.vaneAngleDeg,  vanePar.adcv[4], vanePar.adcv[5],
    	  	 vanePar.adcv[6],  vanePar.adcv[7]);
	}
  } else {
	  longHelp(status, usage, &Correlator::execVane);
  }
}

/*************************************************************************************/
/**
  \brief JSON vane control.

  This method controls the vane control card and readouts, JSON return strings.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execJVane(return_type status, argument_type arg)
{
	  static const char *usage =
	  "[Command]\r\n"
      "  Vane commands with JSON returns:\r\n"
      "    OBS moves ambient vane out of the beam.\r\n"
	  "    CAL moves ambient vane into calibration position.\r\n"
      "    MAN switches off both relays for manual control.\r\n"
	  "  No argument returns monitor point data.\r\n"
			  ;

  int rtn = 0;

  if (!arg.help) {
	  if (arg.str) {
	  // Command called with one or more arguments.
	  char kw[10] = {0};
	  int narg = sscanf(arg.str, "%3s", kw);

	  if (narg == 1) {
	      if (!strcasecmp(kw, "obs") || !strcasecmp(kw, "cal") || !strcasecmp(kw, "man")) {
	    	  sprintf(status, "{\"vane\": {\"cmdOK\": true, \"state\":[%d.0]}}\r\n", 2);
	    	  rtn = vane_obscal(kw);
	      } else {
	    	  longHelp(status, usage, &Correlator::execJVane);
	      }
	  } else {
		  longHelp(status, usage, &Correlator::execJVane);
	  }
	} else {
		  rtn = vane_readADC();
		  // check vane position
		  if (fabs(vanePar.vaneAngleDeg) < VANECALERRANGLE) {
			  vanePar.vaneFlag = 1; // record command position as cal, in beam
			  vanePar.vanePos = "CAL";
		  } else if (fabsf(vanePar.vaneAngleDeg - VANESWINGANGLE) < VANEOBSERRANGLE) {
			  vanePar.vaneFlag = 0; // record command position as obs, out of beam
			  vanePar.vanePos = "OBS";
		  } else if (vanePar.vaneFlag > 1 && vanePar.vaneFlag < 8) {
			 // don't change output if stall or other error reported
		  } else if (!rtn) {   // I2C bus error
			  vanePar.vaneFlag = 99;
			  vanePar.vanePos = "ERROR";
		  } else {             // report if not in position and no known error
			  vanePar.vaneFlag = 8;
			  vanePar.vanePos = "UNKNOWN";
		  }
      sprintf(status, "{\"vane\": {\"cmdOK\":%s, \"powSupp\":[%.3f], \"angle\":[%.1f], \"Tvane\":[%.3f], "
    		  "\"Tamb\":[%.3f], \"Tshroud\":[%.3f], \"position\": [%d.0], \"state\":[%d.0]}}\r\n",
    		  (!rtn ? "true" : "false"),
    		  vanePar.adcv[0], vanePar.vaneAngleDeg, vanePar.adcv[5], vanePar.adcv[6], vanePar.adcv[7],
    		  vanePar.vaneFlag, 0);
	}
  } else {
	  longHelp(status, usage, &Correlator::execJVane);
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
