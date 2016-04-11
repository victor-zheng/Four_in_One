/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   solutions/instaspin_foc/src/proj_lab05b.c
//! \brief Adjusting the speed current controller
//!
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup PROJ_LAB05b PROJ_LAB05b
//@{

//! \defgroup PROJ_LAB05b_OVERVIEW Project Overview
//!
//! Adjusting the supplied speed controller
//!

// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main.h"

#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif

// Include header files used in the main function


// **************************************************************************
// the defines


#define LED_BLINK_FREQ_Hz   5


// **************************************************************************
// the globals

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

MATH_vec3 gAdcBiasI;
MATH_vec3 gAdcBiasV;
CTRL_Handle ctrlHandle;
CTRL_Obj    ctrlObj_esmo;


CLARKE_Handle   clarkeHandle_I;               //!< the handle for the current Clarke transform
CLARKE_Obj      clarke_I;                     //!< the current Clarke transform object

CLARKE_Handle   clarkeHandle_V;               //!< the handle for the voltage Clarke transform
CLARKE_Obj      clarke_V;                     //!< the voltage Clarke transform object

EST_Handle      estHandle;                    //!< the handle for the estimator

IPARK_Handle    iparkHandle;                  //!< the handle for the inverse Park transform
IPARK_Obj       ipark;                        //!< the inverse Park transform object

PARK_Handle     parkHandle;                   //!< the handle for the Park object
PARK_Obj        park;                         //!< the Park transform object

SVGEN_Handle    svgenHandle;                  //!< the handle for the space vector generator
SVGEN_Obj       svgen;                        //!< the space vector generator object

HAL_Handle halHandle;

USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else
CTRL_Obj ctrl;				//v1p7 format
#endif

uint16_t gLEDcnt = 0;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif


#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;
//********************************************************************************
TRAJ_ANGLE_Obj      trajAngleObj;
TRAJ_ANGLE_Handle   trajAngleHandle;

void angleGeneratorInit(TRAJ_ANGLE_Handle handle);

void angleGenerator(TRAJ_ANGLE_Handle handle);
//****************************************************************************
//eSMO
volatile ESMOPOS esmo1=ESMOPOS_DEFAULTS;
volatile ESMOPOS esmo2=ESMOPOS_DEFAULTS;

SMOPOS_CONST smo1_const = SMOPOS_CONST_DEFAULTS;
volatile PI_CONTROLLER  pi_smo = PI_CONTROLLER_DEFAULTS;
SPEED_ESTIMATION speed3 = SPEED_ESTIMATION_DEFAULTS;
//***************************************************************************

volatile long test_angle;
volatile long test_Va;
volatile long test_Vb;
volatile long test_Ia;
volatile long test_Ib;
// **************************************************************************
// the functions

volatile _iq value;
void main(void)
{

  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
  #endif

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));


  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);


  // store user parameter error in global variable
  gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);


  // do not allow code execution if there is a user parameter error
  if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVars.Flag_enableSys = false;
        }
    }

  // initialize the gMotorVars
  gMotorVars.Flag_enableSys = false;
  gMotorVars.Flag_Run_Identify = false;

  // initialize the user parameters
  USER_setParams(&gUserParams);


  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);


  // set the default controller parameters
  ctrlHandle = (CTRL_Handle)&ctrlObj_esmo;
  ctrlHandle->clarkeHandle_I = (CLARKE_Handle)&(ctrlObj_esmo.clarke_I);
  ctrlHandle->clarkeHandle_V = (CLARKE_Handle)&(ctrlObj_esmo.clarke_V);
  ctrlHandle->parkHandle = (PARK_Handle)&(ctrlObj_esmo.park);
  ctrlHandle->iparkHandle = (IPARK_Handle)&(ctrlObj_esmo.ipark);

  ctrlHandle->pidHandle_spd = (PID_Handle)&(ctrlObj_esmo.pid_spd);
  ctrlHandle->pidHandle_Id = (PID_Handle)&(ctrlObj_esmo.pid_Id);
  ctrlHandle->pidHandle_Iq = (PID_Handle)&(ctrlObj_esmo.pid_Iq);

  ctrlHandle->switchHandle = (SWITCH_Handle)&(ctrlObj_esmo.switch_loop);

  ctrlHandle->svgenHandle = (SVGEN_Handle)&(ctrlObj_esmo.svgen);
  CTRL_setParams(ctrlHandle,&gUserParams);


  // initialize the Clarke modules
  clarkeHandle_I = CLARKE_init(&clarke_I,sizeof(clarke_I));
  clarkeHandle_V = CLARKE_init(&clarke_V,sizeof(clarke_V));


  // set the number of current sensors
  setupClarke_I(clarkeHandle_I,gUserParams.numCurrentSensors);


  // set the number of voltage sensors
  setupClarke_V(clarkeHandle_V,gUserParams.numVoltageSensors);


#ifdef FAST_ROM_V1p6
  estHandle = controller_obj->estHandle;
#else
  estHandle = ctrl.estHandle;
#endif


  // initialize the inverse Park module
  iparkHandle = IPARK_init(&ipark,sizeof(ipark));


  // initialize the Park module
  parkHandle = PARK_init(&park,sizeof(park));


  // initialize the space vector generator module
  svgenHandle = SVGEN_init(&svgen,sizeof(svgen));


  // setup faults
  HAL_setupFaults(halHandle);


  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle);


  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);


  // enable global interrupts
  HAL_enableGlobalInts(halHandle);


  // enable debug interrupts
  HAL_enableDebugInt(halHandle);


  // disable the PWM
  HAL_disablePwm(halHandle);


#ifdef DRV8301_SPI
  // turn on the DRV8301 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8301 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301Vars);

  HAL_writeDrvData(halHandle,&gDrvSpi8301Vars);

  HAL_readDrvData(halHandle,&gDrvSpi8301Vars);
#endif
  // enable DC bus compensation
  CTRL_setFlag_enableDcBusComp(ctrlHandle, true);


  // compute scaling factors for flux and torque calculations
  gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
  gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
  gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
  gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

//****************************************************************************
// Initialize the SMOPOS constant module
  	smo1_const.Rs = USER_MOTOR_Rs;
  	smo1_const.Ls = USER_MOTOR_Ls_d;
  	smo1_const.Ib = USER_IQ_FULL_SCALE_CURRENT_A;
  	smo1_const.Vb = (float)12.0;
  	smo1_const.Ts = USER_CTRL_PERIOD_sec;
  	SMO_CONST_MACRO(smo1_const)


   	// Initialize eSMO parameters
   	esmo1.Fsmopos  = _IQ(smo1_const.Fsmopos);
   	esmo1.Gsmopos  = _IQ(smo1_const.Gsmopos);
   	esmo1.Kslide   = _IQ(0.05308703613);
  	esmo1.base_wTs = _IQ(USER_IQ_FULL_SCALE_FREQ_Hz*USER_CTRL_PERIOD_sec);
  	esmo1.factor   = _IQ(2.0);

   	// Initialize eSMO parameters
   	esmo2.Fsmopos  = _IQ(1.0-USER_MOTOR_Rs*USER_CTRL_PERIOD_sec/USER_MOTOR_Ls_d);
   	esmo2.Gsmopos  = _IQ(USER_CTRL_PERIOD_sec/USER_MOTOR_Ls_d);
   	esmo2.Kslide   = _IQ(0.1*USER_MOTOR_RATED_FLUX*1000.0/USER_IQ_FULL_SCALE_VOLTAGE_V);
   	esmo2.Kslf = _IQ(USER_CTRL_PERIOD_sec*2*3.14*Fc);
   	esmo2.base_wTs = _IQ(USER_IQ_FULL_SCALE_FREQ_Hz*USER_CTRL_PERIOD_sec);


  	// Initialize the PI module for smo angle filter
    pi_smo.Kp   = _IQ(2.0);
   	pi_smo.Ki   = _IQdiv(_IQ(100*0.001/USER_CTRL_FREQ_Hz), pi_smo.Kp);
   	pi_smo.Umax = _IQ(1.0);
   	pi_smo.Umin = _IQ(-1.0);

   	// Initialize the SPEED_EST module SMOPOS based speed calculation
	speed3.K1 = _IQ21(1/(USER_IQ_FULL_SCALE_FREQ_Hz*USER_CTRL_PERIOD_sec));
	speed3.K2 = _IQ(1/(1+USER_CTRL_PERIOD_sec*2*3.14159*5));  // Low-pass cut-off frequency
	speed3.K3 = _IQ(1)-speed3.K2;
	speed3.BaseRpm = 120*(USER_IQ_FULL_SCALE_FREQ_Hz/USER_MOTOR_NUM_POLE_PAIRS);
//****************************************************************************

	trajAngleHandle = (TRAJ_ANGLE_Handle)&trajAngleObj;
	angleGeneratorInit(trajAngleHandle);
	CTRL_setState(ctrlHandle,CTRL_State_Offline);
  for(;;)
  {
    // Waiting for enable system flag to be set
    while(!(gMotorVars.Flag_enableSys));

    // loop while the enable system flag is true
    while(gMotorVars.Flag_enableSys)
    {
      // increment counters
      gCounter_updateGlobals++;

      updateGlobalVariables_motor(ctrlHandle);

//The state machine

      switch(ctrlObj_esmo.state)
      {
      case CTRL_State_Offline:
    	  if(ctrlHandle->flag_enableOffset == 0)
    	  {
    		  ctrlHandle->flag_enableOffset = 1;
    		  HAL_enablePwm(halHandle);
    	  }
    	  else
    	  {
			  if(ctrlHandle->counter_calibration > gUserParams.ctrlWaitTime[CTRL_State_Offline])   //2 seconds
			  {
				  HAL_disablePwm(halHandle);
				  ctrlHandle->flag_enableOffset = 0;
				  ctrlHandle->counter_calibration = 0;

				  halHandle->adcBias.I.value[0] = halHandle->adcBias.I.value[0] + halHandle->offset_I[0].value;
				  halHandle->adcBias.I.value[1] = halHandle->adcBias.I.value[1] + halHandle->offset_I[1].value;
				  halHandle->adcBias.I.value[2] = halHandle->adcBias.I.value[2] + halHandle->offset_I[2].value;

				  halHandle->adcBias.V.value[0] = halHandle->adcBias.V.value[0] + halHandle->offset_V[0].value;
				  halHandle->adcBias.V.value[1] = halHandle->adcBias.V.value[1] + halHandle->offset_V[1].value;
				  halHandle->adcBias.V.value[2] = halHandle->adcBias.V.value[2] + halHandle->offset_V[2].value;
	    		  CTRL_setState(ctrlHandle,CTRL_State_Idle);
			  }
    	  }
    	  break;
      case CTRL_State_Idle:
	  	  GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_6);
    	  if(gMotorVars.Flag_Run_Identify == 1)
    	  {
    		  HAL_enablePwm(halHandle);
          	  ctrlHandle->Idq_ref.value[0] = _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A);//Id=1.0A
          	  ctrlHandle->Idq_ref.value[1] = _IQ(0.0);
          	  ctrlHandle->counter_forceangle = 0;
          	  ctrlHandle->flag_enableSpeedCtrl = 0;
	    	  CTRL_setState(ctrlHandle,CTRL_State_ForceAngle);
    	  }
    	  else
    	  {
    		  HAL_disablePwm(halHandle);
    		  CTRL_setParams(ctrlHandle,&gUserParams);
    	  }
    	  break;
      case CTRL_State_ForceAngle:  //open loop
    	  if(gMotorVars.Flag_Run_Identify == 0) CTRL_setState(ctrlHandle,CTRL_State_Idle);
    	  if(ctrlHandle->counter_forceangle > gUserParams.ctrlWaitTime[CTRL_State_ForceAngle])   //1 seconds
    	  {
          	  ctrlHandle->Idq_ref.value[0] = _IQ(0.0);
          	  ctrlHandle->Idq_ref.value[1] = _IQ(0.5/USER_IQ_FULL_SCALE_CURRENT_A);//Iq=1.0A

    		  angleGeneratorInit(trajAngleHandle);
    		  switchInit(ctrlHandle);

    		  ctrlHandle->counter_forceangle = 0;
    		  CTRL_setState(ctrlHandle,CTRL_State_OpenLoop);
    	  }
      case CTRL_State_OpenLoop:  //open loop
    	  if(gMotorVars.Flag_Run_Identify == 0) CTRL_setState(ctrlHandle,CTRL_State_Idle);

    	  ctrlHandle->switchHandle->speedError = _IQabs(ctrlHandle->switchHandle->speedPoint - gMotorVars.Speed_krpm);

    	  if(trajAngleHandle->currentAngulaRate > _IQmpy(trajAngleHandle->maxAngulaRate,_IQ(0.9)))
    	  {
    		  if(ctrlHandle->switchHandle->speedError < _IQmpy(ctrlHandle->switchHandle->speedThreshold,ctrlHandle->switchHandle->speedPoint))
    		  {

            	  ctrlHandle->Idq_ref.value[0] = _IQ(0.0);
              	  ctrlHandle->Idq_ref.value[1] = _IQ(0.1/USER_IQ_FULL_SCALE_CURRENT_A);//Iq=1.0A
             	  ctrlHandle->flag_enableSpeedCtrl = 0;
        		  CTRL_setState(ctrlHandle,CTRL_State_OnLine);
    		  }
    	  }
    	  break;
      case CTRL_State_OnLine:
    	  if(gMotorVars.Flag_Run_Identify == 0) CTRL_setState(ctrlHandle,CTRL_State_Idle);
	  	  GPIO_setLow(halHandle->gpioHandle,GPIO_Number_6);

    	  break;
      case CTRL_State_Error:
          CTRL_setFlag_enableCtrl(ctrlHandle,false);
          HAL_disablePwm(halHandle);
          gMotorVars.Flag_enableSys = false;
    	  break;
      default:
          CTRL_setFlag_enableCtrl(ctrlHandle,false);
          HAL_disablePwm(halHandle);
          gMotorVars.Flag_enableSys = false;
      }
      updateGlobalVariables_motor(ctrlHandle);
      updateKpKiGains(ctrlHandle);
    } // end of while(gFlag_enableSys) loop


    // disable the PWM
    HAL_disablePwm(halHandle);
    // set the default controller parameters (Reset the control to re-identify the motor)
    CTRL_setParams(ctrlHandle,&gUserParams);
    gMotorVars.Flag_Run_Identify = false;

  } // end of for(;;) loop

} // end of main() function

int test_switch_flag = 0;
_iq angle_error_pu = 0;
interrupt void mainISR(void)
{


  _iq angle_pu = 0;

  _iq speed_ref_pu = TRAJ_getIntValue(((CTRL_Obj *)ctrlHandle)->trajHandle_spd);
  _iq speed_outMax_pu = TRAJ_getIntValue(((CTRL_Obj *)ctrlHandle)->trajHandle_spdMax);

  MATH_vec2 Iab_pu;
  MATH_vec2 Vab_pu;
  MATH_vec2 Vdq_out_pu;
  MATH_vec2 Vab_out_pu;
  MATH_vec2 phasor;


  // toggle status LED
  if(gLEDcnt++ > (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
  {
    HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
    gLEDcnt = 0;
  }


  // acknowledge the ADC interrupt
  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);


  // convert the ADC data
  HAL_readAdcData(halHandle,&gAdcData);

    // if needed, run the controller
    if(ctrlHandle->counter_isr >= ctrlHandle->numIsrTicksPerCtrlTick)
    {
		// reset the isr count
		ctrlHandle->counter_isr = 1;

		// run Clarke transform on current
		CLARKE_run(clarkeHandle_I,&gAdcData.I,&Iab_pu);

		// run Clarke transform on voltage
		CLARKE_run(clarkeHandle_V,&gAdcData.V,&Vab_pu);

  	    /********************************************************************
  	    * eSMO module start
  	    * ******************************************************************/
		esmo2.Ialpha   = Iab_pu.value[0];
		esmo2.Ibeta    = Iab_pu.value[1];
		esmo2.Valpha   = Vab_pu.value[0];
		esmo2.Vbeta    = Vab_pu.value[1];
		esmo2.runSpeed = speed3.EstimatedSpeed;
		esmo2.cmdSpeed = speed_ref_pu;

		eSMO_MODULE_victor((ESMOPOS*)&esmo2);

		esmo2.Theta2 = esmo2.Theta;
	    esmo2.Theta2 = angleFilter((PI_CONTROLLER*)&pi_smo, (ESMOPOS*)&esmo2);  // optional - uncomment to include
		speed3.EstimatedTheta = esmo2.Theta2;
		SE_MACRO(speed3)
		ctrlHandle->speed_fb_pu = speed3.EstimatedSpeed;
		/********************************************************************
		 * eSMO module End
		 * ******************************************************************/
        if(ctrlHandle->state <= CTRL_State_ForceAngle)
        {
        	angle_pu = _IQ(0.0);
        	ctrlHandle->counter_forceangle++;
        }
        else if(ctrlHandle->state == CTRL_State_OpenLoop)
        {
        	angleGenerator(trajAngleHandle);
        	angle_pu = trajAngleHandle->outputAngle;
        }
        else if(ctrlHandle->state == CTRL_State_OnLine)
        {
        	angle_pu = esmo2.Theta;
        }
		PWMDAC_write_CmpA(halHandle->pwmDacHandle[PWMDAC_Number_2],esmo2.Theta >> 15);
		PWMDAC_write_CmpB(halHandle->pwmDacHandle[PWMDAC_Number_2],trajAngleHandle->outputAngle >> 15);
        //**********************************************************************
		// compute the sin/cos phasor
		CTRL_computePhasor(angle_pu,&phasor);

		// set the phasor in the Park transform
		PARK_setPhasor(parkHandle,&phasor);

		// run the Park transform
		PARK_run(parkHandle,&Iab_pu,CTRL_getIdq_in_addr(ctrlHandle));
//**********************************************************************************************************
		// run the online controller
        if(ctrlHandle->state >CTRL_State_Idle)
        {
	    // increment the current count
		ctrlHandle->counter_current++;

	    // increment the speed count
		ctrlHandle->counter_speed++;

		// when appropriate, run the PID speed controller
		if(ctrlHandle->flag_enableSpeedCtrl == 1)
		{
			if(ctrlHandle->counter_speed >= ctrlHandle->numCtrlTicksPerSpeedTick)
			{
				  // reset the speed count
				  ctrlHandle->counter_speed = 0;
				  ctrlHandle->pidHandle_spd->outMax = ctrlHandle->speed_outMax_pu;
			      ctrlHandle->pidHandle_spd->outMin = 0;
			      PID_run_spd(ctrlHandle->pidHandle_spd,
			    		      ctrlHandle->speed_ref_pu,
							  ctrlHandle->speed_fb_pu,
							  &ctrlHandle->Idq_ref.value[1]);
			}
		}
		// when appropriate, run the PID Id and Iq controllers
		if(ctrlHandle->counter_current >= ctrlHandle->numCtrlTicksPerCurrentTick)
		{
			// reset the current count
			ctrlHandle->counter_current = 0;
			_iq Kp_Id = ctrlHandle->Kp_Id;
			_iq Kp_Iq = ctrlHandle->Kp_Iq;;

		    // compute the Kp gain
		    // Scale Kp instead of output to prevent saturation issues
		    if(CTRL_getFlag_enableDcBusComp(ctrlHandle))
		    {
		    	Kp_Id = _IQmpy(Kp_Id,_IQ(1.0));
		    	Kp_Iq = _IQmpy(Kp_Iq,_IQ(1.0));
		    }
		    ctrlHandle->pidHandle_Id->Kp = Kp_Id;
		    ctrlHandle->pidHandle_Iq->Kp = Kp_Iq;

		    ctrlHandle->pidHandle_Id->outMax = ctrlHandle->maxVsMag_pu;
		    ctrlHandle->pidHandle_Id->outMin = -ctrlHandle->maxVsMag_pu;
		    ctrlHandle->pidHandle_Iq->outMax = ctrlHandle->maxVsMag_pu;
		    ctrlHandle->pidHandle_Iq->outMin = -ctrlHandle->maxVsMag_pu;

		    // run the Id PID controller
		    PID_run(ctrlHandle->pidHandle_Id,
		    		ctrlHandle->Idq_ref.value[0],
		    		ctrlHandle->Idq_in.value[0],
					(_iq *)&ctrlHandle->Vdq_out.value[0]);

		    // run the Iq PID controller
		    PID_run(ctrlHandle->pidHandle_Iq,
		    		ctrlHandle->Idq_ref.value[1],
					ctrlHandle->Idq_in.value[1],
					(_iq *)&ctrlHandle->Vdq_out.value[1]);
		    // add voltage offsets
		    ctrlHandle->Vdq_offset_pu.value[0] = _IQ(0.0);
		    ctrlHandle->Vdq_offset_pu.value[1] = _IQ(0.0);
		    CTRL_addVdq_offset(ctrlHandle);

			// get the controller output
			CTRL_getVdq_out_pu(ctrlHandle,&Vdq_out_pu);

			// set the phasor in the inverse Park transform
			IPARK_setPhasor(iparkHandle,&phasor);

			// run the inverse Park module
			IPARK_run(iparkHandle,&Vdq_out_pu,&Vab_out_pu);

			// run the space Vector Generator (SVGEN) module
			SVGEN_run(svgenHandle,&Vab_out_pu,&(gPwmData.Tabc));
		}
        }
        else if(ctrlHandle->state == CTRL_State_Offline)   //calibration
        {
        	if(ctrlHandle->flag_enableOffset == 1)
  		    {
        		ctrlHandle->counter_calibration++;
        		CTRL_runOffLine(ctrlHandle,halHandle,&gAdcData,&gPwmData);
  		    }
        }
        else   //CTRL_State_Idle
        {
        	gPwmData.Tabc.value[0] = 0;
        	gPwmData.Tabc.value[1] = 0;
        	gPwmData.Tabc.value[2] = 0;
        }
    }
    else
    {
      // increment the isr count
      ctrlHandle->counter_isr++;
    }

  // write the PWM compare values
  HAL_writePwmData(halHandle,&gPwmData);


  return;
} // end of mainISR() function

void angleGeneratorInit(TRAJ_ANGLE_Handle handle)
{
	handle->currentAngulaRate = _IQ(0.0);
	handle->maxAngulaRate = _IQ(USER_MOTOR_OPEN_LOOP_ANGULA_RATE_pu);
	handle->lastTime = _IQ(USER_MOTOR_OPEN_LOOP_TIME_sec);
	handle->acceleration = _IQdiv(handle->maxAngulaRate,handle->lastTime);
	handle->deltaTime = _IQ(USER_CTRL_PERIOD_sec);
	handle->currentAngleRateStep = _IQmpy(handle->acceleration,handle->deltaTime);
	handle->outputAngle = _IQ(0.0);
}

void angleGenerator(TRAJ_ANGLE_Handle handle)
{
	handle->currentAngulaRate = handle->currentAngulaRate + handle->currentAngleRateStep;
	handle->currentAngulaRate = _IQsat(handle->currentAngulaRate,handle->maxAngulaRate,_IQ(0.0));
	handle->outputAngle = handle->outputAngle + _IQmpy(handle->currentAngulaRate,handle->deltaTime);
	if(handle->outputAngle > _IQ(1.0))
	{
		handle->outputAngle = handle->outputAngle - _IQ(1.0);
	}
}

void setupClarke_I(CLARKE_Handle handle,const uint_least8_t numCurrentSensors)
{
  _iq alpha_sf,beta_sf;


  // initialize the Clarke transform module for current
  if(numCurrentSensors == 3)
    {
      alpha_sf = _IQ(MATH_ONE_OVER_THREE);
      beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
  else if(numCurrentSensors == 2)
    {
      alpha_sf = _IQ(1.0);
      beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
  else
    {
      alpha_sf = _IQ(0.0);
      beta_sf = _IQ(0.0);
    }

  // set the parameters
  CLARKE_setScaleFactors(handle,alpha_sf,beta_sf);
  CLARKE_setNumSensors(handle,numCurrentSensors);

  return;
} // end of setupClarke_I() function


void setupClarke_V(CLARKE_Handle handle,const uint_least8_t numVoltageSensors)
{
  _iq alpha_sf,beta_sf;


  // initialize the Clarke transform module for voltage
  if(numVoltageSensors == 3)
    {
      alpha_sf = _IQ(MATH_ONE_OVER_THREE);
      beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
 else
    {
      alpha_sf = _IQ(0.0);
      beta_sf = _IQ(0.0);
    }

  // set the parameters
  CLARKE_setScaleFactors(handle,alpha_sf,beta_sf);
  CLARKE_setNumSensors(handle,numVoltageSensors);

  return;
} // end of setupClarke_V() function


void updateGlobalVariables_motor(CTRL_Handle handle)
{
//******************************************************************************************
  //set value
  CTRL_setFlag_enableCtrl(handle, gMotorVars.Flag_Run_Identify);
  CTRL_setSpd_ref_krpm(handle,gMotorVars.SpeedRef_krpm);
//*******************************************************************************************
  //get value

  // get the controller state
  gMotorVars.CtrlState = CTRL_getState(handle);

  // Get the DC buss voltage
  gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

  gMotorVars.Speed_krpm = _IQmpy(speed3.EstimatedSpeed,_IQ(10.0));

  return;
} // end of updateGlobalVariables_motor() function


void updateKpKiGains(CTRL_Handle handle)
{
  if((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdate == false))
    {
      // set the kp and ki speed values from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVars.Kp_spd);
      CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVars.Ki_spd);

      // set the kp and ki current values for Id and Iq from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVars.Ki_Idq);
      CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVars.Ki_Idq);
	}

  return;
} // end of updateKpKiGains() function

void switchInit(CTRL_Handle handle)
{
	handle->switchHandle->counter = 0;
	handle->switchHandle->speedPoint = _IQ(USER_MOTOR_OPEN_LOOP_SPEED_pu);
	handle->switchHandle->speedPointHalf = _IQmpy(handle->switchHandle->speedPoint, _IQ(0.5));
	handle->switchHandle->speedThreshold = _IQ(0.1);
}

//@} //defgroup
// end of file



