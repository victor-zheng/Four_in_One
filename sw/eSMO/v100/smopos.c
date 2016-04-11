/*=====================================================================================
 File name:        SMOPOS.C  (IQ version)                  
                    
 Originator:	Digital Control Systems Group
			Texas Instruments

 Description:  Rotor Position Estimator of PMSM using Sliding-Mode Theory                 

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
-------------------------------------------------------------------------------------*/

#include "IQmathLib.h"         // Include header for IQmath library 
// Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file 
#include "dmctype.h"
#include "smopos.h"

void smopos_calc(SMOPOS *v)
{	
    _iq E0;
    
    E0 = _IQ(0.5);
    
// Sliding mode current observer
    v->EstIalpha = _IQmpy(v->Fsmopos,v->EstIalpha) + _IQmpy(v->Gsmopos,(v->Valpha-v->Ealpha-v->Zalpha));
    v->EstIbeta = _IQmpy(v->Fsmopos,v->EstIbeta) + _IQmpy(v->Gsmopos,(v->Vbeta-v->Ebeta-v->Zbeta));

// Current errors
    v->IalphaError = v->EstIalpha - v->Ialpha;
    v->IbetaError = v->EstIbeta - v->Ibeta;

// Sliding control calculator
    if (_IQabs(v->IalphaError) < E0)
       v->Zalpha = _IQmpy(v->Kslide,_IQdiv(v->IalphaError,E0));  
    else if (v->IalphaError >= E0) 
       v->Zalpha = v->Kslide;
    else if (v->IalphaError <= -E0) 
       v->Zalpha = -v->Kslide;

    if (_IQabs(v->IbetaError) < E0)
       v->Zbeta = _IQmpy(v->Kslide,_IQdiv(v->IbetaError,E0));  
    else if (v->IbetaError >= E0) 
       v->Zbeta = v->Kslide;
    else if (v->IbetaError <= -E0) 
       v->Zbeta = -v->Kslide;

// Sliding control filter -> back EMF calculator
    v->Ealpha = v->Ealpha + _IQmpy(v->Kslf,(v->Zalpha-v->Ealpha));
    v->Ebeta = v->Ebeta + _IQmpy(v->Kslf,(v->Zbeta-v->Ebeta));

// Rotor angle calculator -> Theta = atan(-Ealpha,Ebeta)
    v->Theta = _IQatan2PU(-v->Ealpha,v->Ebeta); 
}



