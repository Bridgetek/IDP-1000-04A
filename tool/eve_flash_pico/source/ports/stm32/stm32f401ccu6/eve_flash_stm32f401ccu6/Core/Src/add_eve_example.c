#include "common.h"
#include "add_eve_example.h"

extern EVE_HalContext g_Host;
void eve_example_init(){
	EVE_HalContext *s_pHalContext = &g_Host;

	Gpu_Init(s_pHalContext);

	// read and store calibration setting
	//Esd_Calibrate(s_pHalContext);
	//Calibration_Save(s_pHalContext);

	Draw_Text(s_pHalContext, "Eve_Example1");
}
