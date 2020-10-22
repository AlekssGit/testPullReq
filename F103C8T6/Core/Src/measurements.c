#include "measurements.h"



void measure_data()
{
	  U_AB 				= (filtr_U_ac_dc* 8850)/100000;		//(filtr_U_ac_dc* 8057)/100000;
	  U_ac_dc 			= (filtr_U_AB * 8660)/100000;		//(filtr_U_AB * 8057)/100000;
	  I_shunt 			= (filtr_I_shunt*8057)/100000;
	  temperature	 	= (((filtr_temperature*330000)/4096) - 273150)/100;
}

void init_filtr()
{
	filtr_U_ac_dc 		= 0;
	filtr_U_AB 			= 0;
	filtr_I_shunt 		= 0;
	filtr_temperature 	= 0;
}


void mean_vals()
{

	filtr_U_ac_dc 		= (adc_1[0] + filtr_U_ac_dc		*(FILTR_NUM-1)) / FILTR_NUM;
	filtr_U_AB 			= (adc_1[1] + filtr_U_AB		*(FILTR_NUM-1)) / FILTR_NUM;
	filtr_I_shunt 		= (adc_1[2] + filtr_I_shunt		*(FILTR_NUM-1)) / FILTR_NUM;
	filtr_temperature 	= (adc_1[3] + filtr_temperature	*(FILTR_NUM-1)) / FILTR_NUM;
}

