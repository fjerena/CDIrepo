/*
 * GENERAL_DEF.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#include "GENERAL_DEF.h"

calibrationBlock calibFlashBlock;
systemInfoBlock sysInfoBlock;

const systemInfoBlock Initial_SystemInfo = {0, 120, 0, 0, 0};

/*
2 different speed
Low  <  1200                                  // fixed advance ignition
High >= First breakpoint(with restriction %)  // according igntion map
%Due the sw conception, the first break point must be >= 1200rpm
Engine Speed: Stopped, Acceleration, Steady State, Decelerate
Engine > Cut_Ignition threshould -> Cut ignition complete in Overspeed
*/

const calibrationBlock Initial_Calibration = { 1, 28, 7500,
	                                            ////The first Engine Speed value in the array needs to be 1200 or greater than mandatory
                                              { 1300, 2000, 2500, 3000, 3500, 4000, 4500, 7000, 8000, 9000,12000,15000},
                                              //{  64,   64,   64,   64,   64,   64,   64,   64,   64,   64,    64,    64}, 90, 80, 10};
                                              //{  48,   48,   48,   48,   48,   48,   48,   48,   48,   48,    48,    48}, 90, 80, 10};
                                              //{  32,   32,   32,   32,   32,   32,   32,   32,   32,   32,    32,    32}, 90, 80, 10};
                                              //{  16,   16,   16,   16,   16,   16,   16,   16,   16,   16,    16,    16}, 90, 80, 10};
                                              //{   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0}, 90, 80, 10};
                                              //{  64,   54,   44,   39,   36,   32,   32,   36,   40,   45,     55,     64}, 90, 80, 10};
                                              {   64,   58,   48,   38,   25,   15,    0,    0,   40,   45,   55,   64}, 90, 80, 10};
                                              //64 -> 18 degree, calib_table = 64-ang_obj+18 <-> ang_obj = 64-calib_table+		

volatile system_vars scenario = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	
void Set_Diagnose(uint8_t diagnose)
{
		sysInfoBlock.systemInfo_RAM.diagCode=(sysInfoBlock.systemInfo_RAM.diagCode|diagnose);
}	

void Clear_Diagnose(uint8_t diagnose)
{
		sysInfoBlock.systemInfo_RAM.diagCode=(sysInfoBlock.systemInfo_RAM.diagCode&(~diagnose));
}	

void Clear_All_Diagnoses(void)
{
		sysInfoBlock.systemInfo_RAM.diagCode=0xFF;		
}
