/*
 * GENERAL_DEF.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#include "GENERAL_DEF.h"

calibrationBlock calibFlashBlock;

const calibrationBlock Initial_Calibration = { 28, 7500,
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
	
programSheet request = {0, {{0,0}, {0,0}, {0,0}, {0,0}}};
programSheet Pulse_Program = { 0, {{0,0}, {0,0}, {0,0}, {0,0}}};

enum Interruption_type int_types=INT_FROM_CH1;
enum Event_status status=EMPTY;
enum Engine_States engstates=STOPPED;
enum engineSpeed pulseMngmt=LOW;
