/*
  cap_control.h - capacitance controller
*/

#ifndef cap_control_h
#define cap_control_h 

// Initializes the motion_control subsystem resources
void cc_init();
void cc_measure_cap(int selection);
int  cc_axisAverageCapValue(int axis, uint8_t numSamples);
int  cc_endMillAverageCapValue(uint8_t numSamples);
float cc_getAverageVal();

#endif
