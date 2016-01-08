#ifndef __A36772_CONFIG_H
#define __A36772_CONFIG_H

/*
  We have a couple of compile time options
  __MODE_CAN_INTERFACE        
    In this mode, the gun driver is controlled over the CAN interface.
    The descrete digital/analog is not used.
    R126 and R127 should be installed

  __MODE_POT_INTERFACE
    In this mode, the gun driver is controlled by discrete fiber or signal lines.
    The pulse top, high voltage, and heater references are generated from the on board pots
    R126 and R127 should not be installed 

  __MODE_DISCRETE_INTERFACE
    In this mode, the gun driver is controlled by discrete fiber or signal lines.
    The pulse top, high voltage, and heater references are generated from the external interface
    R126 and R127 should not be installed 

  __OPTION_ENABLE_CAN
    This is only valid for __MODE_POT_INTERFACE and __MODE_DISCRETE_INTERFACE
    This allows the CAN port to be used for test and debugging while operating in one of these modes

 */

#define __MODE_CAN_INTERFACE        //used to be commentted out
//#define __MODE_POT_INTERFACE      //used to not be commentted out
//#define __MODE_DISCRETE_INTERFACE
//#define __OPTION_ENABLE_CAN       //used to not be commentted out

// Make sure that at least more mode is selected
#ifndef __MODE_CAN_INTERFACE
#ifndef __MODE_POT_INTERFACE
#ifndef __MODE_DISCRETE_INTERFACE
#error "No reference Source Selected"
#endif
#endif
#endif


// Create and check compile time options based on configuration above
#ifdef __MODE_CAN_INTERFACE
#define __CAN_CONTROLS
#define __CAN_ENABLED
#define __CAN_REFERENCE
#ifdef __OPTION_ENABLE_CAN
#error "OPTION_ENABLE_CAN not valid modifier to MODE_CAN_INTERFACE"
#endif
#endif


#ifdef __MODE_DISCRETE_INTERFACE
#define __DISCRETE_REFERENCE
#define __DISCRETE_CONTROLS
#ifdef  __CAN_REFERENCE
#error "Multiple references selected"
#endif
#endif

#ifdef __MODE_POT_INTERFACE
#define __POT_REFERENCE
#define __DISCRETE_CONTROLS
#ifdef  __CAN_REFERENCE
#error "Multiple references selected"
#endif
#ifdef  __DISCRETE_REFERENCE
#error "Multiple references selected"
#endif
#endif

#ifdef __OPTION_ENABLE_CAN
#define __CAN_ENABLED
#endif

// ----------- Gun Driver Load Specific Parameters ----------------------
//#define __LOAD_LINAC_GUN
#define __LOAD_RESISTIVE
//#define __LOAD_TEST_GUN

#ifndef __LOAD_LINAC_GUN
#ifndef __LOAD_RESISTIVE
#ifndef __LOAD_TEST_GUN
#error "No Load Selected"
#endif
#endif
#endif

#ifdef __LOAD_LINAC_GUN
#ifdef  __LOAD_RESISTIVE
#error "Multiple Loads Selected"
#endif
#ifdef  __LOAD_TEST_GUN
#error "Multiple Loads Selected"
#endif
#define HEATER_RAMP_TIME                 24000
#define MAX_RAMP_HTR_I                   1600           // 1.600 Amps
#define HTR_OC_ABS                       1750           // 1.750 Amps
#define GUN_DRIVER_LOAD_TYPE             0
#endif

#ifdef __LOAD_RESISTIVE
#ifdef  __LOAD_TEST_GUN
#error "Multiple Loads Selected"
#endif
#define HEATER_RAMP_TIME                 24000
#define MAX_RAMP_HTR_I                   1600           // 1.600 Amps
#define HTR_OC_ABS                       1750           // 1.750 Amps
#define GUN_DRIVER_LOAD_TYPE             1
#endif

#ifdef __LOAD_TEST_GUN
#define HEATER_RAMP_TIME                 60000
#define MAX_RAMP_HTR_I                   3000           // 3.000 Amps
#define HTR_OC_ABS                       3200           // 3.200 Amps
#define GUN_DRIVER_LOAD_TYPE             2
#endif



// ----------- Timers configurations - ALL Times are in 10ms Units --------------------
#define LED_STARTUP_FLASH_TIME                500      // Time LEDs will flash at startup
#define MAX_HEATER_RAMP_UP_TIME               HEATER_RAMP_TIME    // If the heater does not reach it's programed voltage in this time a fault will be generated
#define HEATER_AUTO_RESTART_TIME              500      // Time delay between a heater fault and when the heater gets restarted
#define HEATER_RAMP_UP_TIME_PERIOD            5        // Durring heater ramp up, the heater voltage will be increased every N 10ms (see HEATER_RAMP_UP_INCREMENT)
#define GUN_DRIVER_POWER_SUPPLY_STATUP_TIME   100      // Wait this long between enabling High Voltage / Pulse Top / Bias and cheching that they are at correct values

// System control Parameters
#define MAX_HEATER_CURRENT_DURING_RAMP_UP     MAX_RAMP_HTR_I     // mA Units.  Whenever the heater voltage is increased (ramp-up or increasing the set point).  The voltage will be current limited by this current
#define MAX_CONVERTER_LOGIC_ADC_READ_ERRORS   20       // If the ADC read exceeds this number a fault will be created
#define MAX_HEATER_START_UP_ATTEMPTS          5        // If the heater ramp up process does not succeed in this many attempts, a fault will be generated that requires power cycle
#define MAX_DAC_TX_ATTEMPTS                   10       // The pic will attempt to write to the Converter Logic DAC this many times before giving up
#define HEATER_RAMP_UP_INCREMENT              50       // mV Units.  When ramping up the heater voltage it is increased by this amount each HEATER_RAMP_UP_TIME_PERIOD




#define HEATER_VOLTAGE_CURRENT_LIMITED_FAULT_TIME (500 / HEATER_RAMP_UP_TIME_PERIOD)  // 5 Seconds


#ifdef __CAN_CONTROLS
#define HEATER_WARM_UP_TIME 100 //18000     // In Can control mode the heater warm up time is enforced by the ECB
#else
#define HEATER_WARM_UP_TIME 12000 // 2 minutes
#endif





// ------------- Converter Logic Board ADC Input Settings ---------------------

// DPARKER figure out how to convert the temperature 
// It is in 2's compliment  This only works for positive temperatures
#define ADC_TEMPERATURE_SENSOR_FIXED_SCALE    1.25
#define ADC_TEMPERATURE_SENSOR_FIXED_OFFSET   0


#define ADC_HV_VMON_FIXED_SCALE               .34722
#define ADC_HV_VMON_FIXED_OFFSET              0
#define ADC_HV_VMON_RELATIVE_TRIP_SCALE       MACRO_DEC_TO_CAL_FACTOR_2(.2)
#define ADC_HV_VMON_RELATIVE_TRIP_FLOOR       1000                     
#define ADC_HV_VMON_RELATIVE_TRIP_COUNT       50                                // 500mS


#define ADC_HV_IMON_FIXED_SCALE               .10419
#define ADC_HV_IMON_FIXED_OFFSET              0


#define ADC_GUN_I_PEAK_FIXED_SCALE            .17313
#define ADC_GUN_I_PEAK_FIXED_OFFSET           0


#define ADC_HTR_V_MON_FIXED_SCALE             .13875
#define ADC_HTR_V_MON_FIXED_OFFSET            0
#define ADC_HTR_V_MON_RELATIVE_TRIP_SCALE     MACRO_DEC_TO_CAL_FACTOR_2(.2)
#define ADC_HTR_V_MON_RELATIVE_TRIP_FLOOR     200                               // Minimum 200mV
#define ADC_HTR_V_MON_RELATIVE_TRIP_COUNT     50                                // 500mS


#define ADC_HTR_I_MON_FIXED_SCALE             .10419
#define ADC_HTR_I_MON_FIXED_OFFSET            0
#define ADC_HTR_I_MON_OVER_LIMIT_ABSOLUTE     HTR_OC_ABS                        // 1.750 Amps
#define ADC_HTR_I_MON_UNDER_LIMIT_ABSOLUTE    200                               // 0.200 Amps
#define ADC_HTR_I_MON_ABSOLUTE_TRIP_TIME      50                                // 500mS


#define ADC_TOP_V_MON_FIXED_SCALE             .69438
#define ADC_TOP_V_MON_FIXED_OFFSET            0
#define ADC_TOP_V_MON_RELATIVE_TRIP_SCALE     MACRO_DEC_TO_CAL_FACTOR_2(.2)
#define ADC_TOP_V_MON_RELATIVE_TRIP_FLOOR     1000                              // 10 Volts
#define ADC_TOP_V_MON_RELATIVE_TRIP_TIME      50                                // 500mS
//#define ADC_TOP_V_MON_RELATIVE_TRIP_TIME      200                               // 2S


#define ADC_BIAS_V_MON_FIXED_SCALE            .34688
#define ADC_BIAS_V_MON_FIXED_OFFSET           0
#define ADC_BIAS_V_MON_OVER_LIMIT_ABSOLUTE    18000                             // -180V
#define ADC_BIAS_V_MON_UNDER_LIMIT_ABSOLUTE   14000                             // -140V
#define ADC_BIAS_V_MON_ABSOLUTE_TRIP_TIME     50                                // 500mS 


#define ADC_24_V_MON_FIXED_SCALE              .41688
#define ADC_24_V_MON_FIXED_OFFSET             0


#define ADC_TEMPERATURE_MON_FIXED_SCALE       .08331
#define ADC_TEMPERATURE_MON_FIXED_OFFSET      20400



// --------------------- Converter Logic Board DAC output Settings -------------- //
#define DAC_HIGH_VOLTAGE_FIXED_SCALE          3.0000
#define DAC_HIGH_VOLTAGE_FIXED_OFFSET         0
#define HIGH_VOLTAGE_MAX_SET_POINT            20000                             // -20KV
#define HIGH_VOLTAGE_MIN_SET_POINT            5000                              // -5KV


#define DAC_TOP_VOLTAGE_FIXED_SCALE           1.5000
#define DAC_TOP_VOLTAGE_FIXED_OFFSET          0
#define TOP_VOLTAGE_MAX_SET_POINT             26000                             // 180V
#define TOP_VOLTAGE_MIN_SET_POINT             0                                 // -80V


#define DAC_HEATER_VOLTAGE_FIXED_SCALE        7.5188
#define DAC_HEATER_VOLTAGE_FIXED_OFFSET       0
#define HEATER_VOLTAGE_MAX_SET_POINT          8000                              // 8V
#define HEATER_VOLTAGE_MIN_SET_POINT          0                                 // 0V



// ------------- A36772 Internal PIC ADC Input Settings --------------------- //
#define POT_HTR_FIXED_SCALE                   .15625
#define POT_HTR_FIXED_OFFSET                  0

#define POT_VTOP_FIXED_SCALE                  .78125
#define POT_VTOP_FIXED_OFFSET                 0

#define POT_EK_FIXED_SCALE                    .42230
#define POT_EK_FIXED_OFFSET                   0

#define REF_HTR_FIXED_SCALE                   .15625
#define REF_HTR_FIXED_OFFSET                  0

#define REF_VTOP_FIXED_SCALE                  .78125
#define REF_VTOP_FIXED_OFFSET                 0

#define REF_EK_FIXED_SCALE                    .42230
#define REF_EK_FIXED_OFFSET                   0



// ------------- A36772 Onboard DAC Output Settings --------------------- //
#define DAC_MONITOR_HEATER_VOLTAGE_FIXED_SCALE    5.3333
#define DAC_MONITOR_HEATER_VOLTAGE_FIXED_OFFSET   0

#define DAC_MONITOR_HEATER_CURRENT_FIXED_SCALE    10.6667
#define DAC_MONITOR_HEATER_CURRENT_FIXED_OFFSET   0

#define DAC_MONITOR_CATHODE_VOLTAGE_FIXED_SCALE   1.9733
#define DAC_MONITOR_CATHODE_VOLTAGE_FIXED_OFFSET  0

#define DAC_MONITOR_GRID_VOLTAGE_FIXED_SCALE      1.0667
#define DAC_MONITOR_GRID_VOLTAGE_FIXED_OFFSET     0


#endif
