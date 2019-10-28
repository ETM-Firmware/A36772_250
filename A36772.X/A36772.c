// This is firmware for the Gun Driver Board


// DPARKER - IF there is an error writing to the offboard DAC we probably need to re-write the entire DAC to ensure that data is correct

#include "A36772.h"
#include "A36772_CONFIG.h"

_FOSC(EC & CSW_FSCM_OFF);
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8); // 8 Second watchdog timer
_FBORPOR(PWRT_64 & PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


//void ETMCanSpoofPulseSyncNextPulseLevel();
//void ETMCanSpoofAFCHighSpeedDataLog();
unsigned int next_pulse_count = 0;
unsigned int spoof_counter = 0;
unsigned int dac_resets_debug = 0;

void DoStateMachine(void); // This handles the state machine for the interface board
void InitializeA36772(void); // Initialize the A36772 for operation
void DoStartupLEDs(void); // Used to flash the LEDs at startup
void ResetAllFaultInfo(void); // Clears all fault/status bits and resets all fault/status counters
unsigned int CheckHeaterFault(void); // Check for any fault that requires the heater to be turned off
unsigned int CheckFault(void); // Checks for any fault that does not require the heater to be turned off
unsigned int CheckPreTopFault(void); // Same as CheckFault(), ignoring top under voltage fault
unsigned int CheckPreHVFault(void); // Same as CheckFault(), ignoring hv and top under voltage faults
unsigned int CheckRampingHeaterFault(void); //Faults for before heater warmup time elapses

// Helper functions for DoA36772
void DoA36772(void);
/*
  DoA36772 is called every time the processor cycles throuh it's control loop
  If _T2IF is set (indicateds 10mS has passed) it executes everything that happens on 10mS time scale
 */
void UpdateFaults(void); // Update the fault bits based on analog/digital parameters
void UpdateLEDandStatusOutuputs(void); // Updates the LED and status outputs based on the system state
void WatchdogCheck(void); // Checks for SPI communication fails
/*
  Helper Function used to Enable/Disable Supplies on Converter logic board
 */
void EnableHeater(void);
void DisableHeater(void);
void EnableHighVoltage(void);
void DisableHighVoltage(void);
void EnableTopSupply(void);
void EnableBeam(void);
void DisableBeam(void);


/*
  -------------------- Converter Logic Board Helper Functions -----------------------
 */
void ResetFPGA(void);
/* 
   Resets the Converter Logic Board - 
   This Clears the on Board DAC so all outputs are set to zero
 */
void ADCConfigure(void);
/* 
   Configures the ADC module on the Converter Logic Board
   Average (16x) all 16 inputs + internal temperature sensor
 */
void ADCStartAcquisition(void);
/* 
   Start the configured acquisition sequence
   This will start an automated acquistion that will read each input 16 times and store results
   (along with the temeperature) into the FIFO buffer
 */
void UpdateADCResults(void);
/* 
   Read 34 bytes from the converter logic ADC FIFO buffer, 
   perform basic error checking on the data 
   If data is valid, scale/calibrate readings and move the values to AnalogInput
 */
void DACWriteChannel(unsigned int command_word, unsigned int data_word);
/*
  Writes a single channel to the DAC on the converter logic board
 */
void FPGAReadData(void);
/*
  This reads 32 bits of data from the FPGA
  It checks that Major rev matches and stores the status information
 */
unsigned char SPICharInvertered(unsigned char transmit_byte);
/*
  The fiberoptic inverter the data line
  This function inverters the send data before transmitting 
  and inverts the received data before returning it.
 */



// Digital Input Functions (NEEDS and ETM Module)
//void ETMDigitalInitializeInput(TYPE_DIGITAL_INPUT* input, unsigned int initial_value, unsigned int filter_time);
//void ETMDigitalUpdateInput(TYPE_DIGITAL_INPUT* input, unsigned int current_value);





// -------------------------- GLOBAL VARIABLES --------------------------- //
TYPE_GLOBAL_DATA_A36772 global_data_A36772;
LTC265X U32_LTC2654;

int main(void) {
    global_data_A36772.control_state = STATE_START_UP;
    while (1) {
        DoStateMachine();
    }
}

void DoStateMachine(void) {
    switch (global_data_A36772.control_state) {


        case STATE_START_UP:
            InitializeA36772();
            DisableBeam();
            DisableHighVoltage();
            DisableHeater();
            _CONTROL_NOT_CONFIGURED = 1;
            _CONTROL_NOT_READY = 1;
            _FAULT_SPI_COMMUNICATION = 0;
            global_data_A36772.control_config = 0;
            global_data_A36772.heater_start_up_attempts = 0;
            global_data_A36772.run_time_counter = 0;
            global_data_A36772.watchdog_counter = 0;
            global_data_A36772.reset_active = 0;

#ifndef __CAN_REFERENCE
            _CONTROL_NOT_CONFIGURED = 0;
#endif
            global_data_A36772.control_state = STATE_WAIT_FOR_CONFIG;
            break;


        case STATE_WAIT_FOR_CONFIG:
            DisableBeam();
            DisableHighVoltage();
            DisableHeater();
            while (global_data_A36772.control_state == STATE_WAIT_FOR_CONFIG) {
                DoA36772();
                DoStartupLEDs();
                if ((global_data_A36772.run_time_counter >= LED_STARTUP_FLASH_TIME) && (_CONTROL_NOT_CONFIGURED == 0)) {
                    global_data_A36772.control_state = STATE_RESET_FPGA;
                }
            }
            break;


        case STATE_RESET_FPGA:
            ResetFPGA();
            global_data_A36772.control_state = STATE_HEATER_RAMP_UP;
            break;


        case STATE_HEATER_RAMP_UP:
            _CONTROL_NOT_READY = 1;
            global_data_A36772.analog_output_heater_voltage.set_point = 0;
            global_data_A36772.set_current_reached = 0;
            global_data_A36772.heater_ramp_interval = 0;
            global_data_A36772.heater_operational = 0;
            global_data_A36772.heater_start_up_attempts++;
            global_data_A36772.initial_ramp_timer = 0;
            //    global_data_A36772.watchdog_counter = 0;
            global_data_A36772.watchdog_set_mode = WATCHDOG_MODE_0;
            global_data_A36772.dac_digital_watchdog_oscillator = WATCHDOG_VALUE_0;
            DisableBeam();
            DisableHighVoltage();
            EnableHeater();
            DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_H, global_data_A36772.dac_digital_watchdog_oscillator);
            while (global_data_A36772.control_state == STATE_HEATER_RAMP_UP) {
                DoA36772();
                if (global_data_A36772.set_current_reached == 1) {
                    global_data_A36772.control_state = STATE_HEATER_WARM_UP;
                }
                if (global_data_A36772.initial_ramp_timer > MAX_INITIAL_RAMP_TIME) {
                    global_data_A36772.control_state = STATE_FAULT_WARMUP_HEATER_OFF;
                }
                if (CheckRampingHeaterFault()) {
                    global_data_A36772.control_state = STATE_FAULT_WARMUP_HEATER_OFF;
                }
                if (ETMCanSlaveGetSyncMsgGunDriverDisableHeater()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
            }
            break;


        case STATE_HEATER_WARM_UP:
            _CONTROL_NOT_READY = 1;
            DisableBeam();
            DisableHighVoltage();
            EnableHeater();
            _STATUS_HEATER_AT_OPERATING_CURRENT = 1;
            global_data_A36772.heater_operational = 0;
            while (global_data_A36772.control_state == STATE_HEATER_WARM_UP) {
                DoA36772();
                if (global_data_A36772.warmup_complete) {
                    global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
                }
                if (CheckRampingHeaterFault()) {
                    global_data_A36772.control_state = STATE_FAULT_WARMUP_HEATER_OFF;
                }
                if (ETMCanSlaveGetSyncMsgGunDriverDisableHeater()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
            }
            break;

        case STATE_HEATER_WARM_UP_DONE:
            _CONTROL_NOT_READY = 1;
            DisableBeam();
            DisableHighVoltage();
            EnableHeater();
            global_data_A36772.heater_start_up_attempts = 0;
            _STATUS_HEATER_AT_OPERATING_CURRENT = 1;
            global_data_A36772.heater_operational = 1;
            while (global_data_A36772.control_state == STATE_HEATER_WARM_UP_DONE) {
                DoA36772();
                if (global_data_A36772.request_hv_enable) {
                    global_data_A36772.control_state = STATE_POWER_SUPPLY_RAMP_UP;
                }
                if (CheckHeaterFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
                if (_STATUS_SPI_COM_FAULTED) {
                    global_data_A36772.control_state = STATE_FAULT_WARMUP_HEATER_OFF;
                }
                if (ETMCanSlaveGetSyncMsgGunDriverDisableHeater()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
            }
            break;


        case STATE_POWER_SUPPLY_RAMP_UP:
            _CONTROL_NOT_READY = 1;
            DisableBeam();
            EnableHighVoltage();
            global_data_A36772.heater_operational = 1;
            global_data_A36772.power_supply_startup_remaining = GUN_DRIVER_POWER_SUPPLY_STARTUP_TIME;
            while (global_data_A36772.control_state == STATE_POWER_SUPPLY_RAMP_UP) {
                DoA36772();

                if (global_data_A36772.power_supply_startup_remaining == 0) {
                    global_data_A36772.control_state = STATE_HV_ON;
                }
                if (!global_data_A36772.request_hv_enable) {
                    global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
                }
                if (CheckPreHVFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
                }
                if (CheckHeaterFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
                if (_STATUS_SPI_COM_FAULTED) {
                    global_data_A36772.control_state = STATE_FAULT_WARMUP_HEATER_OFF;
                }
                if (ETMCanSlaveGetSyncMsgGunDriverDisableHeater()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
            }
            break;


        case STATE_HV_ON:
            _CONTROL_NOT_READY = 1;
            DisableBeam();
            global_data_A36772.heater_operational = 1;
            _T3IF = 0; //wait 1s before next state
            while (global_data_A36772.control_state == STATE_HV_ON) {
                DoA36772();
                if (_T3IF) {
                    global_data_A36772.control_state = STATE_TOP_ON;
                }
                if (!global_data_A36772.request_hv_enable) {
                    global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
                }
                if (_FAULT_ADC_HV_V_MON_UNDER_RELATIVE) {
                    if (PIN_INTERLOCK_RELAY_CLOSED != ILL_PIN_CUSTOMER_HV_ON_ENABLE_HV) {
                        _STATUS_INTERLOCK_INHIBITING_HV = 1;
                    }
                }
                if (CheckPreTopFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
                }
                if (CheckHeaterFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
                if (_STATUS_SPI_COM_FAULTED) {
                    global_data_A36772.control_state = STATE_FAULT_WARMUP_HEATER_OFF;
                }
                if (ETMCanSlaveGetSyncMsgGunDriverDisableHeater()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
            }
            break;


        case STATE_TOP_ON:
            _CONTROL_NOT_READY = 1;
            DisableBeam();
            EnableTopSupply();
            global_data_A36772.heater_operational = 1;
            _T3IF = 0; //wait 1s before next state
            while (global_data_A36772.control_state == STATE_TOP_ON) {
                DoA36772();
                if (_T3IF) {
                    global_data_A36772.control_state = STATE_TOP_READY;
                }
                if (!global_data_A36772.request_hv_enable) {
                    global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
                }
                if (CheckPreTopFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
                }
                if (CheckHeaterFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
                if (_STATUS_SPI_COM_FAULTED) {
                    global_data_A36772.control_state = STATE_FAULT_WARMUP_HEATER_OFF;
                }
                if (ETMCanSlaveGetSyncMsgGunDriverDisableHeater()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
            }
            break;

        case STATE_TOP_READY:
            _CONTROL_NOT_READY = 1;
            DisableBeam();
            global_data_A36772.heater_operational = 1;
            _T3IF = 0; //wait 1s before next state
            while (global_data_A36772.control_state == STATE_TOP_READY) {
                DoA36772();
                if (_T3IF) {
                    global_data_A36772.control_state = STATE_BEAM_ENABLE;
                }
                if (!global_data_A36772.request_hv_enable) {
                    global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
                }
                if (CheckFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
                }
                if (CheckHeaterFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
                if (_STATUS_SPI_COM_FAULTED) {
                    global_data_A36772.control_state = STATE_FAULT_WARMUP_HEATER_OFF;
                }
                if (ETMCanSlaveGetSyncMsgGunDriverDisableHeater()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
            }
            break;


        case STATE_BEAM_ENABLE:
            EnableBeam();
            _CONTROL_NOT_READY = 0;
            global_data_A36772.heater_operational = 1;
            while (global_data_A36772.control_state == STATE_BEAM_ENABLE) {
                DoA36772();
                if (!global_data_A36772.request_hv_enable) {
                    global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
                }
                if (CheckFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
                }
                if (CheckHeaterFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
                if (_STATUS_SPI_COM_FAULTED) {
                    global_data_A36772.control_state = STATE_FAULT_WARMUP_HEATER_OFF;
                }
                if (ETMCanSlaveGetSyncMsgGunDriverDisableHeater()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
            }
            break;


        case STATE_FAULT_HEATER_ON:
            _CONTROL_NOT_READY = 1;
            DisableBeam();
            DisableHighVoltage();
            ETMAnalogClearFaultCounters(&global_data_A36772.input_hv_v_mon);
            ETMAnalogClearFaultCounters(&global_data_A36772.input_top_v_mon);
            while (global_data_A36772.control_state == STATE_FAULT_HEATER_ON) {
                DoA36772();
                if (global_data_A36772.reset_active) {
                    global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
                }
                if (CheckHeaterFault()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
                if (_STATUS_SPI_COM_FAULTED) {
                    global_data_A36772.control_state = STATE_FAULT_WARMUP_HEATER_OFF;
                }
                if (ETMCanSlaveGetSyncMsgGunDriverDisableHeater()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
            }
            break;


        case STATE_FAULT_HEATER_OFF:
            _CONTROL_NOT_READY = 1;
            _CONTROL_NOT_CONFIGURED = 1;
            global_data_A36772.control_config = 0;
            global_data_A36772.watchdog_counter = 0;
            DisableBeam();
            DisableHighVoltage();
            DisableHeater();
            ETMAnalogClearFaultCounters(&global_data_A36772.input_htr_v_mon);
            ETMAnalogClearFaultCounters(&global_data_A36772.input_htr_i_mon);
            ETMAnalogClearFaultCounters(&global_data_A36772.input_bias_v_mon);
            while (global_data_A36772.control_state == STATE_FAULT_HEATER_OFF) {
                DoA36772();
                if ((global_data_A36772.reset_active != 0) &&
                        (ETMCanSlaveGetSyncMsgGunDriverDisableHeater() == 0) &&
                        (_FAULT_REGISTER == 0)) {
                    global_data_A36772.control_state = STATE_WAIT_FOR_CONFIG;
                }
            }
            break;


        case STATE_FAULT_WARMUP_HEATER_OFF:
            _CONTROL_NOT_READY = 1;
            _CONTROL_NOT_CONFIGURED = 1;
            global_data_A36772.control_config = 0;
            DisableBeam();
            DisableHighVoltage();
            DisableHeater();
            ETMCanSlaveDoCan();
            //    ETMAnalogClearFaultCounters(&global_data_A36772.input_htr_v_mon);
            //    ETMAnalogClearFaultCounters(&global_data_A36772.input_htr_i_mon);
            //    ETMAnalogClearFaultCounters(&global_data_A36772.input_bias_v_mon);
            ResetAllFaultInfo();
            global_data_A36772.fault_restart_remaining = HEATER_AUTO_RESTART_TIME;
            while (global_data_A36772.control_state == STATE_FAULT_WARMUP_HEATER_OFF) {
                DoA36772();
                if (global_data_A36772.fault_restart_remaining == 0) {
                    global_data_A36772.control_state = STATE_WAIT_FOR_CONFIG;
                }
                if (global_data_A36772.heater_start_up_attempts > MAX_HEATER_START_UP_ATTEMPTS) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_FAILURE;
                }
                if (ETMCanSlaveGetSyncMsgGunDriverDisableHeater()) {
                    global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
                }
            }
            break;


        case STATE_FAULT_HEATER_FAILURE:
            _CONTROL_NOT_READY = 1;
            DisableBeam();
            DisableHighVoltage();
            DisableHeater();
            _FAULT_HEATER_STARTUP_FAILURE = 1;
            while (global_data_A36772.control_state == STATE_FAULT_HEATER_FAILURE) {
                // Can't leave this state without power cycle
                DoA36772();
            }
            break;


        default:
            global_data_A36772.control_state = STATE_FAULT_HEATER_FAILURE;
            break;

    }
}

void InitializeA36772(void) {

    // Initialize the status register and load the inhibit and fault masks
    _FAULT_REGISTER = 0;
    _CONTROL_REGISTER = 0;
    _WARNING_REGISTER = 0;
    _NOT_LOGGED_REGISTER = 0;

    // --------- BEGIN IO PIN CONFIGURATION ------------------

    // Initialize Ouput Pin Latches BEFORE setting the pins to Output
    PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
    PIN_CS_ADC = !OLL_PIN_CS_ADC_SELECTED;
    PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;

    // ---- Configure the dsPIC ADC Module Analog Inputs------------ //
    ADPCFG = 0xFFFF; // all are digital I/O

    // Initialize all I/O Registers
    TRISA = A36772_TRISA_VALUE;
    TRISB = A36772_TRISB_VALUE;
    TRISC = A36772_TRISC_VALUE;
    TRISD = A36772_TRISD_VALUE;
    TRISF = A36772_TRISF_VALUE;
    TRISG = A36772_TRISG_VALUE;

    // Config SPI1 for Gun Driver
    ConfigureSPI(ETM_SPI_PORT_1, A36772_SPI1CON_VALUE, 0, A36772_SPI1STAT_VALUE, SPI_CLK_1_MBIT, FCY_CLK);


    // ---------- Configure Timers ----------------- //

    // Initialize TMR2
    PR2 = A36772_PR2_VALUE;
    TMR2 = 0;
    _T2IF = 0;
    _T2IP = 5;
    T2CON = A36772_T2CON_VALUE;

    // Initialize TMR3
    PR3 = A36772_PR3_VALUE;
    TMR3 = 0;
    _T3IF = 0;
    _T3IP = 5;
    T3CON = A36772_T3CON_VALUE;

    // Configure on-board DAC
    SetupLTC265X(&U32_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);

    //Configure EEPROM
    ETMEEPromUseExternal();
    ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);

    // ------------- Configure Internal ADC --------- //
    ADCON1 = ADCON1_SETTING; // Configure the high speed ADC module based on H file parameters
    ADCON2 = ADCON2_SETTING; // Configure the high speed ADC module based on H file parameters
    ADCON3 = ADCON3_SETTING; // Configure the high speed ADC module based on H file parameters
    ADCHS = ADCHS_SETTING; // Configure the high speed ADC module based on H file parameters

    ADPCFG = ADPCFG_SETTING; // Set which pins are analog and which are digital I/O
    ADCSSL = ADCSSL_SETTING; // Set which analog pins are scanned

    _ADIF = 0;
    _ADIP = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)
    _ADIE = 1;
    _ADON = 1;
	
	
	//---------Configure Resistance Limit Variables---------//
	global_data_A36772.filament_resistance = 0;
	global_data_A36772.heater_voltage_double = 0;
	global_data_A36772.heater_current_double = 0;
	global_data_A36772.scaled_filament_resistance = 0;
	global_data_A36772.scaled_filament_resistance_for_display = 0;
	global_data_A36772.resistance_warmup_delay = 0;
	global_data_A36772.filament_resistance_limit = 3150;

#ifdef __CAN_ENABLED
    // Initialize the Can module
    ETMCanSlaveInitialize(CAN_PORT_2, FCY_CLK, ETM_CAN_ADDR_GUN_DRIVER_BOARD, _PIN_RC3, 4, _PIN_RC3, _PIN_RC3);
    ETMCanSlaveLoadConfiguration(36772, 250, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_MINOR_REV);
#endif

    ADCConfigure();

    // Initialize off board ADC Inputs
    ETMAnalogInitializeInput(&global_data_A36772.input_adc_temperature,
            MACRO_DEC_TO_SCALE_FACTOR_16(ADC_TEMPERATURE_SENSOR_FIXED_SCALE),
            ADC_TEMPERATURE_SENSOR_FIXED_OFFSET,
            ANALOG_INPUT_NO_CALIBRATION,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);


    ETMAnalogInitializeInput(&global_data_A36772.input_hv_v_mon,
            MACRO_DEC_TO_SCALE_FACTOR_16(ADC_HV_VMON_FIXED_SCALE),
            ADC_HV_VMON_FIXED_OFFSET,
            ANALOG_INPUT_0,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            ADC_HV_VMON_RELATIVE_TRIP_SCALE,
            ADC_HV_VMON_RELATIVE_TRIP_FLOOR,
            ADC_HV_VMON_RELATIVE_TRIP_COUNT,
            NO_ABSOLUTE_COUNTER);

    ETMAnalogInitializeInput(&global_data_A36772.input_hv_i_mon,
            MACRO_DEC_TO_SCALE_FACTOR_16(ADC_HV_IMON_FIXED_SCALE),
            ADC_HV_IMON_FIXED_OFFSET,
            ANALOG_INPUT_1,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);


    ETMAnalogInitializeInput(&global_data_A36772.input_gun_i_peak,
            MACRO_DEC_TO_SCALE_FACTOR_16(ADC_GUN_I_PEAK_FIXED_SCALE),
            ADC_GUN_I_PEAK_FIXED_OFFSET,
            ANALOG_INPUT_2,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);

    ETMAnalogInitializeInput(&global_data_A36772.input_htr_v_mon,
            MACRO_DEC_TO_SCALE_FACTOR_16(ADC_HTR_V_MON_FIXED_SCALE),
            ADC_HTR_V_MON_FIXED_OFFSET,
            ANALOG_INPUT_3,
            ADC_HTR_V_MON_OVER_LIMIT_ABSOLUTE,
            ADC_HTR_V_MON_UNDER_LIMIT_ABSOLUTE,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            ADC_HTR_V_MON_ABSOLUTE_TRIP_TIME);

    ETMAnalogInitializeInput(&global_data_A36772.input_htr_i_mon,
            MACRO_DEC_TO_SCALE_FACTOR_16(ADC_HTR_I_MON_FIXED_SCALE),
            ADC_HTR_I_MON_FIXED_OFFSET,
            ANALOG_INPUT_4,
            ADC_HTR_I_MON_OVER_LIMIT_ABSOLUTE,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            ADC_HTR_I_MON_ABSOLUTE_TRIP_TIME);

    ETMAnalogInitializeInput(&global_data_A36772.input_top_v_mon,
            MACRO_DEC_TO_SCALE_FACTOR_16(ADC_TOP_V_MON_FIXED_SCALE),
            ADC_TOP_V_MON_FIXED_OFFSET,
            ANALOG_INPUT_5,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            ADC_TOP_V_MON_RELATIVE_TRIP_SCALE,
            ADC_TOP_V_MON_RELATIVE_TRIP_FLOOR,
            ADC_TOP_V_MON_RELATIVE_TRIP_TIME,
            NO_ABSOLUTE_COUNTER);

    ETMAnalogInitializeInput(&global_data_A36772.input_bias_v_mon,
            MACRO_DEC_TO_SCALE_FACTOR_16(ADC_BIAS_V_MON_FIXED_SCALE),
            ADC_BIAS_V_MON_FIXED_OFFSET,
            ANALOG_INPUT_6,
            ADC_BIAS_V_MON_OVER_LIMIT_ABSOLUTE,
            ADC_BIAS_V_MON_UNDER_LIMIT_ABSOLUTE,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            ADC_BIAS_V_MON_ABSOLUTE_TRIP_TIME);

    ETMAnalogInitializeInput(&global_data_A36772.input_24_v_mon,
            MACRO_DEC_TO_SCALE_FACTOR_16(ADC_24_V_MON_FIXED_SCALE),
            ADC_24_V_MON_FIXED_OFFSET,
            ANALOG_INPUT_7,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);

    ETMAnalogInitializeInput(&global_data_A36772.input_temperature_mon,
            MACRO_DEC_TO_SCALE_FACTOR_16(ADC_TEMPERATURE_MON_FIXED_SCALE),
            ADC_TEMPERATURE_MON_FIXED_OFFSET,
            ANALOG_INPUT_8,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);

    ETMAnalogInitializeInput(&global_data_A36772.input_dac_monitor,
            MACRO_DEC_TO_SCALE_FACTOR_16(1),
            0,
            ANALOG_INPUT_NO_CALIBRATION,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);



    // ----------------- Initialize PIC's internal ADC Inputs --------------------- //

    ETMAnalogInitializeInput(&global_data_A36772.pot_htr,
            MACRO_DEC_TO_SCALE_FACTOR_16(POT_HTR_FIXED_SCALE),
            POT_HTR_FIXED_OFFSET,
            ANALOG_INPUT_9,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);


    ETMAnalogInitializeInput(&global_data_A36772.pot_vtop,
            MACRO_DEC_TO_SCALE_FACTOR_16(POT_VTOP_FIXED_SCALE),
            POT_VTOP_FIXED_OFFSET,
            ANALOG_INPUT_A,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);

    ETMAnalogInitializeInput(&global_data_A36772.pot_ek,
            MACRO_DEC_TO_SCALE_FACTOR_16(POT_EK_FIXED_SCALE),
            POT_EK_FIXED_OFFSET,
            ANALOG_INPUT_B,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);


    ETMAnalogInitializeInput(&global_data_A36772.ref_htr,
            MACRO_DEC_TO_SCALE_FACTOR_16(REF_HTR_FIXED_SCALE),
            REF_HTR_FIXED_OFFSET,
            ANALOG_INPUT_9,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);


    ETMAnalogInitializeInput(&global_data_A36772.ref_vtop,
            MACRO_DEC_TO_SCALE_FACTOR_16(REF_VTOP_FIXED_SCALE),
            REF_VTOP_FIXED_OFFSET,
            ANALOG_INPUT_A,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);

    ETMAnalogInitializeInput(&global_data_A36772.ref_ek,
            MACRO_DEC_TO_SCALE_FACTOR_16(REF_EK_FIXED_SCALE),
            REF_EK_FIXED_OFFSET,
            ANALOG_INPUT_B,
            NO_OVER_TRIP,
            NO_UNDER_TRIP,
            NO_TRIP_SCALE,
            NO_FLOOR,
            NO_RELATIVE_COUNTER,
            NO_ABSOLUTE_COUNTER);






    // ------------- Initialize Converter Logic Board DAC Outputs ------------------------------ //
    ETMAnalogInitializeOutput(&global_data_A36772.analog_output_high_voltage,
            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_HIGH_VOLTAGE_FIXED_SCALE),
            DAC_HIGH_VOLTAGE_FIXED_OFFSET,
            ANALOG_OUTPUT_0,
            HIGH_VOLTAGE_MAX_SET_POINT,
            HIGH_VOLTAGE_MIN_SET_POINT,
            0);

    ETMAnalogInitializeOutput(&global_data_A36772.analog_output_top_voltage,
            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_TOP_VOLTAGE_FIXED_SCALE),
            DAC_TOP_VOLTAGE_FIXED_OFFSET,
            ANALOG_OUTPUT_1,
            TOP_VOLTAGE_MAX_SET_POINT,
            TOP_VOLTAGE_MIN_SET_POINT,
            0);

    ETMAnalogInitializeOutput(&global_data_A36772.analog_output_heater_voltage,
            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_HEATER_VOLTAGE_FIXED_SCALE),
            DAC_HEATER_VOLTAGE_FIXED_OFFSET,
            ANALOG_OUTPUT_2,
            HEATER_VOLTAGE_MAX_SET_POINT,
            HEATER_VOLTAGE_MIN_SET_POINT,
            0);


    // ----------------------- Initialize on Board DAC Outputs ---------------------------- //  
    ETMAnalogInitializeOutput(&global_data_A36772.monitor_heater_voltage,
            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_HEATER_VOLTAGE_FIXED_SCALE),
            DAC_MONITOR_HEATER_VOLTAGE_FIXED_OFFSET,
            ANALOG_OUTPUT_3,
            0xFFFF,
            0,
            0);

    ETMAnalogInitializeOutput(&global_data_A36772.monitor_heater_current,
            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_HEATER_CURRENT_FIXED_SCALE),
            DAC_MONITOR_HEATER_CURRENT_FIXED_OFFSET,
            ANALOG_OUTPUT_4,
            0xFFFF,
            0,
            0);

    ETMAnalogInitializeOutput(&global_data_A36772.monitor_cathode_voltage,
            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_CATHODE_VOLTAGE_FIXED_SCALE),
            DAC_MONITOR_CATHODE_VOLTAGE_FIXED_OFFSET,
            ANALOG_OUTPUT_5,
            0xFFFF,
            0,
            0);

    ETMAnalogInitializeOutput(&global_data_A36772.monitor_grid_voltage,
            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_GRID_VOLTAGE_FIXED_SCALE),
            DAC_MONITOR_GRID_VOLTAGE_FIXED_OFFSET,
            ANALOG_OUTPUT_6,
            0xFFFF,
            0,
            0);

    global_data_A36772.monitor_heater_voltage.enabled = 1;
    global_data_A36772.monitor_heater_current.enabled = 1;
    global_data_A36772.monitor_grid_voltage.enabled = 1;
    global_data_A36772.monitor_cathode_voltage.enabled = 1;

    ResetAllFaultInfo();
}

void DoStartupLEDs(void) {
    switch (((global_data_A36772.run_time_counter >> 4) & 0b11)) {

        case 0:
            PIN_LED_I2A = OLL_LED_ON;
            PIN_LED_I2B = !OLL_LED_ON;
            PIN_LED_I2C = !OLL_LED_ON;
            PIN_LED_I2D = !OLL_LED_ON;
            break;

        case 1:
            PIN_LED_I2A = !OLL_LED_ON;
            PIN_LED_I2B = OLL_LED_ON;
            PIN_LED_I2C = !OLL_LED_ON;
            PIN_LED_I2D = !OLL_LED_ON;
            break;

        case 2:
            PIN_LED_I2A = !OLL_LED_ON;
            PIN_LED_I2B = !OLL_LED_ON;
            PIN_LED_I2C = OLL_LED_ON;
            PIN_LED_I2D = !OLL_LED_ON;
            break;

        case 3:
            PIN_LED_I2A = !OLL_LED_ON;
            PIN_LED_I2B = !OLL_LED_ON;
            PIN_LED_I2C = !OLL_LED_ON;
            PIN_LED_I2D = OLL_LED_ON;
            break;
    }
}

void ResetAllFaultInfo(void) {
    _FAULT_FPGA_FIRMWARE_MAJOR_REV_MISMATCH = 0;
    _FAULT_ADC_HV_V_MON_OVER_RELATIVE = 0;
    _FAULT_ADC_HV_V_MON_UNDER_RELATIVE = 0;
    _FAULT_ADC_HTR_V_MON_OVER_ABSOLUTE = 0;
    _FAULT_ADC_HTR_V_MON_UNDER_ABSOLUTE = 0;
    _FAULT_ADC_HTR_I_MON_OVER_ABSOLUTE = 0;
    _FAULT_ADC_TOP_V_MON_OVER_RELATIVE = 0;
    _FAULT_ADC_TOP_V_MON_UNDER_RELATIVE = 0;
    _FAULT_ADC_BIAS_V_MON_OVER_ABSOLUTE = 0;
    _FAULT_ADC_BIAS_V_MON_UNDER_ABSOLUTE = 0;
    //  _FAULT_ADC_DIGITAL_WATCHDOG = 0;
    _FAULT_ADC_DIGITAL_ARC = 0;
    _FAULT_ADC_DIGITAL_OVER_TEMP = 0;
    //  _FAULT_ADC_DIGITAL_PULSE_WIDTH_DUTY = 0;
    _FAULT_ADC_DIGITAL_GRID = 0;
    _FAULT_CONVERTER_LOGIC_ADC_READ_FAILURE = 0;
    //  _FAULT_HEATER_RAMP_TIMEOUT = 0;
    _FAULT_HEATER_STARTUP_FAILURE = 0;
    _FAULT_CAN_COMMUNICATION = 0;
    //  _FAULT_SPI_COMMUNICATION = 0;
    _STATUS_INTERLOCK_INHIBITING_HV = 0;
    _STATUS_HEATER_AT_OPERATING_CURRENT = 0;
    _STATUS_CUSTOMER_HV_ON = 0;
    _STATUS_CUSTOMER_BEAM_ENABLE = 0;
    _STATUS_ADC_DIGITAL_HEATER_NOT_READY = 0;
    _STATUS_DAC_WRITE_FAILURE = 0;
    _STATUS_SPI_COM_FAULTED = 0;

    _FPGA_CONVERTER_LOGIC_PCB_REV_MISMATCH = 0;
    _FPGA_FIRMWARE_MINOR_REV_MISMATCH = 0;
    _FPGA_ARC_COUNTER_GREATER_ZERO = 0;
    _FPGA_ARC_HIGH_VOLTAGE_INHIBIT_ACTIVE = 0;
    //  _FPGA_HEATER_VOLTAGE_LESS_THAN_4_5_VOLTS       = 0;
    _FPGA_MODULE_TEMP_GREATER_THAN_65_C = 0;
    _FPGA_MODULE_TEMP_GREATER_THAN_75_C = 0;
    //  _FPGA_PULSE_WIDTH_LIMITING                     = 0;
    //  _FPGA_PRF_FAULT                                = 0;
    _FPGA_CURRENT_MONITOR_PULSE_WIDTH_FAULT = 0;
    _FPGA_GRID_MODULE_HARDWARE_FAULT = 0;
    _FPGA_GRID_MODULE_OVER_VOLTAGE_FAULT = 0;
    _FPGA_GRID_MODULE_UNDER_VOLTAGE_FAULT = 0;
    _FPGA_GRID_MODULE_BIAS_VOLTAGE_FAULT = 0;
    _FPGA_HV_REGULATION_WARNING = 0;
    _FPGA_DIPSWITCH_1_ON = 0;
    _FPGA_TEST_MODE_TOGGLE_SWITCH_TEST_MODE = 0;
    _FPGA_LOCAL_MODE_TOGGLE_SWITCH_LOCAL_MODE = 0;


    // Initialize Digital Input Filters for FPGA Status
    ETMDigitalInitializeInput(&global_data_A36772.fpga_coverter_logic_pcb_rev_mismatch, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_firmware_major_rev_mismatch, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_firmware_minor_rev_mismatch, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_arc, 0, 5);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_arc_high_voltage_inihibit_active, 0, 0);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_heater_voltage_less_than_4_5_volts, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_module_temp_greater_than_65_C, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_module_temp_greater_than_75_C, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_pulse_width_limiting_active, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_prf_fault, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_current_monitor_pulse_width_fault, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_grid_module_hardware_fault, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_grid_module_over_voltage_fault, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_grid_module_under_voltage_fault, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_grid_module_bias_voltage_fault, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_hv_regulation_warning, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_dipswitch_1_on, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_test_mode_toggle_switch_set_to_test, 0, 30);
    ETMDigitalInitializeInput(&global_data_A36772.fpga_local_mode_toggle_switch_set_to_local, 0, 30);

    // Initialize Digital Input Filters For ADC "Digital" Inputs
    ETMDigitalInitializeInput(&global_data_A36772.adc_digital_warmup_flt, 1, 30);
    ETMDigitalInitializeInput(&global_data_A36772.adc_digital_watchdog_flt, 1, 30);
    ETMDigitalInitializeInput(&global_data_A36772.adc_digital_arc_flt, 1, 30);
    ETMDigitalInitializeInput(&global_data_A36772.adc_digital_over_temp_flt, 1, 30);
    ETMDigitalInitializeInput(&global_data_A36772.adc_digital_pulse_width_duty_flt, 1, 30);
    ETMDigitalInitializeInput(&global_data_A36772.adc_digital_grid_flt, 1, 30);

    // Reset all the Analog input fault counters
    ETMAnalogClearFaultCounters(&global_data_A36772.input_adc_temperature);
    ETMAnalogClearFaultCounters(&global_data_A36772.input_hv_v_mon);
    ETMAnalogClearFaultCounters(&global_data_A36772.input_hv_i_mon);
    ETMAnalogClearFaultCounters(&global_data_A36772.input_gun_i_peak);
    ETMAnalogClearFaultCounters(&global_data_A36772.input_htr_v_mon);
    ETMAnalogClearFaultCounters(&global_data_A36772.input_htr_i_mon);
    ETMAnalogClearFaultCounters(&global_data_A36772.input_top_v_mon);
    ETMAnalogClearFaultCounters(&global_data_A36772.input_bias_v_mon);
    ETMAnalogClearFaultCounters(&global_data_A36772.input_24_v_mon);
    ETMAnalogClearFaultCounters(&global_data_A36772.input_dac_monitor);

    ETMAnalogClearFaultCounters(&global_data_A36772.pot_htr);
    ETMAnalogClearFaultCounters(&global_data_A36772.pot_vtop);
    ETMAnalogClearFaultCounters(&global_data_A36772.pot_ek);
    ETMAnalogClearFaultCounters(&global_data_A36772.ref_htr);
    ETMAnalogClearFaultCounters(&global_data_A36772.ref_vtop);
    ETMAnalogClearFaultCounters(&global_data_A36772.ref_ek);
    ETMAnalogClearFaultCounters(&global_data_A36772.pos_15v_mon);
    ETMAnalogClearFaultCounters(&global_data_A36772.neg_15v_mon);

    global_data_A36772.adc_read_error_test = 0;
    global_data_A36772.adc_read_error_count = 0;
    global_data_A36772.adc_read_ok = 1;

    global_data_A36772.dac_write_error_count = 0;
    global_data_A36772.dac_write_failure = 0;
    global_data_A36772.dac_write_failure_count = 0;
}

unsigned int CheckRampingHeaterFault(void) {
    unsigned int fault = 0;
    fault = _FAULT_FPGA_FIRMWARE_MAJOR_REV_MISMATCH;
    fault |= _FAULT_ADC_HTR_V_MON_OVER_ABSOLUTE;
    fault |= _FAULT_ADC_HTR_I_MON_OVER_ABSOLUTE;
    fault |= _FAULT_ADC_DIGITAL_OVER_TEMP;
    fault |= _FAULT_ADC_DIGITAL_GRID;
    fault |= _FAULT_CONVERTER_LOGIC_ADC_READ_FAILURE;
    fault |= _FAULT_ADC_BIAS_V_MON_OVER_ABSOLUTE;
    fault |= _FAULT_ADC_BIAS_V_MON_UNDER_ABSOLUTE;
    //  fault |= _FAULT_SPI_COMMUNICATION;
    fault |= _STATUS_SPI_COM_FAULTED;
    fault |= _FAULT_CAN_COMMUNICATION;
    if (fault) {
        return 1;
    } else {
        return 0;
    }
}

unsigned int CheckHeaterFault(void) {
    unsigned int fault = 0;
    fault = _FAULT_FPGA_FIRMWARE_MAJOR_REV_MISMATCH;
    fault |= _FAULT_ADC_HTR_V_MON_OVER_ABSOLUTE;
    fault |= _FAULT_ADC_HTR_V_MON_UNDER_ABSOLUTE;
    fault |= _FAULT_ADC_HTR_I_MON_OVER_ABSOLUTE;
    fault |= _FAULT_ADC_DIGITAL_OVER_TEMP;
    fault |= _FAULT_ADC_DIGITAL_GRID;
    fault |= _FAULT_CONVERTER_LOGIC_ADC_READ_FAILURE;
    fault |= _FAULT_ADC_BIAS_V_MON_OVER_ABSOLUTE;
    fault |= _FAULT_ADC_BIAS_V_MON_UNDER_ABSOLUTE;
    //  fault |= _FAULT_SPI_COMMUNICATION;
    fault |= _FAULT_CAN_COMMUNICATION;
    if (fault) {
        return 1;
    } else {
        return 0;
    }
}

unsigned int CheckFault(void) {
    unsigned int fault = 0;
    fault = _FAULT_ADC_HV_V_MON_OVER_RELATIVE;
    fault |= _FAULT_ADC_HV_V_MON_UNDER_RELATIVE;
    fault |= _FAULT_ADC_TOP_V_MON_OVER_RELATIVE;
    fault |= _FAULT_ADC_TOP_V_MON_UNDER_RELATIVE;
    fault |= _FAULT_ADC_DIGITAL_ARC;
    if (fault) {
        return 1;
    } else {
        return 0;
    }
}

unsigned int CheckPreTopFault(void) {
    unsigned int fault = 0;
    fault = _FAULT_ADC_HV_V_MON_OVER_RELATIVE;
    fault |= _FAULT_ADC_HV_V_MON_UNDER_RELATIVE;
    fault |= _FAULT_ADC_DIGITAL_ARC;
    if (fault) {
        return 1;
    } else {
        return 0;
    }
}

unsigned int CheckPreHVFault(void) {
    unsigned int fault = 0;
    fault = _FAULT_ADC_HV_V_MON_OVER_RELATIVE;
    fault |= _FAULT_ADC_DIGITAL_ARC;
    if (fault) {
        return 1;
    } else {
        return 0;
    }
}

void DoA36772(void) {

#ifdef __CAN_ENABLED
    ETMCanSlaveDoCan();
#endif

#ifndef __CAN_REQUIRED
    ClrWdt();
#endif


#ifdef __DISCRETE_CONTROLS
    if (PIN_CUSTOMER_HV_ON == ILL_PIN_CUSTOMER_HV_ON_ENABLE_HV) {
        global_data_A36772.request_hv_enable = 1;
        _STATUS_CUSTOMER_HV_ON = 1;
    } else {
        global_data_A36772.request_hv_enable = 0;
        _STATUS_CUSTOMER_HV_ON = 0;
    }

    if (PIN_CUSTOMER_BEAM_ENABLE == ILL_PIN_CUSTOMER_BEAM_ENABLE_BEAM_ENABLED) {
        global_data_A36772.request_beam_enable = 1;
        _STATUS_CUSTOMER_BEAM_ENABLE = 1;
    } else {
        global_data_A36772.request_beam_enable = 0;
        _STATUS_CUSTOMER_BEAM_ENABLE = 0;
    }
#endif

#ifdef __CAN_CONTROLS

    if (ETMCanSlaveGetPulseLevel()) {
        global_data_A36772.high_energy_pulse = 1;
    } else {
        global_data_A36772.high_energy_pulse = 0;
    }

    if (!ETMCanSlaveGetSyncMsgSystemHVDisable()) {
        global_data_A36772.request_hv_enable = 1;
        _STATUS_CUSTOMER_HV_ON = 1;
    } else {
        global_data_A36772.request_hv_enable = 0;
        _STATUS_CUSTOMER_HV_ON = 0;
    }

    if (!ETMCanSlaveGetSyncMsgPulseSyncDisableHV()) {
        global_data_A36772.warmup_complete = 1;
    } else {
        global_data_A36772.warmup_complete = 0;
    }

#endif

    if (_T2IF) {
        // Run once every 10ms
        _T2IF = 0;

        /*
        if (ETMCanSlaveGetSyncMsgHighSpeedLogging()) {
          spoof_counter++;
          if (spoof_counter >= 10) {
        spoof_counter = 0;
        next_pulse_count++;
        ETMCanSpoofPulseSyncNextPulseLevel();
        ETMCanSpoofAFCHighSpeedDataLog();
          }
        }
         */
#ifdef __CAN_CONTROLS
        if (ETMCanSlaveGetSyncMsgResetEnable()) {
            global_data_A36772.reset_active = 1;
        } else {
            global_data_A36772.reset_active = 0;
        }

        if (ETMCanSlaveGetSyncMsgClearDebug()) {
            global_data_A36772.reset_debug = 1;
        } else {
            global_data_A36772.reset_debug = 0;
        }

        if (ETMCanSlaveGetComFaultStatus()) {
            _FAULT_CAN_COMMUNICATION = 1;
        } else if (global_data_A36772.reset_active) {
            _FAULT_CAN_COMMUNICATION = 0;
        }
#endif

#ifdef __DISCRETE_CONTROLS
        if ((PIN_CUSTOMER_HV_ON == !ILL_PIN_CUSTOMER_HV_ON_ENABLE_HV) && (global_data_A36772.previous_state_pin_customer_hv_on == ILL_PIN_CUSTOMER_HV_ON_ENABLE_HV)) {
            global_data_A36772.reset_active = 1;
        } else {
            global_data_A36772.reset_active = 0;
        }
        global_data_A36772.previous_state_pin_customer_hv_on = PIN_CUSTOMER_HV_ON;
#endif

        if (_STATUS_INTERLOCK_INHIBITING_HV != 0) {
            if ((global_data_A36772.control_state < STATE_POWER_SUPPLY_RAMP_UP) ||
                    (PIN_INTERLOCK_RELAY_CLOSED == ILL_PIN_CUSTOMER_HV_ON_ENABLE_HV)) {
                _STATUS_INTERLOCK_INHIBITING_HV = 0;
            }
        }

        // Update to counter used to flash the LEDs at startup and time transmits to DACs
        if (global_data_A36772.power_supply_startup_remaining) {
            global_data_A36772.power_supply_startup_remaining--;
        }

        if (global_data_A36772.fault_restart_remaining) {
            global_data_A36772.fault_restart_remaining--;
        }

        global_data_A36772.initial_ramp_timer++;

        //    global_data_A36772.watchdog_counter++;
        global_data_A36772.run_time_counter++;

        if (global_data_A36772.run_time_counter & 0x0010) {
            PIN_LED_OPERATIONAL = 1;
        } else {
            PIN_LED_OPERATIONAL = 0;
        }

        // Update Data from the FPGA
        FPGAReadData();

        // Read all the data from the external ADC
        UpdateADCResults();

        // Start the next acquisition from the external ADC
        ADCStartAcquisition();

        if (global_data_A36772.control_state != STATE_WAIT_FOR_CONFIG) {
            if (global_data_A36772.watchdog_set_mode == WATCHDOG_MODE_0) {
                if ((global_data_A36772.input_dac_monitor.filtered_adc_reading > MIN_WD_VALUE_0) &&
                        (global_data_A36772.input_dac_monitor.filtered_adc_reading < MAX_WD_VALUE_0)) {
                    global_data_A36772.watchdog_counter = 0;
                    global_data_A36772.watchdog_state_change = 1;
                    global_data_A36772.watchdog_set_mode = WATCHDOG_MODE_1;
                    global_data_A36772.dac_digital_watchdog_oscillator = WATCHDOG_VALUE_1;
                    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_H, global_data_A36772.dac_digital_watchdog_oscillator);
                } else {
                    global_data_A36772.watchdog_counter++;
                    global_data_A36772.dac_digital_watchdog_oscillator = WATCHDOG_VALUE_0;
                }
            } else if (global_data_A36772.watchdog_set_mode == WATCHDOG_MODE_1) {
                if ((global_data_A36772.input_dac_monitor.filtered_adc_reading > MIN_WD_VALUE_1) &&
                        (global_data_A36772.input_dac_monitor.filtered_adc_reading < MAX_WD_VALUE_1)) {
                    global_data_A36772.watchdog_counter = 0;
                    global_data_A36772.watchdog_state_change = 1;
                    global_data_A36772.watchdog_set_mode = WATCHDOG_MODE_0;
                    global_data_A36772.dac_digital_watchdog_oscillator = WATCHDOG_VALUE_0;
                    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_H, global_data_A36772.dac_digital_watchdog_oscillator);
                } else {
                    global_data_A36772.watchdog_counter++;
                    global_data_A36772.dac_digital_watchdog_oscillator = WATCHDOG_VALUE_1;
                }
            } else {
                global_data_A36772.watchdog_set_mode = WATCHDOG_MODE_0;
            }
        }


        if (dac_resets_debug < global_data_A36772.watchdog_counter) {
            dac_resets_debug++;
        }


        if (global_data_A36772.reset_debug) {
            dac_resets_debug = 0;
        }

        //    if (global_data_A36772.watchdog_counter >= WATCHDOG_PERIOD) {
        //      global_data_A36772.watchdog_counter = 0;
        //      global_data_A36772.watchdog_fault = 0;
        //      global_data_A36772.watchdog_fault_count = 0;
        //      if (global_data_A36772.watchdog_set_mode == WATCHDOG_MODE_0){
        //        global_data_A36772.watchdog_set_mode = WATCHDOG_MODE_1;
        //      } else {
        //        global_data_A36772.watchdog_set_mode = WATCHDOG_MODE_0;
        //      }
        //    }
        //    
        //    if (global_data_A36772.watchdog_set_mode == WATCHDOG_MODE_0){
        //      global_data_A36772.dac_digital_watchdog_oscillator = WATCHDOG_VALUE_0;
        //    } else {
        //      global_data_A36772.dac_digital_watchdog_oscillator = WATCHDOG_VALUE_1;
        //    }
        //    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_H, global_data_A36772.dac_digital_watchdog_oscillator);    

        /*     if (global_data_A36772.watchdog_counter >= 3) {
              global_data_A36772.watchdog_counter = 0;
              if (global_data_A36772.dac_digital_watchdog_oscillator < ((WATCHDOG_HIGH >> 1) + (WATCHDOG_LOW >> 1))) {
            global_data_A36772.dac_digital_watchdog_oscillator = WATCHDOG_HIGH;
              } else {
            global_data_A36772.dac_digital_watchdog_oscillator = WATCHDOG_LOW;
              }
            }
            DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_H, global_data_A36772.dac_digital_watchdog_oscillator); */

        // Scale and Calibrate the internal ADC Readings
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.pot_htr);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.pot_vtop);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.pot_ek);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.ref_htr);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.ref_vtop);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.ref_ek);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.pos_15v_mon);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.neg_15v_mon);

        //ETMCanSlaveSetDebugRegister(0xA, global_data_A36772.pot_htr.reading_scaled_and_calibrated);
        //ETMCanSlaveSetDebugRegister(0xB, global_data_A36772.pot_vtop.reading_scaled_and_calibrated);
        //ETMCanSlaveSetDebugRegister(0xC, global_data_A36772.pot_ek.reading_scaled_and_calibrated);
        //ETMCanSlaveSetDebugRegister(0xD, global_data_A36772.ref_htr.reading_scaled_and_calibrated);
        //ETMCanSlaveSetDebugRegister(0xE, global_data_A36772.ref_vtop.reading_scaled_and_calibrated);
        //ETMCanSlaveSetDebugRegister(0xF, global_data_A36772.ref_ek.reading_scaled_and_calibrated);

        ETMCanSlaveSetDebugRegister(0xA, global_data_A36772.analog_output_heater_voltage.enabled); //global_data_A36772.run_time_counter);
        ETMCanSlaveSetDebugRegister(0xB, _FAULT_ADC_TOP_V_MON_UNDER_RELATIVE); //global_data_A36772.warmup_complete);//global_data_A36772.fault_restart_remaining);
        ETMCanSlaveSetDebugRegister(0xC, _FAULT_ADC_TOP_V_MON_OVER_RELATIVE); //_FAULT_ADC_BIAS_V_MON_UNDER_ABSOLUTE);
        ETMCanSlaveSetDebugRegister(0xD, ETMCanSlaveGetSyncMsgGunDriverDisableHeater()); //_FPGA_HEATER_VOLTAGE_LESS_THAN_4_5_VOLTS);
        ETMCanSlaveSetDebugRegister(0xE, dac_resets_debug);
        ETMCanSlaveSetDebugRegister(0xF, global_data_A36772.control_state);




		//Calculates the Resistance of the Heater Filament
        global_data_A36772.heater_voltage_double = ((double) global_data_A36772.input_htr_v_mon.reading_scaled_and_calibrated);			//Converts heater voltage and current into type double for division.
        global_data_A36772.heater_current_double = ((double) global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated);
        global_data_A36772.filament_resistance = global_data_A36772.heater_voltage_double / global_data_A36772.heater_current_double;	//Outputs filament resistance.
        global_data_A36772.scaled_filament_resistance = (unsigned int) (global_data_A36772.filament_resistance * 1000); 				//Converts back to unsigned integer type and scales for significant figures (i.e. 1.456 -> 1456).
        global_data_A36772.scaled_filament_resistance_for_display = (unsigned int) (global_data_A36772.filament_resistance * 10); 		//Converts back to unsigned integer type and scales for GUI feedback.
		
		
		
        
        slave_board_data.log_data[0] = global_data_A36772.scaled_filament_resistance;//global_data_A36772.input_gun_i_peak.reading_scaled_and_calibrated;
        slave_board_data.log_data[1] = global_data_A36772.input_hv_v_mon.reading_scaled_and_calibrated;
        slave_board_data.log_data[2] = global_data_A36772.input_top_v_mon.reading_scaled_and_calibrated; //gdoc says low energy
        slave_board_data.log_data[3] = global_data_A36772.input_top_v_mon.reading_scaled_and_calibrated; //gdoc says high energy
        slave_board_data.log_data[4] = global_data_A36772.input_temperature_mon.reading_scaled_and_calibrated;
        slave_board_data.log_data[5] = global_data_A36772.initial_ramp_timer;
        slave_board_data.log_data[6] = global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated;
        slave_board_data.log_data[7] = global_data_A36772.input_htr_v_mon.reading_scaled_and_calibrated;
        slave_board_data.log_data[8] = global_data_A36772.analog_output_high_voltage.set_point;
        slave_board_data.log_data[9] = global_data_A36772.heater_current_target;
        slave_board_data.log_data[10] = global_data_A36772.analog_output_top_voltage.set_point; //gdoc says low energy
        slave_board_data.log_data[11] = global_data_A36772.analog_output_top_voltage.set_point; //gdoc says high energy
        slave_board_data.log_data[12] = global_data_A36772.input_bias_v_mon.reading_scaled_and_calibrated;
        slave_board_data.log_data[13] = global_data_A36772.control_state;
        slave_board_data.log_data[14] = global_data_A36772.adc_read_error_count;
        slave_board_data.log_data[15] = GUN_DRIVER_LOAD_TYPE;
        //    slave_board_data.log_data[15] = //FPGA ASDR 16bit reg


        ETMCanSlaveSetDebugRegister(7, global_data_A36772.dac_write_failure_count);

#ifdef __POT_REFERENCE
        // The set points should be based on the pots
        ETMAnalogSetOutput(&global_data_A36772.analog_output_high_voltage, global_data_A36772.pot_ek.reading_scaled_and_calibrated);
        ETMAnalogSetOutput(&global_data_A36772.analog_output_top_voltage, global_data_A36772.pot_vtop.reading_scaled_and_calibrated);
        global_data_A36772.heater_voltage_target = global_data_A36772.pot_htr.reading_scaled_and_calibrated;

#endif

#ifdef __DISCRETE_REFERENCE
        // The set points should be based on the analog references
        ETMAnalogSetOutput(&global_data_A36772.analog_output_high_voltage, global_data_A36772.ref_ek.reading_scaled_and_calibrated);
        ETMAnalogSetOutput(&global_data_A36772.analog_output_top_voltage, global_data_A36772.ref_vtop.reading_scaled_and_calibrated);
        global_data_A36772.heater_voltage_target = global_data_A36772.ref_htr.reading_scaled_and_calibrated;

#endif

#ifdef __CAN_REFERENCE

        ETMAnalogSetOutput(&global_data_A36772.analog_output_high_voltage, global_data_A36772.can_high_voltage_set_point);
        global_data_A36772.heater_current_target = global_data_A36772.can_heater_current_set_point;
        if (global_data_A36772.high_energy_pulse) {
            ETMAnalogSetOutput(&global_data_A36772.analog_output_top_voltage, global_data_A36772.can_pulse_top_high_set_point);
        } else {
            ETMAnalogSetOutput(&global_data_A36772.analog_output_top_voltage, global_data_A36772.can_pulse_top_low_set_point);
        }

#endif

        if (global_data_A36772.heater_current_target > MAX_PROGRAM_HTR_CURRENT) {
            global_data_A36772.heater_current_target = MAX_PROGRAM_HTR_CURRENT;
        }

        //delay for 15 seconds once heater current is at target before count down
        if (global_data_A36772.resistance_warmup_delay <= 1500){
            if(global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated >= global_data_A36772.heater_current_target){
                global_data_A36772.resistance_warmup_delay++;
            }
        }
        
		//Current Limited
        if(global_data_A36772.resistance_warmup_delay <= 1500){
        // Ramp the heater voltage
        global_data_A36772.heater_ramp_interval++;
        if (!global_data_A36772.heater_operational) {
            if (global_data_A36772.heater_ramp_interval >= HEATER_RAMP_UP_TIME_PERIOD_SHORT) {
                global_data_A36772.heater_ramp_interval = 0;
                if (global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated < global_data_A36772.heater_current_target) {
                    global_data_A36772.analog_output_heater_voltage.set_point += HEATER_RAMP_UP_INCREMENT;
                } else{
                    global_data_A36772.analog_output_heater_voltage.set_point -= HEATER_FINE_VOLT_INCREMENT;
                }
            }
        } else {
            if (global_data_A36772.heater_ramp_interval >= HEATER_RAMP_UP_TIME_PERIOD_SHORT) {
                global_data_A36772.heater_ramp_interval = 0;
                if (global_data_A36772.scaled_filament_resistance < global_data_A36772.filament_resistance_limit) {
                    global_data_A36772.analog_output_heater_voltage.set_point += HEATER_FINE_VOLT_INCREMENT;
                } else if (global_data_A36772.scaled_filament_resistance > global_data_A36772.filament_resistance_limit) {
                    global_data_A36772.analog_output_heater_voltage.set_point -= HEATER_FINE_VOLT_INCREMENT;
                }
            }
        }
		//Resistance Limited & Current Limited
        }else{
        global_data_A36772.heater_ramp_interval++;
        if (!global_data_A36772.heater_operational) {
            if (global_data_A36772.heater_ramp_interval >= HEATER_RAMP_UP_TIME_PERIOD) {
                global_data_A36772.heater_ramp_interval = 0;
                if (global_data_A36772.scaled_filament_resistance < global_data_A36772.filament_resistance_limit && global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated < global_data_A36772.heater_current_target) {
                    global_data_A36772.analog_output_heater_voltage.set_point += HEATER_RAMP_UP_INCREMENT;
                } else{
                    global_data_A36772.set_current_reached = 1;	//warmup starts counting down
                    global_data_A36772.analog_output_heater_voltage.set_point -= HEATER_FINE_VOLT_INCREMENT;
                }
            }
		//Resitance Limited Only
        } else {
            if (global_data_A36772.heater_ramp_interval >= HEATER_RAMP_UP_TIME_PERIOD_LONG) {
                global_data_A36772.heater_ramp_interval = 0;
                if (global_data_A36772.scaled_filament_resistance < global_data_A36772.filament_resistance_limit){
                    if(global_data_A36772.scaled_filament_resistance < (global_data_A36772.filament_resistance_limit -(global_data_A36772.filament_resistance_limit*0.006))){ //If resistance is not within 0.6% of limit use fine increment
                    global_data_A36772.analog_output_heater_voltage.set_point += HEATER_FINE_VOLT_INCREMENT;
                    }else{																							//Else resistance is within 0.6% use extra fine increment
                    global_data_A36772.analog_output_heater_voltage.set_point += HEATER_XTRAFINE_VOLT_INCREMENT;
                    }
                } else if (global_data_A36772.scaled_filament_resistance > global_data_A36772.filament_resistance_limit){
                    if(global_data_A36772.scaled_filament_resistance > (global_data_A36772.filament_resistance_limit +(global_data_A36772.filament_resistance_limit*0.006))){ //If resistance is not within 0.6% of limit use fine increment
                    global_data_A36772.analog_output_heater_voltage.set_point -= HEATER_FINE_VOLT_INCREMENT;
                    }else{																							//Else resistance is within 0.6% use extra fine increment
                    global_data_A36772.analog_output_heater_voltage.set_point -= HEATER_XTRAFINE_VOLT_INCREMENT;
                    }
                }
            }
        }
        }
        /* 	if (global_data_A36772.analog_output_heater_voltage.set_point > global_data_A36772.heater_voltage_target) {
              global_data_A36772.analog_output_heater_voltage.set_point = global_data_A36772.heater_voltage_target;
            }*/

        // update the DAC programs based on the new set points.
        ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.analog_output_high_voltage);
        ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.analog_output_top_voltage);
        ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.analog_output_heater_voltage);

        ETMAnalogSetOutput(&global_data_A36772.monitor_heater_voltage, global_data_A36772.input_htr_v_mon.reading_scaled_and_calibrated);
        ETMAnalogSetOutput(&global_data_A36772.monitor_heater_current, global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated);
        ETMAnalogSetOutput(&global_data_A36772.monitor_cathode_voltage, global_data_A36772.input_hv_v_mon.reading_scaled_and_calibrated);
        ETMAnalogSetOutput(&global_data_A36772.monitor_grid_voltage, global_data_A36772.input_top_v_mon.reading_scaled_and_calibrated);

        ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.monitor_heater_voltage);
        ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.monitor_heater_current);
        ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.monitor_cathode_voltage);
        ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.monitor_grid_voltage);

        // Send out Data to local DAC and offboard.  Each channel will be updated once every 80mS
        // Do not send out while in state "STATE_WAIT_FOR_CONFIG" because the module is not ready to recieve data and
        // you will just get data transfer errors

        // Commented out local DAC communication because monitor signals are not used on the A36772-250Z board

        if (global_data_A36772.control_state != STATE_WAIT_FOR_CONFIG) {
            //Update top voltage every 10ms for quicker energy change transition   
            DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_B, global_data_A36772.analog_output_top_voltage.dac_setting_scaled_and_calibrated);

            switch ((global_data_A36772.run_time_counter & 0b111)) {

                case 0:
                    //	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36772.monitor_heater_voltage.dac_setting_scaled_and_calibrated);
                    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36772.analog_output_high_voltage.dac_setting_scaled_and_calibrated);

                    ETMCanSlaveSetDebugRegister(0, global_data_A36772.analog_output_high_voltage.dac_setting_scaled_and_calibrated);
                    break;


                case 1:
                    //      WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_B, global_data_A36772.monitor_heater_current.dac_setting_scaled_and_calibrated);
                    //      DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_B, global_data_A36772.analog_output_top_voltage.dac_setting_scaled_and_calibrated);

                    ETMCanSlaveSetDebugRegister(1, global_data_A36772.analog_output_top_voltage.dac_setting_scaled_and_calibrated);
                    break;


                case 2:
                    //	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36772.monitor_cathode_voltage.dac_setting_scaled_and_calibrated);
                    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36772.analog_output_heater_voltage.dac_setting_scaled_and_calibrated);

                    ETMCanSlaveSetDebugRegister(2, global_data_A36772.analog_output_heater_voltage.dac_setting_scaled_and_calibrated);
                    break;


                case 3:
                    //	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36772.monitor_grid_voltage.dac_setting_scaled_and_calibrated);
                    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36772.dac_digital_hv_enable);

                    ETMCanSlaveSetDebugRegister(3, global_data_A36772.dac_digital_hv_enable);
                    break;


                case 4:
                    //	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36772.monitor_heater_voltage.dac_setting_scaled_and_calibrated);
                    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_E, global_data_A36772.dac_digital_heater_enable);

                    ETMCanSlaveSetDebugRegister(4, global_data_A36772.dac_digital_heater_enable);
                    break;


                case 5:
                    //	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_B, global_data_A36772.monitor_heater_current.dac_setting_scaled_and_calibrated);
                    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_F, global_data_A36772.dac_digital_top_enable);

                    ETMCanSlaveSetDebugRegister(5, global_data_A36772.dac_digital_top_enable);
                    break;


                case 6:
                    //	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36772.monitor_cathode_voltage.dac_setting_scaled_and_calibrated);
                    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_G, global_data_A36772.dac_digital_trigger_enable);

                    ETMCanSlaveSetDebugRegister(6, global_data_A36772.dac_digital_trigger_enable);
                    break;


                case 7:
                    //	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36772.monitor_grid_voltage.dac_setting_scaled_and_calibrated);
                    if (global_data_A36772.watchdog_state_change == 0) {
                        DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_H, global_data_A36772.dac_digital_watchdog_oscillator);
                    } else {
                        global_data_A36772.watchdog_state_change = 0;
                    }
                    break;
            }
        }

        // Check SPI Communication    
        //    WatchdogCheck();

        // Update Faults
        UpdateFaults();

        // Mange LED and Status Outputs
        UpdateLEDandStatusOutuputs();
    }
}

void UpdateFaults(void) {


    //  if ((global_data_A36772.control_state == STATE_FAULT_HEATER_FAILURE) ||
    //       (global_data_A36772.control_state == STATE_FAULT_WARMUP_HEATER_OFF) ||
    //       (global_data_A36772.control_state == STATE_FAULT_HEATER_OFF) ||
    //       (global_data_A36772.control_state == STATE_FAULT_HEATER_ON) ||
    //       (global_data_A36772.control_state == STATE_WAIT_FOR_CONFIG)) {
    //    // Do not evalute any more fault conditions
    //    return;
    //  }

    //  if (global_data_A36772.control_state < STATE_HEATER_RAMP_UP) {
    //    // Do not evalute any more fault conditions
    //    return;
    //  }
    if (global_data_A36772.control_state == STATE_WAIT_FOR_CONFIG) {
        // Do not evalute any more fault conditions
        return;
    }

    if (global_data_A36772.fpga_firmware_major_rev_mismatch.filtered_reading) {
        _FAULT_FPGA_FIRMWARE_MAJOR_REV_MISMATCH = 1;
    } else if (global_data_A36772.reset_active) {
        _FAULT_FPGA_FIRMWARE_MAJOR_REV_MISMATCH = 0;
    }

    // Evaluate the readings from the Coverter Logic Board ADC
    if (global_data_A36772.adc_read_ok) {
        // There was a valid read of the data from the converter logic board

        // ------------------- Evaluate the digital readings from the Coverter Logic Board ADC ---------------------//  

        if (global_data_A36772.adc_digital_warmup_flt.filtered_reading == 0) {
            _STATUS_ADC_DIGITAL_HEATER_NOT_READY = 1;
        } else if (global_data_A36772.reset_active) {
            _STATUS_ADC_DIGITAL_HEATER_NOT_READY = 0;
        }

        if (global_data_A36772.control_state >= STATE_POWER_SUPPLY_RAMP_UP) {
            if (global_data_A36772.adc_digital_arc_flt.filtered_reading == 0) {
                _FAULT_ADC_DIGITAL_ARC = 1;
            }
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_DIGITAL_ARC = 0;
        }

        if (global_data_A36772.adc_digital_over_temp_flt.filtered_reading == 0) {
            _FAULT_ADC_DIGITAL_OVER_TEMP = 1;
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_DIGITAL_OVER_TEMP = 0;
        }

        if (global_data_A36772.adc_digital_grid_flt.filtered_reading == 0) {
            _FAULT_ADC_DIGITAL_GRID = 1;
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_DIGITAL_GRID = 0;
        }

        // ------------------- Evaluate the analog readings from the Coverter Logic Board ADC ---------------------//
        global_data_A36772.input_htr_v_mon.target_value = global_data_A36772.analog_output_heater_voltage.set_point;
        global_data_A36772.input_hv_v_mon.target_value = global_data_A36772.analog_output_high_voltage.set_point;
        global_data_A36772.input_top_v_mon.target_value = global_data_A36772.analog_output_top_voltage.set_point;

        /*     // If the set point is less that 1.5 V clear the under current counter
            if (global_data_A36772.analog_output_heater_voltage.set_point < 1500) {
              global_data_A36772.input_htr_v_mon.absolute_under_counter = 0;
            }
         */

        if (ETMAnalogCheckOverAbsolute(&global_data_A36772.input_htr_i_mon)) {
            _FAULT_ADC_HTR_I_MON_OVER_ABSOLUTE = 1;
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_HTR_I_MON_OVER_ABSOLUTE = 0;
        }

        if (ETMAnalogCheckOverAbsolute(&global_data_A36772.input_htr_v_mon)) {
            _FAULT_ADC_HTR_V_MON_OVER_ABSOLUTE = 1;
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_HTR_V_MON_OVER_ABSOLUTE = 0;
        }

        if (global_data_A36772.control_state >= STATE_HEATER_WARM_UP_DONE) {
            if (ETMAnalogCheckUnderAbsolute(&global_data_A36772.input_htr_v_mon)) {
                _FAULT_ADC_HTR_V_MON_UNDER_ABSOLUTE = 1;
            }
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_HTR_V_MON_UNDER_ABSOLUTE = 0;
        }

        if (global_data_A36772.control_state >= STATE_POWER_SUPPLY_RAMP_UP) {
            if (ETMAnalogCheckOverRelative(&global_data_A36772.input_hv_v_mon)) {
                _FAULT_ADC_HV_V_MON_OVER_RELATIVE = 1;
            }
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_HV_V_MON_OVER_RELATIVE = 0;
        }

        // Only check for HV undervoltage after HV is enabled
        if (global_data_A36772.control_state >= STATE_HV_ON) {
            if (ETMAnalogCheckUnderRelative(&global_data_A36772.input_hv_v_mon)) {
                _FAULT_ADC_HV_V_MON_UNDER_RELATIVE = 1;
            }
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_HV_V_MON_UNDER_RELATIVE = 0;
        }


        // Only check for top supply overvoltage after top is enabled
        if (global_data_A36772.control_state >= STATE_TOP_READY) {
            if (ETMAnalogCheckOverRelative(&global_data_A36772.input_top_v_mon)) {
                _FAULT_ADC_TOP_V_MON_OVER_RELATIVE = 1;
            }
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_TOP_V_MON_OVER_RELATIVE = 0;
        }

        // Only check for top supply undervoltage after top is enabled
        if (global_data_A36772.control_state >= STATE_TOP_READY) {
            if (ETMAnalogCheckUnderRelative(&global_data_A36772.input_top_v_mon)) {
                _FAULT_ADC_TOP_V_MON_UNDER_RELATIVE = 1;
            }
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_TOP_V_MON_UNDER_RELATIVE = 0;
        }

        if (ETMAnalogCheckOverAbsolute(&global_data_A36772.input_bias_v_mon)) {
            _FAULT_ADC_BIAS_V_MON_OVER_ABSOLUTE = 1;
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_BIAS_V_MON_OVER_ABSOLUTE = 0;
        }

        if (ETMAnalogCheckUnderAbsolute(&global_data_A36772.input_bias_v_mon)) {
            _FAULT_ADC_BIAS_V_MON_UNDER_ABSOLUTE = 1;
        } else if (global_data_A36772.reset_active) {
            _FAULT_ADC_BIAS_V_MON_UNDER_ABSOLUTE = 0;
        }

        if (global_data_A36772.watchdog_counter >= WATCHDOG_MAX_COUNT) { //latched Watchdog fault
            _STATUS_SPI_COM_FAULTED = 1;
            global_data_A36772.watchdog_counter = 0;
            ResetFPGA();
        } else if (global_data_A36772.reset_active) {
            _STATUS_SPI_COM_FAULTED = 0;
        }

        if (_STATUS_SPI_COM_FAULTED) {
            _FAULT_SPI_COMMUNICATION = 1;
        } else if (global_data_A36772.reset_active) {
            _FAULT_SPI_COMMUNICATION = 0;
        }

    }

    if (global_data_A36772.adc_read_error_test > MAX_CONVERTER_LOGIC_ADC_READ_ERRORS) {
        global_data_A36772.adc_read_error_test = MAX_CONVERTER_LOGIC_ADC_READ_ERRORS;
        _FAULT_CONVERTER_LOGIC_ADC_READ_FAILURE = 1;
    } else if ((global_data_A36772.adc_read_error_test < MAX_CONVERTER_LOGIC_ADC_READ_ERRORS) &&
            (global_data_A36772.reset_active != 0)) {
        _FAULT_CONVERTER_LOGIC_ADC_READ_FAILURE = 0;
    }


    //  if (global_data_A36772.watchdog_counter >= WATCHDOG_MAX_COUNT) {                 //latched Watchdog fault
    //    _FAULT_SPI_COMMUNICATION = 1;
    //  } else if (global_data_A36772.reset_active) {
    //    _FAULT_SPI_COMMUNICATION = 0;
    //  }  

}


//void WatchdogCheck(void) {
//
//  unsigned int test = 0;
//  
//  if (global_data_A36772.watchdog_counter == WATCHDOG_TEST_TIME_1) {
//    test = 1;
//  }
//  if (global_data_A36772.watchdog_counter == WATCHDOG_TEST_TIME_2) {
//    test = 2;
//  }
//  if (global_data_A36772.watchdog_counter == WATCHDOG_TEST_TIME_3) {
//    test = 3;
//  } 
//  
//  if (global_data_A36772.watchdog_set_mode == WATCHDOG_MODE_0) {
//    if (test == 1 || test == 2 || test == 3) {
//      if (global_data_A36772.input_dac_monitor.filtered_adc_reading > MAX_WD_VALUE_0) {
//        global_data_A36772.watchdog_fault_count++;
//      } else {
//        if (global_data_A36772.watchdog_fault_count) {
//          global_data_A36772.watchdog_fault_count--;
//        }
//      }
//      if (test == 3) {
//        if (global_data_A36772.watchdog_fault_count) {
//          global_data_A36772.watchdog_fault = 1;
//        } else {
//          global_data_A36772.watchdog_fault = 0;
//        }
//      }
//    }
//  } else {
//    if (test == 1 || test == 2 || test == 3) {
//      if (global_data_A36772.input_dac_monitor.filtered_adc_reading < MIN_WD_VALUE_1) {
//        global_data_A36772.watchdog_fault_count++;
//      } else {
//        if (global_data_A36772.watchdog_fault_count) {
//          global_data_A36772.watchdog_fault_count--;
//        }
//      }
//      if (test == 3) {
//        if (global_data_A36772.watchdog_fault_count) {
//          global_data_A36772.watchdog_fault = 1;
//        } else {
//          global_data_A36772.watchdog_fault = 0;
//        }
//      }
//    }
//  }   
//}

void UpdateLEDandStatusOutuputs(void) {
    // Warmup status
    if ((global_data_A36772.control_state >= STATE_START_UP) && (global_data_A36772.control_state <= STATE_HEATER_WARM_UP)) {
        PIN_LED_WARMUP = OLL_LED_ON;
        PIN_CPU_WARMUP_STATUS = OLL_STATUS_ACTIVE;
    } else {
        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_CPU_WARMUP_STATUS = !OLL_STATUS_ACTIVE;
    }

    // Standby Status
    if (global_data_A36772.control_state == STATE_HEATER_WARM_UP_DONE) {
        PIN_LED_STANDBY = OLL_LED_ON;
        PIN_CPU_STANDBY_STATUS = OLL_STATUS_ACTIVE;
    } else {
        PIN_LED_STANDBY = !OLL_LED_ON;
        PIN_CPU_STANDBY_STATUS = !OLL_STATUS_ACTIVE;
    }

    // HV ON Status
    if (global_data_A36772.control_state == STATE_POWER_SUPPLY_RAMP_UP) {
        // FLASH THE HV ON LED
        if (global_data_A36772.run_time_counter & 0x0010) {
            PIN_LED_HV_ON = OLL_LED_ON;
            PIN_CPU_HV_ON_STATUS = OLL_STATUS_ACTIVE;
        } else {
            PIN_LED_HV_ON = !OLL_LED_ON;
            PIN_CPU_HV_ON_STATUS = !OLL_STATUS_ACTIVE;
        }
    } else if (global_data_A36772.control_state >= STATE_HV_ON) {
        PIN_LED_HV_ON = OLL_LED_ON;
        PIN_CPU_HV_ON_STATUS = OLL_STATUS_ACTIVE;
    } else {
        PIN_LED_HV_ON = !OLL_LED_ON;
        PIN_CPU_HV_ON_STATUS = !OLL_STATUS_ACTIVE;
    }

    // Beam enabled Status
    if (global_data_A36772.control_state == STATE_BEAM_ENABLE) {
        PIN_LED_BEAM_ENABLE = OLL_LED_ON;
        PIN_CPU_BEAM_ENABLE_STATUS = OLL_STATUS_ACTIVE;
    } else {
        PIN_LED_BEAM_ENABLE = !OLL_LED_ON;
        PIN_CPU_BEAM_ENABLE_STATUS = !OLL_STATUS_ACTIVE;
    }

    //  if (global_data_A36772.control_state == STATE_HV_ON) {
    //    PIN_LED_BEAM_ENABLE = OLL_LED_ON;
    //    PIN_CPU_BEAM_ENABLE_STATUS = OLL_STATUS_ACTIVE;
    //  } else {
    //    PIN_LED_BEAM_ENABLE = !OLL_LED_ON;
    //    PIN_CPU_BEAM_ENABLE_STATUS = !OLL_STATUS_ACTIVE;
    //  }

    // System OK Status
    if (global_data_A36772.control_state <= STATE_FAULT_HEATER_ON) {
        PIN_CPU_SYSTEM_OK_STATUS = !OLL_STATUS_ACTIVE;
        PIN_LED_SYSTEM_OK = !OLL_LED_ON;
    } else {
        PIN_CPU_SYSTEM_OK_STATUS = OLL_STATUS_ACTIVE;
        PIN_LED_SYSTEM_OK = OLL_LED_ON;
    }
}

void EnableHeater(void) {
    /* 
       Set the heater ref
       Set the heater enable control voltage
     */
    global_data_A36772.analog_output_heater_voltage.enabled = 1;
    global_data_A36772.dac_digital_heater_enable = DAC_DIGITAL_ON;
    //DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_E, global_data_A36772.dac_digital_heater_enable); // hkw -may need this
}

void DisableHeater(void) {
    /* 
       Set the heater ref to zero
       Clear the heater enable control voltage
     */
    global_data_A36772.analog_output_heater_voltage.enabled = 0;
    global_data_A36772.dac_digital_heater_enable = DAC_DIGITAL_OFF;
    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_E, global_data_A36772.dac_digital_heater_enable);
    _STATUS_HEATER_AT_OPERATING_CURRENT = 0;
}

void EnableHighVoltage(void) {
    /*
      Set the HVPS reference
      Set the grid top reference 
      Set the HVPS enable control voltage
      Set the grid top enable control voltage
     */
    global_data_A36772.analog_output_high_voltage.enabled = 1;
    global_data_A36772.dac_digital_hv_enable = DAC_DIGITAL_ON;
    PIN_CPU_HV_ENABLE = OLL_PIN_CPU_HV_ENABLE_HV_ENABLED;
}

void EnableTopSupply(void) {
    /*
       Set the grid top reference
       Set the grid top enable control voltage
     */

    global_data_A36772.analog_output_top_voltage.enabled = 1;
    global_data_A36772.dac_digital_top_enable = DAC_DIGITAL_ON;
}

void DisableHighVoltage(void) {
    /*
      Set the HVPS reference to zero
      Set the grid top reference to zero 
      Clear the HVPS enable control voltage
      Clear the grid top enable control voltage
     */
    global_data_A36772.analog_output_top_voltage.enabled = 0;
    global_data_A36772.analog_output_high_voltage.enabled = 0;
    global_data_A36772.dac_digital_top_enable = DAC_DIGITAL_OFF;
    global_data_A36772.dac_digital_hv_enable = DAC_DIGITAL_OFF;
    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_F, global_data_A36772.dac_digital_top_enable);
    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36772.dac_digital_hv_enable);
    PIN_CPU_HV_ENABLE = !OLL_PIN_CPU_HV_ENABLE_HV_ENABLED;
}

void EnableBeam(void) {
    global_data_A36772.dac_digital_trigger_enable = DAC_DIGITAL_ON;
    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_G, global_data_A36772.dac_digital_trigger_enable);
    PIN_CPU_BEAM_ENABLE = OLL_PIN_CPU_BEAM_ENABLE_BEAM_ENABLED;
}

void DisableBeam(void) {
    global_data_A36772.dac_digital_trigger_enable = DAC_DIGITAL_OFF;
    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_G, global_data_A36772.dac_digital_trigger_enable);
    PIN_CPU_BEAM_ENABLE = !OLL_PIN_CPU_BEAM_ENABLE_BEAM_ENABLED;
}

void ResetFPGA(void) {
    PIN_CS_FPGA = OLL_PIN_CS_FPGA_SELECTED;
    PIN_CS_DAC = OLL_PIN_CS_DAC_SELECTED;
    PIN_CS_ADC = OLL_PIN_CS_ADC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);

    PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
    PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
    PIN_CS_ADC = !OLL_PIN_CS_ADC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);
}

void ADCConfigure(void) {
    /*
      Configure for read of all channels + temperature with 8x (or 16x) Averaging
     */
    unsigned char temp;

    PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
    PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
    PIN_CS_ADC = OLL_PIN_CS_ADC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);

    temp = SPICharInvertered(MAX1230_RESET_BYTE);
    temp = SPICharInvertered(MAX1230_SETUP_BYTE);
    temp = SPICharInvertered(MAX1230_AVERAGE_BYTE);


    PIN_CS_ADC = !OLL_PIN_CS_ADC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);
}

void ADCStartAcquisition(void) {
    /* 
       Start the acquisition process
     */
    unsigned char temp;

    PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
    PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
    PIN_CS_ADC = OLL_PIN_CS_ADC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);

    temp = SPICharInvertered(MAX1230_CONVERSION_BYTE);

    PIN_CS_ADC = !OLL_PIN_CS_ADC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);

}

void UpdateADCResults(void) {
    unsigned int n;
    unsigned int read_error;
    unsigned int read_data[17];

    /*
      Read all the results of the 16 Channels + temp sensor
      16 bits per channel
      17 channels
      272 bit message
      Approx 400us (counting processor overhead)
     */

    // Select the ADC
    PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
    PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
    PIN_CS_ADC = OLL_PIN_CS_ADC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);

    for (n = 0; n < 17; n++) {
        read_data[n] = SPICharInvertered(0);
        read_data[n] <<= 8;
        read_data[n] += SPICharInvertered(0);
    }


    PIN_CS_ADC = !OLL_PIN_CS_ADC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);



    // ERROR CHECKING ON RETURNED DATA.  IF THERE APPEARS TO BE A BIT ERROR, DO NOT LOAD THE DATA

    read_error = 0;
    read_error |= read_data[0];
    read_error |= read_data[1];
    read_error |= read_data[2];
    read_error |= read_data[3];
    read_error |= read_data[4];
    read_error |= read_data[5];
    read_error |= read_data[6];
    read_error |= read_data[7];
    read_error |= read_data[8];
    read_error |= read_data[9];
    read_error |= read_data[10];
    read_error |= read_data[11];
    read_error |= read_data[12];
    read_error |= read_data[13];
    read_error |= read_data[14];
    read_error |= read_data[15];
    read_error |= read_data[16];
    read_error &= 0xF000;

    if (read_data[8] < 0x0200) {
        // The 24V supply is less than the minimum needed to operate
        read_error = 1;
    }

    if (read_error) {
        // There clearly is a data error
        global_data_A36772.adc_read_error_count++;
        global_data_A36772.adc_read_error_test++;
        global_data_A36772.adc_read_ok = 0;
        ADCConfigure();
    } else {
        // The data passed the most basic test.  Load the values into RAM
        global_data_A36772.adc_read_ok = 1;
        if (global_data_A36772.adc_read_error_test) {
            global_data_A36772.adc_read_error_test--;
        }

        global_data_A36772.input_adc_temperature.filtered_adc_reading = read_data[0];
        global_data_A36772.input_hv_v_mon.filtered_adc_reading = read_data[1] << 4;
        global_data_A36772.input_hv_i_mon.filtered_adc_reading = read_data[2] << 4;
        global_data_A36772.input_gun_i_peak.filtered_adc_reading = read_data[3] << 4;
        global_data_A36772.input_htr_v_mon.filtered_adc_reading = read_data[4] << 4;
        global_data_A36772.input_htr_i_mon.filtered_adc_reading = read_data[5] << 4;
        global_data_A36772.input_top_v_mon.filtered_adc_reading = read_data[6] << 4;
        global_data_A36772.input_bias_v_mon.filtered_adc_reading = read_data[7] << 4;
        global_data_A36772.input_24_v_mon.filtered_adc_reading = read_data[8] << 4;
        global_data_A36772.input_temperature_mon.filtered_adc_reading = read_data[9] << 4;
        global_data_A36772.input_dac_monitor.filtered_adc_reading = read_data[16] << 4;

        if (read_data[10] > ADC_DATA_DIGITAL_HIGH) {
            ETMDigitalUpdateInput(&global_data_A36772.adc_digital_warmup_flt, 1);
        } else {
            ETMDigitalUpdateInput(&global_data_A36772.adc_digital_warmup_flt, 0);
        }

        //    if (read_data[11] > ADC_DATA_DIGITAL_HIGH) {
        //      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_watchdog_flt, 1);
        //    } else {
        //      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_watchdog_flt, 0);
        //    }

        if (read_data[12] > ADC_DATA_DIGITAL_HIGH) {
            ETMDigitalUpdateInput(&global_data_A36772.adc_digital_arc_flt, 1);
        } else {
            ETMDigitalUpdateInput(&global_data_A36772.adc_digital_arc_flt, 0);
        }

        if (read_data[13] > ADC_DATA_DIGITAL_HIGH) {
            ETMDigitalUpdateInput(&global_data_A36772.adc_digital_over_temp_flt, 1);
        } else {
            ETMDigitalUpdateInput(&global_data_A36772.adc_digital_over_temp_flt, 0);
        }

        //    if (read_data[14] > ADC_DATA_DIGITAL_HIGH) {
        //      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_pulse_width_duty_flt, 1);
        //    } else {
        //      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_pulse_width_duty_flt, 0);
        //    }

        if (read_data[15] > ADC_DATA_DIGITAL_HIGH) {
            ETMDigitalUpdateInput(&global_data_A36772.adc_digital_grid_flt, 1);
        } else {
            ETMDigitalUpdateInput(&global_data_A36772.adc_digital_grid_flt, 0);
        }

        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_adc_temperature);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_hv_v_mon);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_hv_i_mon);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_gun_i_peak);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_htr_v_mon);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_htr_i_mon);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_top_v_mon);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_bias_v_mon);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_24_v_mon);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_temperature_mon);
        ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_dac_monitor);
    }
}

void DACWriteChannel(unsigned int command_word, unsigned int data_word) {
    unsigned int command_word_check;
    unsigned int data_word_check;
    unsigned int transmission_complete;
    unsigned int loop_counter;
    unsigned int spi_char;

    transmission_complete = 0;
    loop_counter = 0;
    while (transmission_complete == 0) {
        loop_counter++;

        // -------------- Send Out the Data ---------------------//

        // Select the DAC
        PIN_CS_ADC = !OLL_PIN_CS_ADC_SELECTED;
        PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
        PIN_CS_DAC = OLL_PIN_CS_DAC_SELECTED;
        __delay32(DELAY_FPGA_CABLE_DELAY);

        spi_char = (command_word >> 8) & 0x00FF;
        command_word_check = SPICharInvertered(spi_char);
        command_word_check <<= 8;
        spi_char = command_word & 0x00FF;
        command_word_check += SPICharInvertered(spi_char);


        spi_char = (data_word >> 8) & 0x00FF;
        data_word_check = SPICharInvertered(spi_char);
        data_word_check <<= 8;
        spi_char = data_word & 0x00FF;
        data_word_check += SPICharInvertered(spi_char);

        PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
        __delay32(DELAY_FPGA_CABLE_DELAY);



        // ------------- Confirm the data was written correctly ------------------- //

        PIN_CS_DAC = OLL_PIN_CS_DAC_SELECTED;
        __delay32(DELAY_FPGA_CABLE_DELAY);

        spi_char = (LTC265X_CMD_NO_OPERATION >> 8) & 0x00FF;
        command_word_check = SPICharInvertered(spi_char);
        command_word_check <<= 8;
        spi_char = LTC265X_CMD_NO_OPERATION & 0x00FF;
        command_word_check += SPICharInvertered(spi_char);

        spi_char = 0;
        data_word_check = SPICharInvertered(spi_char);
        data_word_check <<= 8;
        spi_char = 0;
        data_word_check += SPICharInvertered(spi_char);


        PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
        __delay32(DELAY_FPGA_CABLE_DELAY);


        if ((command_word_check == command_word) && (data_word_check == data_word)) {
            transmission_complete = 1;
            global_data_A36772.dac_write_failure = 0;
        } else {
            global_data_A36772.dac_write_error_count++;
        }

        if ((transmission_complete == 0) && (loop_counter >= MAX_DAC_TX_ATTEMPTS)) {
            transmission_complete = 1;
            global_data_A36772.dac_write_failure_count++;
            global_data_A36772.dac_write_failure = 1;
            _STATUS_DAC_WRITE_FAILURE = 1;
        } else if (global_data_A36772.reset_active) {
            _STATUS_DAC_WRITE_FAILURE = 0;
            global_data_A36772.dac_write_failure = 0;
        }
    }
}

typedef struct {
    unsigned converter_logic_pcb_rev : 6;
    unsigned fpga_firmware_major_rev : 4;
    unsigned fpga_firmware_minor_rev : 6;
    unsigned arc : 1;
    unsigned arc_high_voltage_inihibit_active : 1;
    unsigned heater_voltage_less_than_4_5_volts : 1;
    unsigned module_temp_greater_than_65_C : 1;
    unsigned module_temp_greater_than_75_C : 1;
    unsigned pulse_width_limiting_active : 1;
    unsigned prf_fault : 1;
    unsigned current_monitor_pulse_width_fault : 1;
    unsigned grid_module_hardware_fault : 1;
    unsigned grid_module_over_voltage_fault : 1;
    unsigned grid_module_under_voltage_fault : 1;
    unsigned grid_module_bias_voltage_fault : 1;
    unsigned hv_regulation_warning : 1;
    unsigned dipswitch_1_on : 1;
    unsigned test_mode_toggle_switch_set_to_test : 1;
    unsigned local_mode_toggle_switch_set_to_local : 1;
} TYPE_FPGA_DATA;

void FPGAReadData(void) {
    unsigned long bits;
    TYPE_FPGA_DATA fpga_bits;
    /*
      Reads 32 bits from the FPGA
     */

    PIN_CS_ADC = !OLL_PIN_CS_ADC_SELECTED;
    PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
    PIN_CS_FPGA = OLL_PIN_CS_FPGA_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);

    bits = SPICharInvertered(0xFF);
    bits <<= 8;
    bits += SPICharInvertered(0xFF);
    bits <<= 8;
    bits += SPICharInvertered(0xFF);
    bits <<= 8;
    bits += SPICharInvertered(0xFF);


    PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);

    // error check the data and update digital inputs  
    fpga_bits = *(TYPE_FPGA_DATA*) & bits;

    // Check the firmware major rev (LATCHED)    
    if (fpga_bits.fpga_firmware_major_rev != TARGET_FPGA_FIRMWARE_MAJOR_REV) {
        ETMDigitalUpdateInput(&global_data_A36772.fpga_firmware_major_rev_mismatch, 1);
    } else {
        ETMDigitalUpdateInput(&global_data_A36772.fpga_firmware_major_rev_mismatch, 0);
    }

    // Only check the rest of the data bits if the Major Rev Matches
    if (fpga_bits.fpga_firmware_major_rev == TARGET_FPGA_FIRMWARE_MAJOR_REV) {

        // Check the logic board pcb rev (NOT LATCHED)
        if (fpga_bits.converter_logic_pcb_rev != TARGET_CONVERTER_LOGIC_PCB_REV) {
            ETMDigitalUpdateInput(&global_data_A36772.fpga_coverter_logic_pcb_rev_mismatch, 1);
        } else {
            ETMDigitalUpdateInput(&global_data_A36772.fpga_coverter_logic_pcb_rev_mismatch, 0);
        }
        if (global_data_A36772.fpga_coverter_logic_pcb_rev_mismatch.filtered_reading) {
            _FPGA_CONVERTER_LOGIC_PCB_REV_MISMATCH = 1;
        } else {
            _FPGA_CONVERTER_LOGIC_PCB_REV_MISMATCH = 0;
        }

        // Check the firmware minor rev (NOT LATCHED)
        if (fpga_bits.fpga_firmware_minor_rev != TARGET_FPGA_FIRMWARE_MINOR_REV) {
            ETMDigitalUpdateInput(&global_data_A36772.fpga_firmware_minor_rev_mismatch, 1);
        } else {
            ETMDigitalUpdateInput(&global_data_A36772.fpga_firmware_minor_rev_mismatch, 0);
        }
        if (global_data_A36772.fpga_firmware_minor_rev_mismatch.filtered_reading) {
            _FPGA_FIRMWARE_MINOR_REV_MISMATCH = 1;
        } else {
            _FPGA_FIRMWARE_MINOR_REV_MISMATCH = 0;
        }

        // Check the Arc Count (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_arc, fpga_bits.arc);
        if (global_data_A36772.fpga_arc.filtered_reading) {
            _FPGA_ARC_COUNTER_GREATER_ZERO = 1;
        } else {
            _FPGA_ARC_COUNTER_GREATER_ZERO = 0;
        }

        // Check Arc High Voltage Inhibit Active (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_arc_high_voltage_inihibit_active, fpga_bits.arc_high_voltage_inihibit_active);
        if (global_data_A36772.fpga_arc_high_voltage_inihibit_active.filtered_reading) {
            _FPGA_ARC_HIGH_VOLTAGE_INHIBIT_ACTIVE = 1;
        } else {
            _FPGA_ARC_HIGH_VOLTAGE_INHIBIT_ACTIVE = 0;
        }


        // Check module temp greater than 65 C (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_module_temp_greater_than_65_C, fpga_bits.module_temp_greater_than_65_C);
        if (global_data_A36772.fpga_module_temp_greater_than_65_C.filtered_reading) {
            _FPGA_MODULE_TEMP_GREATER_THAN_65_C = 1;
        } else {
            _FPGA_MODULE_TEMP_GREATER_THAN_65_C = 0;
        }

        // Check module temp greater than 75 C (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_module_temp_greater_than_75_C, fpga_bits.module_temp_greater_than_75_C);
        if (global_data_A36772.fpga_module_temp_greater_than_75_C.filtered_reading) {
            _FPGA_MODULE_TEMP_GREATER_THAN_75_C = 1;
        } else {
            _FPGA_MODULE_TEMP_GREATER_THAN_75_C = 0;
        }


        // Check Current Monitor Pulse Width Fault (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_current_monitor_pulse_width_fault, fpga_bits.current_monitor_pulse_width_fault);
        if (global_data_A36772.fpga_current_monitor_pulse_width_fault.filtered_reading) {
            _FPGA_CURRENT_MONITOR_PULSE_WIDTH_FAULT = 1;
        } else {
            _FPGA_CURRENT_MONITOR_PULSE_WIDTH_FAULT = 0;
        }

        // Check the heater voltage less than 4.5 Volts (LATCHED)
        //    ETMDigitalUpdateInput(&global_data_A36772.fpga_heater_voltage_less_than_4_5_volts, fpga_bits.heater_voltage_less_than_4_5_volts);
        //    if (global_data_A36772.fpga_heater_voltage_less_than_4_5_volts.filtered_reading) {
        //      _FPGA_HEATER_VOLTAGE_LESS_THAN_4_5_VOLTS = 1;
        //    } else if (global_data_A36772.reset_active) {
        //      _FPGA_HEATER_VOLTAGE_LESS_THAN_4_5_VOLTS = 0;
        //    }

        // Check grid module hardware fault (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_grid_module_hardware_fault, fpga_bits.grid_module_hardware_fault);
        if (global_data_A36772.fpga_grid_module_hardware_fault.filtered_reading) {
            _FPGA_GRID_MODULE_HARDWARE_FAULT = 1;
        } else {
            _FPGA_GRID_MODULE_HARDWARE_FAULT = 0;
        }

        // Check grid module over voltage (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_grid_module_over_voltage_fault, fpga_bits.grid_module_over_voltage_fault);
        if (global_data_A36772.fpga_grid_module_over_voltage_fault.filtered_reading) {
            _FPGA_GRID_MODULE_OVER_VOLTAGE_FAULT = 1;
        } else {
            _FPGA_GRID_MODULE_OVER_VOLTAGE_FAULT = 0;
        }

        // Check grid module under voltage (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_grid_module_under_voltage_fault, fpga_bits.grid_module_under_voltage_fault);
        if (global_data_A36772.fpga_grid_module_under_voltage_fault.filtered_reading) {
            _FPGA_GRID_MODULE_UNDER_VOLTAGE_FAULT = 1;
        } else {
            _FPGA_GRID_MODULE_UNDER_VOLTAGE_FAULT = 0;
        }

        // Check grid module bias voltage (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_grid_module_bias_voltage_fault, fpga_bits.grid_module_bias_voltage_fault);
        if (global_data_A36772.fpga_grid_module_bias_voltage_fault.filtered_reading) {
            _FPGA_GRID_MODULE_BIAS_VOLTAGE_FAULT = 1;
        } else {
            _FPGA_GRID_MODULE_BIAS_VOLTAGE_FAULT = 0;
        }

        // High Voltage regulation Warning (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_hv_regulation_warning, fpga_bits.hv_regulation_warning);
        if (global_data_A36772.fpga_hv_regulation_warning.filtered_reading) {
            _FPGA_HV_REGULATION_WARNING = 1;
        } else {
            _FPGA_HV_REGULATION_WARNING = 0;
        }


        // FPGA DIPSWITCH 1 ON (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_dipswitch_1_on, fpga_bits.dipswitch_1_on);
        if (&global_data_A36772.fpga_dipswitch_1_on.filtered_reading) {
            _FPGA_DIPSWITCH_1_ON = 1;
        } else {
            _FPGA_DIPSWITCH_1_ON = 0;
        }

        // Check test mode toggle switch (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_test_mode_toggle_switch_set_to_test, fpga_bits.test_mode_toggle_switch_set_to_test);
        if (&global_data_A36772.fpga_test_mode_toggle_switch_set_to_test.filtered_reading) {
            _FPGA_TEST_MODE_TOGGLE_SWITCH_TEST_MODE = 1;
        } else {
            _FPGA_TEST_MODE_TOGGLE_SWITCH_TEST_MODE = 0;
        }

        // Check local mode toggle switch (NOT LATCHED)
        ETMDigitalUpdateInput(&global_data_A36772.fpga_local_mode_toggle_switch_set_to_local, fpga_bits.local_mode_toggle_switch_set_to_local);
        if (global_data_A36772.fpga_local_mode_toggle_switch_set_to_local.filtered_reading) {
            _FPGA_LOCAL_MODE_TOGGLE_SWITCH_LOCAL_MODE = 1;
        } else {
            _FPGA_LOCAL_MODE_TOGGLE_SWITCH_LOCAL_MODE = 0;
        }

    }
}

unsigned char SPICharInvertered(unsigned char transmit_byte) {
    unsigned int transmit_word;
    unsigned int receive_word;
    transmit_word = ((~transmit_byte) & 0x00FF);
    receive_word = SendAndReceiveSPI(transmit_word, ETM_SPI_PORT_1);
    receive_word = ((~receive_word) & 0x00FF);
    return (receive_word & 0x00FF);
}

void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
    _ADIF = 0;

    // Copy Data From Buffer to RAM
    if (_BUFS) {
        // read ADCBUF 0-7
        global_data_A36772.pot_ek.adc_accumulator += ADCBUF0;
        global_data_A36772.pot_vtop.adc_accumulator += ADCBUF1;
        global_data_A36772.pot_htr.adc_accumulator += ADCBUF2;
        global_data_A36772.ref_htr.adc_accumulator += ADCBUF3;
        global_data_A36772.ref_vtop.adc_accumulator += ADCBUF4;
        global_data_A36772.ref_ek.adc_accumulator += ADCBUF5;
        global_data_A36772.pos_15v_mon.adc_accumulator += ADCBUF6;
        global_data_A36772.neg_15v_mon.adc_accumulator += ADCBUF7;
    } else {
        // read ADCBUF 8-15
        global_data_A36772.pot_ek.adc_accumulator += ADCBUF8;
        global_data_A36772.pot_vtop.adc_accumulator += ADCBUF9;
        global_data_A36772.pot_htr.adc_accumulator += ADCBUFA;
        global_data_A36772.ref_htr.adc_accumulator += ADCBUFB;
        global_data_A36772.ref_vtop.adc_accumulator += ADCBUFC;
        global_data_A36772.ref_ek.adc_accumulator += ADCBUFD;
        global_data_A36772.pos_15v_mon.adc_accumulator += ADCBUFE;
        global_data_A36772.neg_15v_mon.adc_accumulator += ADCBUFF;
    }

    global_data_A36772.accumulator_counter += 1;

    if (global_data_A36772.accumulator_counter >= 128) {
        global_data_A36772.accumulator_counter = 0;

        // average the 128 12 bit samples into a single 16 bit sample
        global_data_A36772.pot_htr.adc_accumulator >>= 3;
        global_data_A36772.pot_vtop.adc_accumulator >>= 3;
        global_data_A36772.pot_ek.adc_accumulator >>= 3;
        global_data_A36772.ref_htr.adc_accumulator >>= 3;
        global_data_A36772.ref_vtop.adc_accumulator >>= 3;
        global_data_A36772.ref_ek.adc_accumulator >>= 3;
        global_data_A36772.pos_15v_mon.adc_accumulator >>= 3;
        global_data_A36772.neg_15v_mon.adc_accumulator >>= 3;

        // Store the filtred results
        global_data_A36772.pot_htr.filtered_adc_reading = global_data_A36772.pot_htr.adc_accumulator;
        global_data_A36772.pot_vtop.filtered_adc_reading = global_data_A36772.pot_vtop.adc_accumulator;
        global_data_A36772.pot_ek.filtered_adc_reading = global_data_A36772.pot_ek.adc_accumulator;
        global_data_A36772.ref_htr.filtered_adc_reading = global_data_A36772.ref_htr.adc_accumulator;
        global_data_A36772.ref_vtop.filtered_adc_reading = global_data_A36772.ref_vtop.adc_accumulator;
        global_data_A36772.ref_ek.filtered_adc_reading = global_data_A36772.ref_ek.adc_accumulator;
        global_data_A36772.pos_15v_mon.filtered_adc_reading = global_data_A36772.pos_15v_mon.adc_accumulator;
        global_data_A36772.neg_15v_mon.filtered_adc_reading = global_data_A36772.neg_15v_mon.adc_accumulator;

        // clear the accumulators
        global_data_A36772.pot_htr.adc_accumulator = 0;
        global_data_A36772.pot_vtop.adc_accumulator = 0;
        global_data_A36772.pot_ek.adc_accumulator = 0;
        global_data_A36772.ref_htr.adc_accumulator = 0;
        global_data_A36772.ref_vtop.adc_accumulator = 0;
        global_data_A36772.ref_ek.adc_accumulator = 0;
        global_data_A36772.pos_15v_mon.adc_accumulator = 0;
        global_data_A36772.neg_15v_mon.adc_accumulator = 0;
    }
}

void ETMAnalogClearFaultCounters(AnalogInput* ptr_analog_input) {
    ptr_analog_input->absolute_under_counter = 0;
    ptr_analog_input->absolute_over_counter = 0;
    ptr_analog_input->over_trip_counter = 0;
    ptr_analog_input->under_trip_counter = 0;
}


//void ETMCanSpoofPulseSyncNextPulseLevel(void) {
//  ETMCanMessage message;
//  message.identifier = ETM_CAN_MSG_LVL_TX | (ETM_CAN_ADDR_PULSE_SYNC_BOARD << 3);
//  message.word0      = next_pulse_count;
//  message.word1    = 0xFFFF;
//  ETMCanTXMessage(&message, &C2TX2CON);
//}
//
//
//void ETMCanSpoofAFCHighSpeedDataLog(void) {
//  unsigned int packet_id;
//
//  // Spoof HV Lambda Packet 0x4C
//  ETMCanMessage log_message;
//
//  packet_id = 0x004C;
//  packet_id <<= 1;
//  packet_id |= 0b0000011000000000;
//  packet_id <<= 2;
//
//  log_message.identifier = packet_id;
//  log_message.identifier &= 0xFF00;
//  log_message.identifier <<= 3;
//  log_message.identifier |= (packet_id & 0x00FF);
//
//  log_message.word3 = next_pulse_count-1;
//  log_message.word2 = global_data_A36772.heater_voltage_target;
//  log_message.word1 = global_data_A36772.analog_output_heater_voltage.set_point;
//  log_message.word0 = global_data_A36772.pot_htr.reading_scaled_and_calibrated;
//
//  ETMCanAddMessageToBuffer(&etm_can_tx_message_buffer, &log_message);
//  MacroETMCanCheckTXBuffer();
//
//  // Spoof Pulse Sync Packet 0x3C
//
//  packet_id = 0x003C;
//  packet_id <<= 1;
//  packet_id |= 0b0000011000000000;
//  packet_id <<= 2;
//
//  log_message.identifier = packet_id;
//  log_message.identifier &= 0xFF00;
//  log_message.identifier <<= 3;
//  log_message.identifier |= (packet_id & 0x00FF);
//
//  log_message.word3 = next_pulse_count-1;//local_debug_data.debug_4;
//  log_message.word2 = global_data_A36772.control_state;
//  log_message.word1 = global_data_A36772.input_htr_v_mon.reading_scaled_and_calibrated;
//  log_message.word0 = global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated;
//
//  ETMCanAddMessageToBuffer(&etm_can_tx_message_buffer, &log_message);
//  MacroETMCanCheckTXBuffer();
//
//}

void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
    unsigned int index_word;

    index_word = message_ptr->word3;
    switch (index_word) {

        case ETM_CAN_REGISTER_GUN_DRIVER_SET_1_GRID_TOP_SET_POINT:
            global_data_A36772.can_pulse_top_high_set_point = message_ptr->word1;
            global_data_A36772.can_pulse_top_low_set_point = message_ptr->word0;
            global_data_A36772.control_config |= 1;
            if (global_data_A36772.control_config == 3) {
                _CONTROL_NOT_CONFIGURED = 0;
            }
            break;
            //
        case ETM_CAN_REGISTER_GUN_DRIVER_SET_1_HEATER_CATHODE_SET_POINT:
            global_data_A36772.can_high_voltage_set_point = message_ptr->word1;
            global_data_A36772.can_heater_current_set_point = message_ptr->word0;

            global_data_A36772.control_config |= 2;
            if (global_data_A36772.control_config == 3) {
                _CONTROL_NOT_CONFIGURED = 0;
            }
            break;

        case ETM_CAN_REGISTER_GUN_DRIVER_RESET_FPGA:
            if (global_data_A36772.control_state < STATE_POWER_SUPPLY_RAMP_UP) {
                ResetFPGA();
                global_data_A36772.control_state = STATE_WAIT_FOR_CONFIG;
            }

            break;

        default:
            //      local_can_errors.invalid_index++;
            break;

    }

}
