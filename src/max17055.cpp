/******************************************************************//**
* @file max17055.cpp
*
* @author Felipe Neira - Maxim Integrated - TTS
*
* @version 1.5
*
* Started: 13DEC17
*
* Updated: 16APR18.
* Remove unnecessary code.  
* Check spelling
* Error check
*
*
/*******************************************************************************
* Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#include "max17055.h"
 
/* POR Mask */
#define MAX17055_POR_MASK               (0xFFFD)
 
/* MODELCFG register bits */
#define MAX17055_MODELCFG_REFRESH       (1 << 15)
 
 
/* FSTAT register bits */
#define MAX17055_FSTAT_DNR              (1)
 
/* LIBRARY FUNCTION SUCCESS*/
#define F_SUCCESS_0  0
 
/* LIBRARY FUNCTION ERROR CODES */  
#define F_ERROR_1 -1    //-1 if I2C read/write errors exist         
#define F_ERROR_2 -2    //-2 if device is not present
#define F_ERROR_3 -3    //-3 if function error
#define F_ERROR_4 -4    //-4 if other error
#define F_ERROR_5 -5    //-5 if POR not detected
 
 
 
/**
 * @brief       max17055 Constructor
 * @details     max17055 Constructor with battery and i2c as parameters
 */
MAX17055::MAX17055()

{
    //empty block
}
 
/**
 * @brief       Fuel Gauge Destructor
 */
MAX17055::~MAX17055()
{
    //empty block
}
 
/**
 * @brief      Writes a register.
 *
 * @param[in]  reg_addr  The register address
 * @param[in]  reg_data  The register data
 *
 * @retval     0 on success
 * @retval     non-0 for errors
 */
int MAX17055::writeReg(Registers_e reg_addr, uint16_t reg_data)
{
 
  Wire.beginTransmission(I2C_W_ADRS);
  Wire.write(reg_addr);
  Wire.write( reg_data       & 0xFF); // value low byte
  Wire.write((reg_data >> 8) & 0xFF); // value high byte
  uint8_t last_status = Wire.endTransmission();
  return last_status;
}
 
/**
 * @brief      Reads from MAX17055 register.
 *
 * @param[in]  reg_addr  The register address
 * @param      value     The value
 *
 * @retval     0 on success
 * @retval     non-0 for errors
 */
int32_t MAX17055::readReg(Registers_e reg_addr, uint16_t &value)
{

    Wire.beginTransmission(I2C_W_ADRS); 
    Wire.write(reg_addr);
    uint8_t last_status = Wire.endTransmission(false);
    
    uint8_t count = Wire.requestFrom(I2C_W_ADRS, (uint8_t) 2); 
    value  = Wire.read();
    value |= (uint16_t)Wire.read() << 8;      // value low byte
    log_v("Read reg: %X, count: %d", value, count);
    if (count == 2) {
        return F_SUCCESS_0;
    } else {
        return F_ERROR_1;
    }
}
 
/**
 * @brief        Write and Verify a MAX17055 register
 * @par          Details
 *               This function writes and verifies if the writing process was successful
 *               
 * @param[in]    reg_addr     - register address
 * @param[out]   reg_data     - the variable that contains the data to write
 *                               to the register address
 *
 * @retval       0 on success
 * @retval       non-0 for errors
 */
int MAX17055::write_and_verify_reg(Registers_e reg_addr, uint16_t reg_data)
{
    int attempts=0;
    uint16_t registerValueRead;
    uint8_t lastStatus = 0;
    do {
        writeReg(reg_addr, reg_data);
        delay(10);
        lastStatus = readReg(reg_addr, registerValueRead);
        log_v("write and verify: %X vs %X", reg_data, registerValueRead);
    } while (reg_data != registerValueRead && attempts++<3);
    if (lastStatus == 0 && attempts < 3) {
        return F_SUCCESS_0;
    } else {
        return F_ERROR_1;
    }
}
 
/**
 * @brief       Initialization Function for MAX17055.
 * @par         Details
 *              This function initializes the MAX17055 for the implementation of the EZconfig model.\n
 *              The library needs to be customized for the implementation of customize model.\n  
 *              
 * @retval      0 on success 
 * @retval      non-0 for errors
 */
int MAX17055::init(platform_data des_data)
{
 
    int status, ret;
    int time_out = 30;
    uint16_t hibcfg_value,read_data;
 
 
    status = readReg(VERSION_REG, read_data);
    log_v("Version Reg: %X", status);
    if (status != F_SUCCESS_0)
        return status;
 
    ///STEP 0. Check for POR (Skip load model if POR bit is cleared)
    log_v("Step 0");
    if (check_POR_func() == F_ERROR_5)
        return F_ERROR_5;  //POR not detected. Skip Initialization.
    //This is not an error. 
    log_v("Step 1");
    ///STEP 1. Check if FStat.DNR == 0 (Do not continue until FSTAT.DNR == 0)
    ret = poll_flag_clear(FSTAT_REG, MAX17055_FSTAT_DNR, time_out);
    if (ret < F_SUCCESS_0) {
        return ret;
    }
    log_v("Step 1.2");
    ///STEP 1.2. Force exit from hibernate
    hibcfg_value = forcedExitHiberMode();
    log_v("Stored hibernate value: %X", hibcfg_value);
 
    //printf("step 1 check \r\n");
    log_v("Step 2");
    ///STEP 2. Initialize configuration
    switch (1) {
        case MODEL_LOADING_OPTION1:
            log_v("Step 2.1");
            ///STEP 2.1. Load EZ Config
            EZconfig_init(des_data);
 
            log_v("Step 2.2");
            ///STEP 2.2. Poll ModelCFG.ModelRefresh bit for clear
            ret = poll_flag_clear(MODELCFG_REG, MAX17055_MODELCFG_REFRESH, time_out);
            log_v("Result from poll flag clear: %X", ret);
            if(ret < F_SUCCESS_0) {
                return ret;
            }
 
            break; 
    }
    log_v("Step 3");
    ///STEP3. Restore original HibCfg
    writeReg(HIBCFG_REG, hibcfg_value);
 
    log_v("Clear POR");
    /* Clear Status.POR */
    ret = clear_POR_bit();
    if (ret < F_SUCCESS_0)
        return ret; //See errors   
    return F_SUCCESS_0;
}
 
/**
 * @brief      Check POR function
 * @par Details
 *     This function check is there was a power on reset event for the
 *     MAX17055
 *
 * @retval    0 on success (POR detected)
 * @retval    non-0 for errors (POR not detected)
 * 
 */
int MAX17055::check_POR_func()
{
    uint16_t read_data;
    
    readReg(STATUS_REG, read_data);
 
    if (!(read_data & MAX17055_STATUS_POR ) )
        return F_ERROR_5;  //POR not detected.
    else 
        return F_SUCCESS_0;
}

/**
 * @brief      Reads an specified register from the MAX17055 register.
 *
 * @param[in]  reg_addr  The register address
 * @param      value     The value
 *
 * @retval     reg_data register data
 * @retval     statusRead non-0 for errors
 */
 
int16_t MAX17055::get_regInfo(Registers_e reg_addr)
{
    uint16_t read_data;
    int statusRead;
 
    statusRead = readReg(reg_addr, read_data);
    if (statusRead != F_SUCCESS_0)
        return statusRead;
    else
        return read_data;
 
}
 
/**
 * @brief        clear POR bit function
 * @par          Details
 *               This function clear the indicating bit for POR - MAX17055
 *
 * @retval       0 for Success
 * @retval      non-0 for errors
 */
int MAX17055::clear_POR_bit()
{
    int status;
    uint16_t read_data;
 
 
    readReg(STATUS_REG, read_data);
    log_v("Read status value: %X", status);
    if (status != F_SUCCESS_0) {
        log_v("Clear POR bit status NOK");
        return F_ERROR_2;  //Device is not present in the i2c Bus
    }
    status = write_and_verify_reg(STATUS_REG, (read_data & MAX17055_POR_MASK));
    if (status != F_SUCCESS_0)
        return F_ERROR_1; //read or write error
    else 
        return F_SUCCESS_0;
}
 
/**
 * @brief      Poll Flag clear Function. 
 * @par Details
 *     This function clears status flags for the MAX17055
 *
 * @param[in]  reg_addr  - register address
 * @param[in]  mask      - register address
 * @param[in]  timeout   - register data
 *
 * @retval     0 on success 
 * @retval    non-0 negative for errors
 */
int MAX17055::poll_flag_clear (Registers_e reg_addr, int mask, int timeout)
{
    uint16_t data;
    int ret;
 
    do {
        delay(30);
        ret = readReg(reg_addr, data);
        /*if(ret < F_SUCCESS_0)
            return F_ERROR_1;*/
 
        if(!(data & mask))
            return F_SUCCESS_0;
        timeout -= 1;
    } while(timeout > 0);
    log_v("Error 4 in poll_flag_clear");
    return F_ERROR_4;
}
 
/**
 * @brief        Get Temperature Function from the MAX17055 TEMP register.
 * @par          Details
 *               This function sends a request to access the TEMP register
 *               of the MAX17055, which reflects the temperature measured for the fuel gauge. 
 *               The temperature values will reflect the Config Register (0x1D) selections for Tsel bit (D15).
 *               For this library the setting are for die temperature.
 *               The MAX32620FTHR thermistor bias pin is not connected. The biasing of the thermistor is
 *               done by the MAX77650. See MAX77650 library for how to enable the thermistor biasing.  
 *
 *   
 * @retval      temp - Temperature value from TEMP register in &deg;C
 * @retval      non-0 negative values check for errors
 */
int MAX17055::get_temperature()
{
 
    int ret;
    uint16_t temp;
 
    ret = readReg(TEMP_REG, temp);
    if (ret < F_SUCCESS_0)
        return ret;
 
    /* The value is signed. */
    if (temp & 0x8000)
        temp |= 0xFFFF0000;
 
    /* The value is converted into centigrade scale */
    /* Units of LSB = 1 / 256 degree Celsius */
    temp >>= 8;
 
    return temp;
}
 
/**
 * @brief        Forced Exit Hibernate Mode Function for MAX17055
 * @par          Details
 *               This function executes a force exit from hibernate mode.
 *
 * @retval       HibCFG original value before forced Exit Hibernate mode *
 */
uint16_t MAX17055::forcedExitHiberMode()
{
    uint16_t hibcfg;
 
    /* Force exit from hibernate */
    //STEP 0: Store original HibCFG value
    readReg(HIBCFG_REG, hibcfg);
 
    //STEP 1: Write to Soft-Wakeup Command Register
    writeReg(VFSOC0_QH0_LOCK_REG, 0x90); //Soft-Wakeup from hibernate
 
    //STEP 2: Write to Hibernate Configuration register
    writeReg(HIBCFG_REG, 0x0); //disable hibernate mode
 
    //STEP 3:Write to Soft-Wakeup Command Register
    writeReg(VFSOC0_QH0_LOCK_REG, 0x0); //Clear All commands
 
    return hibcfg;
}
 
/**
 * @brief        EZ Config Initialization function
 * @par          Details
 *               This function implements the steps for the EZ config m5 FuelGauge
 * @param[in]    des_data - Plataform_data struct with information about the design.
 * @retval       0 on success
 * @retval       non-zero for errors
 */
uint16_t MAX17055::EZconfig_init(platform_data des_data)
{
    ///STEP 2.1.1 EZ config values suggested by manufacturer.
    const int charger_th = 4275;
    const int chg_V_high = 51200; // scaling factor high voltage charger
    const int chg_V_low = 44138;
    const int param_EZ_FG1 = 0x8400; // Sets config bit for the charge voltage for the m5
    const int param_EZ_FG2 = 0x8000;
    uint16_t dpacc, ret;
 
    ///STEP 2.1.2 Store the EZ Config values into the appropriate registers. 
    ret = writeReg(DESIGNCAP_REG, des_data.designcap);
    ret = writeReg(DQACC_REG, des_data.designcap >> 5);  //DesignCap divide by 32
    ret = writeReg(ICHGTERM_REG, des_data.ichgterm);
    ret = writeReg(VEMPTY_REG, des_data.vempty);
 
    if (des_data.vcharge > charger_th) {
        dpacc = (des_data.designcap >> 5) * chg_V_high / des_data.designcap;
        ret = writeReg(DPACC_REG, dpacc);
        ret = writeReg(MODELCFG_REG, param_EZ_FG1); // 
    } else {
        dpacc = (des_data.designcap >> 5) * chg_V_low / des_data.designcap;
        ret = writeReg(DPACC_REG, dpacc);
        ret = writeReg(MODELCFG_REG, param_EZ_FG2);
    }
    return ret;
}
 
/**
 * @brief        Get reported State Of Charge(SOC) Function from MAX17055 Fuel Gauge.
 * @par          Details
 *               This function sends a request to access the RepSOC register
 *               of the MAX17055. RepSOC is the reported state-of-charge percentage output of the fuel gauge. 
 *
 * @retval      soc_data - Reported SOC data from the RepSOC register in % value.
 * @retval      non-0 negative values check for errors 
 */
uint16_t MAX17055::get_SOC()
{
 
    int ret;
    uint16_t soc_data;
 
    ret = readReg(REPSOC_REG, soc_data);
    if (ret < F_SUCCESS_0)
        return ret;
 
    soc_data = soc_data >> 8; /* RepSOC LSB: 1/256 % */
 
    return soc_data;
}
 
/**
 * @brief       Get at rate Average State Of Charge(SOC) Function from MAX17055 Fuel Gauge.
 * @par          Details
 *               This function sends a request to access the atAvSOC register of the MAX17055.
 *               The AvSOC registers hold the calculated available capacity and percentage of the
 *               battery based on all inputs from the ModelGauge m5 algorithm including empty 
 *               compensation. These registers provide unfiltered results. Jumps in the reported 
 *               values can be caused by abrupt changes in load current or temperature.
 *
 * @retval       atAvSOC_data - Average SOC data from the atAVSOC register in % value. 
 * @retval       non-0 negative values check for errors      
 */
int MAX17055::get_atAvSOC()
{
    int ret;
    uint16_t atAvSOC_data;
 
    ret = readReg(AVSOC_REG, atAvSOC_data);
    if (ret < F_SUCCESS_0)
        return ret; //Check errors if data is not correct
 
    atAvSOC_data = atAvSOC_data >> 8; /* avSOC LSB: 1/256 % */
 
    return atAvSOC_data;
}
 
/**
 * @brief        Get mix State Of Charge(SOC) Function for MAX17055 Fuel Gauge.
 * @par          Details
 *               This function sends a request to access mixSOC register
 *               of the MAX17055. The MixSOC registers holds the calculated
 *               remaining capacity and percentage of the cell before any empty compensation
 *               adjustments are performed.
 * 
 * @retval       mixSOC_data - Mixed SOC register values from the mixSOC register in % value. 
 * @retval       non-0 for errors
 */
int MAX17055::get_mixSOC()
{
    int ret;
    uint16_t mixSOC_data;
 
    ret = readReg(MIXSOC_REG, mixSOC_data);
    if (ret < F_SUCCESS_0)
        return ret;
 
    mixSOC_data = mixSOC_data >> 8; /* RepSOC LSB: 1/256 % */
 
    return mixSOC_data;
}
 
/**
 * @brief       Get the Time to Empty(TTE) Function form MAX17055 Fuel Gauge.
 * @par         Details
 *              This function sends a request to access the TTE register
 *              of the MAX17055
 *              The TTE register holds the estimated time to empty for the 
 *              application under present temperature and load conditions. The TTE value is 
 *              determined by relating AvCap with AvgCurrent. The corresponding AvgCurrent 
 *              filtering gives a delay in TTE, but provides more stable results.
 *
 * @retval      tte_data - Time to Empty data from the TTE register in seconds.
 * @retval      non-0 negative values check for errors
 */
float MAX17055::get_TTE()
{
 
    int ret;
    uint16_t tte_data;
    float f_tte_data;
 
    ret = readReg(TTE_REG, tte_data);
    if (ret < F_SUCCESS_0)
        return ret;
    else
        f_tte_data = ((float)tte_data * 5.625); /* TTE LSB: 5.625 sec */
 
    return f_tte_data;
}
 
/**
 * @brief       Get the at Time to Empty(atTTE) value Function for MAX17055 Fuel Gauge.
 * @par         Details
 *              This function sends a request to access the internal register
 *              of the MAX17055
 *
 * @retval      atTTE_data - Time to Empty data from the atTTE register in seconds. 
 * @retval      non-0 negative values check for errors
 */
float MAX17055::get_atTTE()
{
 
    int ret;
    uint16_t atTTE_data;
    float f_atTTE_data;
 
    ret = readReg(ATTTE_REG, atTTE_data);
    if (ret < F_SUCCESS_0)
        return ret; //Check for errors
    else
        f_atTTE_data = ((float)atTTE_data * 5.625); /* TTE LSB: 5.625 sec */
 
    return  f_atTTE_data;
}
 
/**
 * @brief      Get the Time to Full(TTE) values Function for MAX17055 Fuel Gauge.
 * @par        Details
 *             This function sends a request to access the internal register of the MAX17055
 *             The TTF register holds the estimated time to full for the application
 *             under present conditions. The TTF value is determined by learning the
 *             constant current and constant voltage portions of the charge cycle based
 *             on experience of prior charge cycles. Time to full is then estimate 
 *             by comparing present charge current to the charge termination current. 
 *             Operation of the TTF register assumes all charge profiles are consistent in the application.
 *
 * @retval     ttf_data - Time to Full data from the TTF register in seconds. 
 * @retval     non-0 negative values check for errors
 */
float MAX17055::get_TTF()
{
 
    int ret;
    uint16_t ttf_data;
    float f_ttf_data;
 
    ret = readReg(TTF_REG, ttf_data);
    if (ret < F_SUCCESS_0)
        return ret;
    else
        f_ttf_data = ((float)ttf_data * 5.625); /* TTE LSB: 5.625 sec */
 
    return  f_ttf_data;
}
 
/**
 * @brief       Get voltage of the cell Function for MAX17055 Fuel Gauge.
 * @par         Details
 *              This function sends a request to access the VCell Register
 *              of the MAX17055 to read the measured voltage from the cell. 
 *              
 * @retval      vcell_data  - vcell data from the VCELL_REG register in uVolts.
 * @retval      non-0 negative values check for errors
 */
int MAX17055::get_Vcell()
{
 
    int ret;
    uint16_t vcell_data;
 
    ret = readReg(VCELL_REG, vcell_data);
    if (ret < F_SUCCESS_0)
        return ret;
    else
        ret = lsb_to_uvolts(vcell_data);
    return ret;
}
 
/**
 * @brief       Get current Function for MAX17055 Fuel Gauge.
 * @par         Details
 *              This function sends a request to access the CURRENT register
 *              of the MAX17055 to read the current readings. 
 *
 * @param[in]   des_data - Plataform_data struct with information about the design.
 * 
 * @retval      curr_data  - current data from the CURRENT register in uAmps. 
 * @retval      non-0 negative values check for errors.
 */
int MAX17055::get_Current( platform_data des_data )
{
 
    int ret,design_rsense;
    uint16_t curr_data;
 
    ret = readReg(CURRENT_REG, curr_data);
    if (ret < F_SUCCESS_0)
        return ret;
    else
    design_rsense = des_data.rsense;
    ret = raw_current_to_uamps((uint32_t)curr_data, design_rsense);
    return ret;
}
 
/**
 * @brief       Get average current Function for MAX17055 Fuel Gauge.
 * @par         Details
 *              This function sends a request to access the aveCURRENT register
 *              of the MAX17055 to read the average current readings. 
 *
 * @param[in]   des_data - Plataform_data struct with information about the design.
 * 
 * @retval      aveCurr_data - current data from the AVGCURRENT register in uAmps. 
 * @retval      non-0 negative values check for errors.
 */
int MAX17055::get_AvgCurrent( platform_data des_data )
{
    int ret, design_rsense;
    uint16_t data;
    uint32_t aveCurr_data;
 
    ret = readReg(AVGCURRENT_REG, data);
    if (ret < F_SUCCESS_0)
        return ret;
    else
    aveCurr_data = data;
    design_rsense = des_data.rsense;
    aveCurr_data = raw_current_to_uamps(aveCurr_data, design_rsense);
    return aveCurr_data;
}
 
/**
 * @brief        lsb_to_uvolts Conversion Function 
 * @par          Details
 *               This function takes the lsb value of the register and convert it
 *               to uvolts
 *
 * @param[in]   lsb - value of register lsb
 * @retval      conv_2_uvolts - value converted lsb to uvolts        
 */
int MAX17055:: lsb_to_uvolts(uint16_t lsb)
{
    int conv_2_uvolts;
    conv_2_uvolts = (lsb * 625) / 8; /* 78.125uV per bit */
    return conv_2_uvolts;
}
 
/**
 * @brief        raw_current_to_uamp Conversion Function 
 * @par          Details
 *               This function takes the raw current value of the register and 
 *               converts it to uamps
 *
 * @param[in]   curr - raw current value of register 
 * @retval      res  - converted raw current to uamps (Signed 2's complement)         
 */
int MAX17055::raw_current_to_uamps(uint32_t curr, int rsense_value)
{
    int res = curr;
    /* Negative */
    if (res & 0x8000) {
        res |= 0xFFFF0000;
    } else {
        res *= 1562500 /(rsense_value * 1000); //Change to interact with the rsense implemented in the design
    }
    return res;
}
 
/**
 * @brief        Save Learned Parameters Function for battery Fuel Gauge model.  
 * @par          Details
 *               It is recommended to save the learned capacity parameters every
 *               time bit 2 of the Cycles register toggles
 *               (so that it is saved every 64% change in the battery) 
 *               so that if power is lost the values can easily be restored.
 *
 * @param[in]   FG_params Fuel Gauge Parameters based on design details. 
 * 
 * @retval      0 for success
 * @retval      non-0 negative for errors
 */
int MAX17055::save_Params(saved_FG_params_t FG_params)
{
    int ret;
    uint16_t data[5], value;
    ///STEP 1. Checks if the cycel register bit 2 has changed.
    ret = readReg(CYCLES_REG, data[3]);
    if (ret < F_SUCCESS_0)
        return ret;
    else {
        value = data[3];
    }
 
    ///STEP 2. Save the capacity parameters for the specific battery.
    ret = readReg(RCOMP0_REG, data[0]);
    if (ret < F_SUCCESS_0)
        return ret;
    else
        FG_params.rcomp0 = data[0];
 
    ret = readReg(TEMPCO_REG, data[1]);
    if (ret < F_SUCCESS_0)
        return ret;
    else
        FG_params.temp_co = data[1];
 
    ret = readReg(FULLCAPREP_REG, data[2]);
    if (ret < F_SUCCESS_0)
        return ret;
    else
        FG_params.full_cap_rep = data[2];
 
    FG_params.cycles = data[3];
 
    ret = readReg(FULLCAPNOM_REG, data[4]);
    if (ret < F_SUCCESS_0)
        return ret;
    else
        FG_params.full_cap_nom = data[4];
    return ret;
}
 
/**
 * @brief        Restore Parameters Function for battery Fuel Gauge model.
 * @par          Details
 *               If power is lost, then the capacity information 
 *               can be easily restored with this function. 
 *
 * @param[in]   FG_params Struct for Fuel Gauge Parameters 
 * @retval      0 for success
 * @retval      non-0 negative for errors
 */
int MAX17055::restore_Params(saved_FG_params_t FG_params)
{
    int ret;
    uint16_t temp_data, fullcapnom_data, mixCap_calc, dQacc_calc;
    uint16_t dPacc_value = 0x0C80;//Set it to 200%
 
    ///STEP 1. Restoring capacity parameters 
    write_and_verify_reg(RCOMP0_REG, FG_params.rcomp0);
    write_and_verify_reg(TEMPCO_REG, FG_params.temp_co);
    write_and_verify_reg(FULLCAPNOM_REG, FG_params.full_cap_nom);
    
    delay(350);//check the type of wait
    
    ///STEP 2. Restore FullCap
    ret = readReg(FULLCAPNOM_REG, fullcapnom_data);
    if (ret < F_SUCCESS_0)
        return ret;
 
    ret = readReg(MIXSOC_REG, temp_data); //check if Error in software guide register incorrect
    if (ret < F_SUCCESS_0)
        return ret;
    
    mixCap_calc = (temp_data*fullcapnom_data)/25600;
 
    write_and_verify_reg(MIXCAP_REG, mixCap_calc);
    write_and_verify_reg(FULLCAPREP_REG, FG_params.full_cap_rep);
    
    ///STEP 3. Write DQACC to 200% of Capacity and DPACC to 200%
    dQacc_calc = (FG_params.full_cap_nom/ 16) ;
 
    write_and_verify_reg(DPACC_REG, dPacc_value);
    write_and_verify_reg(DQACC_REG, dQacc_calc);
 
    delay(350);
 
    ///STEP 4. Restore Cycles register
    ret = write_and_verify_reg(CYCLES_REG, FG_params.cycles);
    if (ret < F_SUCCESS_0)
        return ret;
    return ret;
}
 
/**
 * @brief        Function to Save Average Current to At Rate register.
 * @par          Details
 *               For User friendliness display of atTTE, atAvSOC, atAvCAP
 *               write the average current to At Rate registers every 10sec 
 *               when the battery is in use.
 *               NOTE: do not use this function when the Battery is charging. 
 *
 * @retval      0 for success
 * @retval      non-0 negative for errors
 */
int MAX17055::avCurr_2_atRate()
{
    int ret;
    uint16_t avCurr_data;
 
    ret = readReg(AVGCURRENT_REG, avCurr_data);
    if (ret < F_SUCCESS_0){
        return ret = -3;
    }      
 
    //Write avCurrent to atRate Register
    ret = writeReg(ATRATE_REG, avCurr_data);
    if (ret < F_SUCCESS_0){
        return ret;
    }
    return F_SUCCESS_0;
}