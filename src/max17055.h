/******************************************************************//**
* @file max17055.h
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
 
#ifndef __MAX17055_H_
#define __MAX17055_H_
 
// Include
#include <Arduino.h>
#include <Wire.h>
 
/* STATUS register bits */
#define MAX17055_STATUS_BST             (1 << 3)
#define MAX17055_STATUS_POR             (1 << 1)
 
/// Model loading options
#define MODEL_LOADING_OPTION1           1 //EZ Config
// #define MODEL_LOADING_OPTION2           2 //Not implemented in this version of the library
// #define MODEL_LOADING_OPTION3           3 //Not implemented in this version of the library
//Remove this and leave it 
 
/**
 * @brief MBED Library for the MAX17055\n
 * The MAX17055 is a low 7μA operating current fuel gauge which  \n
 * implements Maxim ModelGauge™ m5 EZ algorithm.                 \n   
 * <a href="https://www.maximintegrated.com/en/design/partners-and-technology/design-technology/modelgauge-battery-fuel-gauge-technology.html">ModelGauge</a>
 * m5 EZ makes fuel gauge implementation easy by eliminating     \n
 * battery characterization requirements and simplifying host    \n
 * software interaction. The ModelGauge m5 EZ robust algorithm   \n
 * provides tolerance against battery diversity for most lithium \n
 * batteries and applications. Communication is through an       \n
 * SPI-compatible interface. The MAX17055 comes as part of the   \n 
 * MAX32620FTHR MBED enable development board.\n 
 * \n
 * Visit the product page for more information:
 * <a href="https://www.maximintegrated.com/MAX17055.html">MAX17055 Product Page</a>\n
 * <a href="https://www.maximintegrated.com/MAX17055.pdf">MAX17055 Data Sheet</a>\n
 * <a href="https://www.maximintegrated.com/MAX32620FTHR.html">MAX32620FTHR Product Page</a>\n
 * <a href="https://www.maximintegrated.com/MAX32620FTHR.pdf">MAX32620FTHR Data Sheet</a>\n
 * 
 * @code
 * 
 * ///This is not the final test code. Just sample place holder. 
 * #include "mbed.h"
 * #include "MAX17055.h"
 *
 *
 * // Hardware serial port
 * Serial serial(USBTX, USBRX);
 *
 * //SPI communications
 * I2C i2c(SCL, SDA);
 *
 * //Fuel Gauge
 * MAX17055 max17055(i2C, Sutff );//To be completed
 *
 *
 * int main(void)
 * {
 *     CODE CODE TBD
 *      while(true)
 *      {
 *          CODE CODE TBD
 *      }
 * }
 * @endcode
 */
 
 
/*-------------------------------------------------------------------------*//**
 * MAX17055 Class
 * @brief      Class for MAX17055 Battery Fuel Gauge
 *          - Generic API for Implementing the Battery Fuel Gauge
 */
class MAX17055
{
 
public:
 
    ///8-bit write address
    static const uint8_t I2C_W_ADRS = 0x36;
    ///8-bit read address
    static const uint8_t I2C_R_ADRS = 0x36;
 
    /**
     * @brief      Register Addresses for the MAX17055
     * @details    Enumerated register addresses
     */
    enum Registers_e {   
        STATUS_REG                 = 0x00, /*!< 0x00 default value = 0x0002 */
        VALRTTH_REG                = 0x01, /*!< 0x01 */
        TALRTTH_REG                = 0x02, /*!< 0x02 */ 
        SALRTTH_REG                = 0x03, /*!< 0x03 */ 
        ATRATE_REG                 = 0x04, /*!< 0x04 write negative 2s comp of a 16-bit theoretical load */
        REPCAP_REG                 = 0x05, /*!< 0x05 */ 
        REPSOC_REG                 = 0x06, /*!< 0x06 */
        TEMP_REG                   = 0x08, /*!< 0x08 */
        VCELL_REG                  = 0x09, /*!< 0x09 */
        CURRENT_REG                = 0x0A, /*!< 0x0A */ 
        AVGCURRENT_REG             = 0x0B, /*!< 0x0B */
        MIXSOC_REG                 = 0x0D, /*!< 0x0D */
        AVSOC_REG                  = 0x0E, /*!< 0x0E */
        MIXCAP_REG                 = 0x0F, /*!< 0x0F */
 
        FULLCAPREP_REG             = 0x10, /*!< 0x10 */
        TTE_REG                    = 0X11, /*!< 0x11 */
        QRTABLE00_REG              = 0x12, /*!< 0x12 */
        FULLSOCTHR_REG             = 0x13, /*!< 0x13 */
        CYCLES_REG                 = 0x17, /*!< 0x17 */
        DESIGNCAP_REG              = 0x18, /*!< 0x18 */
        AVGVCELL_REG               = 0x19, /*!< 0x19 */
        MAXMINVOLT_REG             = 0x1B, /*!< 0x1B */
        CONFIG_REG                 = 0x1D, /*!< 0x1D default = 0x2210 */
        ICHGTERM_REG               = 0x1E, /*!< 0x1E */
 
        TTF_REG                    = 0x20, /*!< 0x20 */
        VERSION_REG                = 0x21, /*!< 0x21 */
        QRTABLE10_REG              = 0x22, /*!< 0x22 */
        FULLCAPNOM_REG             = 0x23, /*!< 0x23 */
        LEARNCFG_REG               = 0x28, /*!< 0x28 */
        RELAXCFG_REG               = 0x2A, /*!< 0x2A */
        TGAIN_REG                  = 0x2C, /*!< 0x2C */
        TOFF_REG                   = 0x2D, /*!< 0x2D */
 
        QRTABLE20_REG              = 0x32, /*!< 0x32 */
        RCOMP0_REG                 = 0x38, /*!< 0x38 */
        TEMPCO_REG                 = 0x39, /*!< 0x39 */
        VEMPTY_REG                 = 0x3A, /*!< 0x39 */
        FSTAT_REG                  = 0x3D, /*!< 0x39 */
 
        QRTABLE30_REG              = 0x42, /*!< 0x39 */
        DQACC_REG                  = 0x45, /*!< 0x39 */
        DPACC_REG                  = 0x46, /*!< 0x39 */
        VFSOC0_REG                 = 0x48, /*!< 0x39 */
        QH0_REG                    = 0x4C, /*!< 0x39 */
        QH_REG                     = 0x4D, /*!< 0x39 */
 
        VFSOC0_QH0_LOCK_REG        = 0x60, /*!< 0x39 */
        LOCK1_REG                  = 0x62, /*!< 0x39 */
        LOCK2_REG                  = 0x63, /*!< 0x39 */
 
        MODELDATA_START_REG        = 0x80, /*!< 0x39 */
 
        IALRTTH_REG                = 0xB4, /*!< 0x39 */
        CURVE_REG                  = 0xB9, /*!< 0x39 */
        HIBCFG_REG                 = 0xBA, /*!< 0x39 default = 0x870C (0x890C)*/
        CONFIG2_REG                = 0xBB, /*!< 0xBB default = 0x3658 */
 
        MODELCFG_REG               = 0xDB, /*!< 0xDB */
        ATTTE_REG                  = 0xDD, /*!< 0xDD */
        ATAVSOC_REG                = 0xDE, /*!< 0xDE */
        ATAVCAP_REG                = 0xDF, /*!< 0xDF */
 
        OCV_REG                    = 0xFB, /*!< 0x39 */
        VFSOC_REG                  = 0xFF  /*!< 0x39 */
    };
      
    /**
     * @brief      Saved Platform Data for Fuel Gauge Model
     * @details    Struct with fuel Gauge Platform Data for Fuel Gauge Model based on the final design.
     */
    struct platform_data{  //to clarify if Part of the class
        uint16_t designcap;/*!< struct value 1 */
        uint16_t ichgterm; /*!< struct value 2 */
        uint16_t vempty;   /*!< struct value 3 */
        int vcharge;       /*!< struct value 1 */
 
        uint16_t learncfg;  /*!< struct value 1 */
        uint16_t relaxcfg;  /*!< struct value 1 */
        uint16_t config;    /*!< struct value 1 */
        uint16_t config2;   /*!< struct value 1 */
        uint16_t fullsocthr;/*!< struct value 1 */
        uint16_t tgain;     /*!< struct value 1 */
        uint16_t toff;      /*!< struct value 1 */
        uint16_t curve;     /*!< struct value 1 */
        uint16_t rcomp0;    /*!< struct value 1 */
        uint16_t tempco;    /*!< struct value 1 */ 
        uint16_t qrtable00; 
        uint16_t qrtable10;
        uint16_t qrtable20;
        uint16_t qrtable30;
 
        uint16_t dpacc;
        uint16_t modelcfg;
 
        //uint16_t model_data[MAX17055_TABLE_SIZE];
        int (*get_charging_status)(void);
        int model_option;
        /**
         * rsense in miliOhms.
         * default 10 (if rsense = 0) as it is the recommended value by
         * the datasheet although it can be changed by board designers.
         */
        unsigned int rsense; 
        int volt_min;   /**< in mV */
        int volt_max;   /**< in mV */
        int temp_min;   /**< in DegreC */
        int temp_max;   /**< in DegreeC */
        int soc_max;    /**< in percent */
        int soc_min;    /**< in percent */
        int curr_max;   /**< in mA */
        int curr_min;   /**< in mA */
     } ;
    
    /**
     * @brief      Saved Fuel Gauge Parameters
     * @details    It is recommended to save the learned capacity parameters
     *             every time bit 2 of the Cycles register toggles (so that it
     *             is saved every 64% change in the battery) so that if power is
     *             lost the values can easily be restored.
     */
    struct saved_FG_params_t{
        int rcomp0;              /**< The RComp0 is the characterization information critical to computing the open-circuit voltage of a cell under loaded conditions. */
        int temp_co;             /**< The TempCo value is the temperature compensation information based on the RComp0 value*/
        int full_cap_rep;        /**< The full capacity in relation with RepCap for reporting to the GUI. A new full-capacity value is calculated at the end of every charge cycle in the application. */
        int cycles;              /**< The Cycles value maintains a total count of the number of charge/discharge cycles of the cell that have occurred */
        int full_cap_nom;        /**< This is the calculated full capacity of the cell, not including temperature and empty compensation. A new full-capacity nominal value
                                    is calculated each time a cell relaxation event is detected. This values is used to generate other outputs of the ModelGauge m5 algorithm. */
     } ;
 
    /**
     * @brief       max17055 Constructor
     */
    MAX17055();
 
    /**
     * @brief       Fuel Gauge Destructor
     */
    ~MAX17055();
    
    /**
     * @brief      Poll Flag clear Function.
     */
    int poll_flag_clear(Registers_e reg_addr, int mask, int timeout);
 
    /**
     * @brief      Check POR function
     */
    int check_POR_func();
 
    /**
     * @brief        clear POR bit function
     */
    int clear_POR_bit();
 
    /**
     * @brief       Write and Verify a MAX17055 register
     */
    int write_and_verify_reg(Registers_e reg_addr, uint16_t reg_data);
 
    /**
     * @brief       Initialization Function for MAX17055.
     */
    int init(platform_data des_data);
 
    /**
     * @brief       Get Temperature Function from the MAX17055 TEMP register.
     */
    int get_temperature();
 
    /**
     * @brief       Forced Exit Hibernate Mode Function for MAX17055
     */
    uint16_t forcedExitHiberMode();// Hibernate spelling 
    
    /**
     * @brief       EZ Config Initialization function
     */
    uint16_t EZconfig_init(platform_data des_data);
      
    /**
     * @brief       Get reported State Of Charge(SOC) Function from MAX17055 Fuel Gauge
     */
    uint16_t get_SOC();
 
    /**
     * @brief       Get at rate Average State Of Charge(SOC) Function from MAX17055 Fuel Gauge.
     */
    int get_atAvSOC();
 
    /**
     * @brief       Get the Time to Empty(TTE) Function form MAX17055 Fuel Gauge.
     */   
    float get_TTE();
 
    /**
     * @brief       Get the at Time to Empty(atTTE) value Function for MAX17055 Fuel Gauge.
     */   
    float get_atTTE();
 
    /**
     * @brief        Get mix State Of Charge(SOC) Function for MAX17055 Fuel Gauge.
     */
    int get_mixSOC();
 
    /**
     * @brief      Get the Time to Full(TTE) values Function for MAX17055 Fuel Gauge.
     */
    float get_TTF();
    
    /**
     * @brief       Get voltage of the cell Function for MAX17055 Fuel Gauge.
     */
    int get_Vcell();
 
    /**
     * @brief       Get current Function for MAX17055 Fuel Gauge.
     */
    int get_Current(platform_data des_data);
    
    /**
     * @brief       Get average current Function for MAX17055 Fuel Gauge.
     */  
    int get_AvgCurrent(platform_data des_data);
    
    /**
     * @brief        lsb_to_uvolts Conversion Function         
     */
    int lsb_to_uvolts(uint16_t lsb);
    
    /**
     * @brief        raw_current_to_uamp Conversion Function         
     */
    int raw_current_to_uamps(uint32_t curr, int rsense_value);
 
    /**
     * @brief        Save Learned Parameters Function for battery Fuel Gauge model.
     */
    int save_Params(saved_FG_params_t FG_params);
 
    /**
     * @brief        Restore Parameters Function for battery Fuel Gauge model.
     */
    int restore_Params(saved_FG_params_t FG_params);
 
    /**
     * @brief        Function to Save Average Current to At Rate register.
     */
    int avCurr_2_atRate();

    /**
     * @brief       Get specified register info Function for MAX17055 Fuel Gauge.
     */
    int16_t get_regInfo(Registers_e reg_addr);
 
protected:
 
    /**
     * @brief      Writes to MAX17055 register.
     */
    int writeReg(const Registers_e reg_addr, uint16_t reg_data);
 
 
    /**
     * @brief      Reads from MAX17055 register.
     */
    int32_t readReg(Registers_e reg_addr, uint16_t &value);
 
 
 
 
private:
 

 
};
 
#endif /* _MAX17055_H_ */
 