#include "main.h"
#define  sck_l    HAL_GPIO_WritePin(GPIOB, INA_sck_Pin, GPIO_PIN_RESET)
#define  sck_h    HAL_GPIO_WritePin(GPIOB, INA_sck_Pin, GPIO_PIN_SET)
#define  sda_l    HAL_GPIO_WritePin(GPIOB, INA_sda_Pin, GPIO_PIN_RESET)
#define  sda_h    HAL_GPIO_WritePin(GPIOB, INA_sda_Pin, GPIO_PIN_SET)
#define  read_sda HAL_GPIO_ReadPin(GPIOB, INA_sda_Pin)
#define  ina_r     0x89
#define  ina_w     0x88
#define  CONF      0x00
#define  V_DAT     0x01
#define  BUS_DAT   0x02
#define  W_DAT     0x03
#define  A_DAT     0x04
#define  AWC_DAT   0x05
#define  RST       0X8000
#define  BUS_16      0X0000
#define  BUS_32      0X2000
#define  V_40        0X0000
#define  V_80        0X0800
#define  V_160       0X1000
#define  V_320       0X1800
#define INA219_I2C_ADDRESS_CONF_0               (uint8_t)(0x40 << 1)     // A0 = GND, A1 = GND
#define INA219_I2C_ADDRESS_CONF_1               (uint8_t)(0x41 << 1)     // A0 = VS+, A1 = GND
#define INA219_I2C_ADDRESS_CONF_2               (uint8_t)(0x42 << 1)     // A0 = SDA, A1 = GND
#define INA219_I2C_ADDRESS_CONF_3               (uint8_t)(0x43 << 1)     // A0 = SCL, A1 = GND
#define INA219_I2C_ADDRESS_CONF_4               (uint8_t)(0x44 << 1)     // A0 = GND, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_5               (uint8_t)(0x45 << 1)     // A0 = VS+, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_6               (uint8_t)(0x46 << 1)     // A0 = SDA, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_7               (uint8_t)(0x47 << 1)     // A0 = SCL, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_8               (uint8_t)(0x48 << 1)     // A0 = GND, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_9               (uint8_t)(0x49 << 1)     // A0 = VS+, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_A               (uint8_t)(0x4A << 1)     // A0 = SDA, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_B               (uint8_t)(0x4B << 1)     // A0 = SCL, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_C               (uint8_t)(0x4C << 1)     // A0 = GND, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_D               (uint8_t)(0x4D << 1)     // A0 = VS+, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_E               (uint8_t)(0x4E << 1)     // A0 = SDA, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_F               (uint8_t)(0x4F << 1)     // A0 = SCL, A1 = SCL
#define INA219_I2C_ADDRESS                      INA219_I2C_ADDRESS_CONF_0
#define INA219_REG_CONFIG                       (uint8_t)(0x00)      // CONFIG REGISTER (R/W)
#define INA219_REG_SHUNTVOLTAGE                 (uint8_t)(0x01)      // SHUNT VOLTAGE REGISTER (R)
#define INA219_REG_BUSVOLTAGE                   (uint8_t)(0x02)      // BUS VOLTAGE REGISTER (R)
#define INA219_REG_POWER                        (uint8_t)(0x03)      // POWER REGISTER (R)
#define INA219_REG_CURRENT                      (uint8_t)(0x04)      // CURRENT REGISTER (R)
#define INA219_REG_CALIBRATION                  (uint8_t)(0x05)      // CALIBRATION REGISTER (R/W)
#define INA219_CFGB_RESET(x)                    (uint16_t)((x & 0x01) << 15)     // Reset Bit
#define INA219_CFGB_BUSV_RANGE(x)               (uint16_t)((x & 0x01) << 13)     // Bus Voltage Range
#define INA219_CFGB_PGA_RANGE(x)                (uint16_t)((x & 0x03) << 11)     // Shunt Voltage Range
#define INA219_CFGB_BADC_RES_AVG(x)             (uint16_t)((x & 0x0F) << 7)      // Bus ADC Resolution/Averaging
#define INA219_CFGB_SADC_RES_AVG(x)             (uint16_t)((x & 0x0F) << 3)      // Shunt ADC Resolution/Averaging
#define INA219_CFGB_MODE(x)                     (uint16_t) (x & 0x07)      
#define INA219_CFG_RESET                        INA219_CFGB_RESET(1)            // Reset Bit
 
#define INA219_CFG_BVOLT_RANGE_MASK             INA219_CFGB_BUSV_RANGE(1)       // Bus Voltage Range Mask
#define INA219_CFG_BVOLT_RANGE_16V              INA219_CFGB_BUSV_RANGE(0)       // 0-16V Range
#define INA219_CFG_BVOLT_RANGE_32V              INA219_CFGB_BUSV_RANGE(1)       // 0-32V Range (default)
 
#define INA219_CFG_SVOLT_RANGE_MASK             INA219_CFGB_PGA_RANGE(3)        // Shunt Voltage Range Mask
#define INA219_CFG_SVOLT_RANGE_40MV             INA219_CFGB_PGA_RANGE(0)        // Gain 1, 40mV Range
#define INA219_CFG_SVOLT_RANGE_80MV             INA219_CFGB_PGA_RANGE(1)        // Gain 2, 80mV Range
#define INA219_CFG_SVOLT_RANGE_160MV            INA219_CFGB_PGA_RANGE(2)        // Gain 4, 160mV Range
#define INA219_CFG_SVOLT_RANGE_320MV            INA219_CFGB_PGA_RANGE(3)        // Gain 8, 320mV Range (default)
 
#define INA219_CFG_BADCRES_MASK                 INA219_CFGB_BADC_RES_AVG(15)    // Bus ADC Resolution and Averaging Mask
#define INA219_CFG_BADCRES_9BIT_1S_84US         INA219_CFGB_BADC_RES_AVG(0)     // 1 x 9-bit Bus sample
#define INA219_CFG_BADCRES_10BIT_1S_148US       INA219_CFGB_BADC_RES_AVG(1)     // 1 x 10-bit Bus sample
#define INA219_CFG_BADCRES_11BIT_1S_276US       INA219_CFGB_BADC_RES_AVG(2)     // 1 x 11-bit Bus sample
#define INA219_CFG_BADCRES_12BIT_1S_532US       INA219_CFGB_BADC_RES_AVG(3)     // 1 x 12-bit Bus sample (default)
#define INA219_CFG_BADCRES_12BIT_2S_1MS         INA219_CFGB_BADC_RES_AVG(9)     // 2 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_4S_2MS         INA219_CFGB_BADC_RES_AVG(10)    // 4 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_8S_4MS         INA219_CFGB_BADC_RES_AVG(11)    // 8 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_16S_8MS        INA219_CFGB_BADC_RES_AVG(12)    // 16 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_32S_17MS       INA219_CFGB_BADC_RES_AVG(13)    // 32 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_64S_34MS       INA219_CFGB_BADC_RES_AVG(14)    // 64 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_128S_68MS      INA219_CFGB_BADC_RES_AVG(15)    // 128 x 12-bit Bus samples averaged together
 
#define INA219_CFG_SADCRES_MASK                 INA219_CFGB_SADC_RES_AVG(15)    // Shunt ADC Resolution and Averaging Mask
#define INA219_CFG_SADCRES_9BIT_1S_84US         INA219_CFGB_SADC_RES_AVG(0)     // 1 x 9-bit Shunt sample
#define INA219_CFG_SADCRES_10BIT_1S_148US       INA219_CFGB_SADC_RES_AVG(1)     // 1 x 10-bit Shunt sample
#define INA219_CFG_SADCRES_11BIT_1S_276US       INA219_CFGB_SADC_RES_AVG(2)     // 1 x 11-bit Shunt sample
#define INA219_CFG_SADCRES_12BIT_1S_532US       INA219_CFGB_SADC_RES_AVG(3)     // 1 x 12-bit Shunt sample (default)
#define INA219_CFG_SADCRES_12BIT_2S_1MS         INA219_CFGB_SADC_RES_AVG(9)     // 2 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_4S_2MS         INA219_CFGB_SADC_RES_AVG(10)    // 4 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_8S_4MS         INA219_CFGB_SADC_RES_AVG(11)    // 8 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_16S_8MS        INA219_CFGB_SADC_RES_AVG(12)    // 16 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_32S_17MS       INA219_CFGB_SADC_RES_AVG(13)    // 32 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_64S_34MS       INA219_CFGB_SADC_RES_AVG(14)    // 64 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_128S_68MS      INA219_CFGB_SADC_RES_AVG(15)    // 128 x 12-bit Shunt samples averaged together
 
#define INA219_CFG_MODE_MASK                    INA219_CFGB_MODE(7)             // Operating Mode Mask
#define INA219_CFG_MODE_POWERDOWN               INA219_CFGB_MODE(0)             // Power-Down
#define INA219_CFG_MODE_SVOLT_TRIGGERED         INA219_CFGB_MODE(1)             // Shunt Voltage, Triggered
#define INA219_CFG_MODE_BVOLT_TRIGGERED         INA219_CFGB_MODE(2)             // Bus Voltage, Triggered
#define INA219_CFG_MODE_SANDBVOLT_TRIGGERED     INA219_CFGB_MODE(3)             // Shunt and Bus, Triggered
#define INA219_CFG_MODE_ADCOFF                  INA219_CFGB_MODE(4)             // ADC Off (disabled)
#define INA219_CFG_MODE_SVOLT_CONTINUOUS        INA219_CFGB_MODE(5)             // Shunt Voltage, Continuous
#define INA219_CFG_MODE_BVOLT_CONTINUOUS        INA219_CFGB_MODE(6)             // Bus Voltage, Continuous
#define INA219_CFG_MODE_SANDBVOLT_CONTINUOUS    INA219_CFGB_MODE(7)             // Shunt and Bus, Continuous (default)
 
typedef struct
{
    signed long voltage_ina219;//signed short
  signed long  shunt_ina219;
  signed long  current_ina219;
  signed long  power_ina219;
}INA219_DATA;
 
 
extern uint8_t  ina219_busVolt_LSB_mV;
extern uint8_t   ina219_shuntVolt_LSB_uV;
extern unsigned short ina219_calValue;
 
extern uint32_t  ina219_current_LSB_uA;
extern uint32_t  ina219_power_LSB_mW; 
uint8_t  ina219_busVolt_LSB_mV = 4;   // Bus Voltage LSB value = 4mV
uint8_t  ina219_shuntVolt_LSB_uV = 10;  // Shunt Voltage LSB value = 10uV
unsigned short ina219_calValue = 0;
 
uint32_t ina219_current_LSB_uA;
uint32_t ina219_power_LSB_mW;
 
 INA219_DATA ina219_data;
 
uint8_t ram_for_ina219[60];
uint8_t INA219process_flag;

// Bus Voltage Register
#define INA219_BVOLT_CNVR                       (uint16_t)(0x0002)       // Conversion Ready
#define INA219_BVOLT_OVF                        (uint16_t)(0x0001)       // Math Overflow Flag



#define  power_down         0x0000
#define  bus_c         0x0007
void delay_ms(uint32_t dat)
{
	uint32_t i;
	while(dat)
	{
			i = 1000;
		while(i)
			i--;
		  dat--;		
	}	
}



 void SDA_OUT(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(GPIOB, INA_sda_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = INA_sda_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
 void SDA_IN(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(GPIOB, INA_sda_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = INA_sda_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void i2s_start(void)
{
	SDA_OUT();
	sda_h;
	sck_l;
	sck_h;
	sda_l;
	sck_l;
	
}
void i2s_stop(void)
{
	SDA_OUT();
	sda_l;
	sck_l;
	sck_h;
	sda_h;
	sck_l;
	
}
void send_dat(uint8_t dat)
{
	uint8_t i;
	SDA_OUT();
	for(i=0;i<8;i++)
	{
		if(dat&0x80)
			sda_h;
		else
			sda_l;
		sck_h;
		sck_l;
		dat<<=1;		
	}	
}
void read_ack(void)
{
	sda_h ;
	SDA_IN();
	sck_h;
	if(GPIO_PIN_RESET==read_sda)
	{
		// HAL_GPIO_WritePin(GPIOE, LED_1_Pin, read_sda);
	}
	if(GPIO_PIN_SET==read_sda)
	{
		
	}
	  // HAL_GPIO_WritePin(GPIOE, LED_1_Pin, read_sda);
	sck_l;
}
uint8_t read_dat(void)
{
	uint8_t i,dat;
	SDA_IN();
	for(i=0;i<8;i++)
	{	
		dat<<=1;
	  sck_h;
	 if(GPIO_PIN_SET==read_sda)	
		 dat |=0x01;
	  sck_l;
	}
  return dat;	
}
void send_ack(void)
{

	SDA_OUT();
	sda_l ;
	sck_h;
	sck_l;
	sda_h ;
}
void send_nack(void)
{

	SDA_OUT();
	sda_h ;
	sck_h;
	sck_l;	
}

void set_addar(uint8_t addar)
{
		i2s_start();
		send_dat(ina_w);
	  read_ack();
	  send_dat(addar);
		send_ack();
	  i2s_stop();	
	
}
void write_dat(uint8_t addar,uint8_t dat_h,uint8_t dat_l)
{
		i2s_start();
		send_dat(ina_w);
	  read_ack();
	  send_dat(addar);
		send_ack();
	  send_dat(dat_h);
		send_ack();
	  send_dat(dat_l);
		send_ack();
	  i2s_stop();	
	
}
void ina219_Write_Register(unsigned char reg, unsigned int dat)
{
    unsigned char val[2];
    
    val[0] = (unsigned char)(dat >> 8);
    val[1] = (unsigned char)(dat & 0xFF);
	  write_dat(reg,val[0],val[1]);
}
void ina219_Read_Register(uint8_t  reg,signed short  *dat)
{
    //printf("read reg == %d\r\n",reg);
  unsigned char val[2];
  
    set_addar(reg);
		i2s_start();
		send_dat(ina_r);
	  read_ack();
	  val[0] =	read_dat();
		send_ack();
		val[1] =	read_dat();
		send_nack();
	  i2s_stop();	
  *dat = ((unsigned int)(val[0]) << 8) + val[1];
  
    //printf("data1 == %x\r\n",val[0]);
    //printf("data2 == %x\r\n",val[1]);
    
}
void ina219_SetCalibration_16V_16A(void)
{
  uint16_t configValue;
  
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.
  
  // VBUS_MAX     = 16V   (Assumes 16V, can also be set to 32V)
  // VSHUNT_MAX   = 0.32  (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
  // RSHUNT       = 0.02   (Resistor value in ohms)
  
  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 16A
  
  // 2. Determine max expected current
  // MaxExpected_I = 16A
  
  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.00048            (0.48mA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0,00390            (3.9mA per bit)
  
  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.00050            (500uA per bit)
  
  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 4096 (0x1000)
  
  ina219_calValue = 0x0D90;  //0x1000;
  
  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.01 (10mW per bit)
  
  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 16.3835A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.32V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If
  
  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 1.6 * 16V
  // MaximumPower = 256W
  
  // Set multipliers to convert raw current/power values
  ina219_current_LSB_uA = 100;     // Current LSB = 500uA per bit
  ina219_power_LSB_mW = 2;        // Power LSB = 10mW per bit = 20 * Current LSB
  
  // Set Calibration register to 'Cal' calculated above
  ina219_Write_Register(INA219_REG_CALIBRATION, ina219_calValue);
  
  // Set Config register to take into account the settings above
  configValue = (INA219_CFG_BVOLT_RANGE_16V | INA219_CFG_SVOLT_RANGE_320MV | INA219_CFG_BADCRES_12BIT_16S_8MS | INA219_CFG_SADCRES_12BIT_16S_8MS | INA219_CFG_MODE_SANDBVOLT_CONTINUOUS );
  
  ina219_Write_Register(INA219_REG_CONFIG, configValue);
}
void ina219_configureRegisters(void)
{
  delay_ms(15);
  
  ina219_SetCalibration_16V_16A();
}
void ina219_init(void)
{
 // ina219_gpio_init();
  
  ina219_configureRegisters();
}
signed short ina219_GetBusVoltage_raw(void)
{
  signed short val;
  
  ina219_Read_Register(INA219_REG_BUSVOLTAGE, &val);
  val >>= 3;                      // Shift to the right 3 to drop CNVR and OVF
  
  return (val);
}
signed short ina219_GetCurrent_raw(void)
{
  signed short val;
  
  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  ina219_Write_Register(INA219_REG_CALIBRATION, ina219_calValue);
  
  // Now we can safely read the CURRENT register!
  ina219_Read_Register(INA219_REG_CURRENT, &val);
  
  return (val);
}
signed short ina219_GetBusVoltage_mV(void)
{
  signed short val;
  
  ina219_Read_Register(INA219_REG_BUSVOLTAGE, &val);
	  OLED_P10x10Ch(6*8,4,val/10000);
		OLED_P10x10Ch(7*8,4,val%10000/1000);
    OLED_P10x10Ch(8*8,4,val%1000/100);
	  OLED_P10x10Ch(9*8,4,val%100/10);
		OLED_P10x10Ch(10*8,4,val%10);	
	
	
  val >>= 3;                      // Shift to the right 3 to drop CNVR and OVF
  val *= ina219_busVolt_LSB_mV;   // multiply by LSB(4mV)
 
  return (val);
}
uint32_t ina219_GetShuntVoltage_uV(void)
{
  uint32_t val;
  signed short reg;
  
  ina219_Read_Register(INA219_REG_SHUNTVOLTAGE, &reg);
  val = (uint32_t)reg * ina219_shuntVolt_LSB_uV;   // multiply by LSB(10uV)
  
  return (val);
}
uint32_t ina219_GetCurrent_uA(void)
{
  uint32_t val;
  signed short reg;
  
  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  ina219_Write_Register(INA219_REG_CALIBRATION, ina219_calValue);
  
  // Now we can safely read the CURRENT register!
  ina219_Read_Register(INA219_REG_CURRENT, &reg);
  
  val = (uint32_t)reg * ina219_current_LSB_uA;
  
  return (val);
}
uint32_t ina219_GetPower_mW(void)
{
  uint32_t val;
  signed short reg;
  
  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  ina219_Write_Register(INA219_REG_CALIBRATION, ina219_calValue);
  
  // Now we can safely read the POWER register!
  ina219_Read_Register(INA219_REG_POWER, &reg);
  
  val = (uint32_t)reg * ina219_power_LSB_mW;
  
  return (val);
}



