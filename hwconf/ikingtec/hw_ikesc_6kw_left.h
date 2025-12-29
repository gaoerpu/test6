/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
	
#ifndef HW_IKESC_6kw_LEFT_H_
#define HW_IKESC_6kw_LEFT_H_

//hardware version

#define HW_NAME					    "IKESC_6kw_LEFT" PRODUCT_VERSION_SUFFIX

//#define HW_NAME					"IKESC_6kw_LEFT"

// HW properties
#define HW_HAS_3_SHUNTS
//#define HW_HAS_PHASE_SHUNTS       //相位分流采样   要开这个，硬件上必须是要电流采样独立布局，而低端采样都是同时接地的
#define HW_HAS_PHASE_FILTERS        //相位滤波功能

//PC9管相位滤波（上游噪）,PD2电流滤波（下游噪） 
/*滤波层级（3 层，PC9/PD2 在硬件层）：
硬件相位滤波（PC9 控）：上游 RC，滤桥臂噪。
硬件电流滤波（PD2 控）：下游 RC，滤 shunt 噪。
软件滤波（无 GPIO）：FOC > Current Filter */

// Macros    宏
#define LED_GREEN_GPIO			GPIOB
#define LED_GREEN_PIN			5
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN			    7

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

// mk 75_300 添加phase filtering功能。 
//通过外观判断版本，最明显特征是003版有拨动开关。
#if defined (IKESC_6kw_LEFT)
#define HW_HAS_PHASE_FILTERS
#define PHASE_FILTER_GPIO		GPIOC
#define PHASE_FILTER_PIN		9
#define PHASE_FILTER_ON()		palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
#define PHASE_FILTER_OFF()		palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)

// Shutdown pin
#define HW_SHUTDOWN_GPIO		GPIOC
#define HW_SHUTDOWN_PIN			5
#define HW_SHUTDOWN_HOLD_ON()	palSetPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
#define HW_SHUTDOWN_HOLD_OFF()	palClearPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
#define HW_SAMPLE_SHUTDOWN()	hw_sample_shutdown_button()

// Hold shutdown pin early to wake up on short pulses
#define HW_EARLY_INIT()			palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL); \
					            HW_SHUTDOWN_HOLD_ON();                    
#endif

//#define AUX_GPIO			GPIOC
//#define AUX_PIN			12
//#define AUX_ON()			palSetPad(AUX_GPIO, AUX_PIN)
//#define AUX_OFF()			palClearPad(AUX_GPIO, AUX_PIN)

//mk 75_300 在2023年11月添加了phase filtering和current filtering。
//但由于是低端采样，我们不建议打开current filtering，因此默认关闭。
//官方原理图用高端采样
#define CURRENT_FILTER_GPIO		GPIOD
// #define CURRENT_FILTER_ON()		palSetPad(CURRENT_FILTER_GPIO, 2)
#define CURRENT_FILTER_OFF()	palClearPad(CURRENT_FILTER_GPIO, 2)

/*
 * ADC Vector
 *
 * 0  (1):	IN0		SENS1
 * 1  (2):	IN1		SENS2
 * 2  (3):	IN2		SENS3
 * 3  (1):	IN10	CURR1
 * 4  (2):	IN11	CURR2
 * 5  (3):	IN12	CURR3
 * 6  (1):	IN5		ADC_EXT1
 * 7  (2):	IN6		ADC_EXT2
 * 8  (3):	IN3		TEMP_MOS
 * 9  (1):	IN14	TEMP_MOTOR
 * 10 (2):	IN15	SHUTDOWN
 * 11 (3):	IN13	AN_IN
 * 12 (1):	Vrefint
 * 13 (2):	IN0		SENS1
 * 14 (3):	IN1		SENS2
 * 15 (1):  IN8		TEMP_MOS_2
 * 16 (2):  IN9		TEMP_MOS_3
 * 17 (3):  IN3		SENS3
 */

#define HW_ADC_CHANNELS			18
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			6

// ADC Indexes
#define ADC_IND_SENS1			0
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			2
#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT			    6
#define ADC_IND_EXT2			7
#define ADC_IND_SHUTDOWN		10
#define ADC_IND_TEMP_MOS		8
//#define ADC_IND_TEMP_MOS_2		15
//#define ADC_IND_TEMP_MOS_3		16
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12

// ADC macros and settings

// Component parameters (can be overridden)  组件参数（可以重写）
#ifndef V_REG
#define V_REG				    3.30
#endif

//分压采集电路：560K和21.5K电阻
#ifndef VIN_R1
#define VIN_R1				    56000.0 
#endif
#ifndef VIN_R2
#define VIN_R2				    2121.0  
#endif

#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0 
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		(0.0005 / 3.0) 
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15) 

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		0
#endif

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6

// UART Peripheral
#define HW_UART_DEV			    SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

// Permanent UART Peripheral (for NRF51)
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD4
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOC
#define HW_UART_P_TX_PIN		10
#define HW_UART_P_RX_PORT		GPIOC
#define HW_UART_P_RX_PIN		11

// ICU Peripheral for servo decoding   (Input Capture Unit，输入捕获单元)
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV			    ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO			    GPIOB
#define HW_ICU_PIN			    6

// I2C Peripheral
#define HW_I2C_DEV			    I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM			    TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// SPI pins
#define HW_SPI_DEV			    SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

// Measurement macros
#define ADC_V_L1			    ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2			    ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3			    ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO			    (ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Override dead time. See the stm32f4 reference manual for calculating this value.
#define HW_DEAD_TIME_NSEC		660.0

// Default setting overrides  默认设置覆盖
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			15.0	// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			90.0	// Maximum input voltage
#endif
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV				    30000.0 
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		200.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			200.0	// Input current limit in Amperes (Upper) -
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-75.0	// Input current limit in Amperes (Lower)
#endif

// Setting limits  
#define HW_LIM_CURRENT			-100.0, 100.0  // ±100A
#define HW_LIM_CURRENT_IN		-280.0, 280.0 
#define HW_LIM_CURRENT_ABS		0.0, 200  // 峰值200A
#define HW_LIM_VIN			    12.0, 90.0  // 
#define HW_LIM_ERPM			    -200e3, 200e3 
#define HW_LIM_DUTY_MIN			0.0, 0.07
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0

//下面主要是用来区分6KW和20KW电机在控制上参数不同，原理是强制覆盖mcconf_default.h中的默认值

#define APPCONF_CONTROLLER_ID  20      //CAN ID识别号

#undef  MCCONF_L_CURRENT_MAX          // 为了保险，先 undef 一下
#define MCCONF_L_CURRENT_MAX            84.25   

#undef  MCCONF_L_CURRENT_MIN
#define MCCONF_L_CURRENT_MIN            -84.25

#undef  MCCONF_FOC_CURRENT_KP
#define MCCONF_FOC_CURRENT_KP			0.0588

#undef  MCCONF_FOC_CURRENT_KI
#define MCCONF_FOC_CURRENT_KI			37.63

#undef  MCCONF_FOC_DT_US
#define MCCONF_FOC_DT_US				0.12 // Microseconds for dead time compensation

#undef  MCCONF_M_INVERT_DIRECTION
#define MCCONF_M_INVERT_DIRECTION		true // 电机方向  true  false

#undef  MCCONF_FOC_CC_DECOUPLING
#define MCCONF_FOC_CC_DECOUPLING		FOC_CC_DECOUPLING_CROSS // Current controller decoupling电流控制器解耦

#undef  MCCONF_FOC_OBSERVER_TYPE
#define MCCONF_FOC_OBSERVER_TYPE		FOC_OBSERVER_MXLEMMING_LAMBDA_COMP // Position observer type for FOC

#undef  MCCONF_FOC_SENSOR_MODE
#define MCCONF_FOC_SENSOR_MODE			FOC_SENSOR_MODE_SENSORLESS

#undef  MCCONF_FOC_SAT_COMP_MODE
#define MCCONF_FOC_SAT_COMP_MODE		SAT_COMP_FACTOR		// 电感补偿

#undef  MCCONF_FOC_SAT_COMP
#define MCCONF_FOC_SAT_COMP				0.1		// Stator saturation compensation factor

#undef  MCCONF_L_RPM_MAX
#define MCCONF_L_RPM_MAX				18000.0	// The motor speed limit (Upper)   6000rpm*3=1.8w

#undef  MCCONF_L_RPM_MIN
#define MCCONF_L_RPM_MIN				-18000.0	// The motor speed limit (Lower)

#undef  MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV				    30000.0 

#undef  MCCONF_FOC_PLL_KP
#define MCCONF_FOC_PLL_KP				2000.0

#undef  MCCONF_FOC_PLL_KI
#define MCCONF_FOC_PLL_KI				30000.0

#undef  MCCONF_FOC_MOTOR_L
#define MCCONF_FOC_MOTOR_L				0.00005877

#undef  MCCONF_FOC_MOTOR_R
#define MCCONF_FOC_MOTOR_R				0.0366

#undef  MCCONF_FOC_MOTOR_FLUX_LINKAGE
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE	0.016365

#undef  MCCONF_FOC_MOTOR_LD_LQ_DIFF
#define MCCONF_FOC_MOTOR_LD_LQ_DIFF		0.00003296

#undef  MCCONF_FOC_OBSERVER_GAIN
#define MCCONF_FOC_OBSERVER_GAIN		3.73e6		// Can be something like 600 / L
 
#undef  MCCONF_FOC_OBSERVER_GAIN_SLOW
#define MCCONF_FOC_OBSERVER_GAIN_SLOW	0.05	// Observer gain scale at minimum duty cycle
 
#undef  MCCONF_FOC_OBSERVER_OFFSET
#define MCCONF_FOC_OBSERVER_OFFSET		-1.0	// Observer offset in timer update cycles
 
#undef  MCCONF_FOC_DUTY_DOWNRAMP_KP
#define MCCONF_FOC_DUTY_DOWNRAMP_KP		50.0	// PI controller for duty control when decreasing the duty
 
#undef  MCCONF_FOC_DUTY_DOWNRAMP_KI
#define MCCONF_FOC_DUTY_DOWNRAMP_KI		1000.0	// PI controller for duty control when decreasing the duty
 
#undef  MCCONF_FOC_START_CURR_DEC
#define MCCONF_FOC_START_CURR_DEC		1.0	// Decrease current to this fraction at start
 
#undef  MCCONF_FOC_START_CURR_DEC_RPM
#define MCCONF_FOC_START_CURR_DEC_RPM	2500.0	// At this RPM the full current is available
 
#undef  MCCONF_FOC_OPENLOOP_RPM
#define MCCONF_FOC_OPENLOOP_RPM			1500.0	// Openloop RPM (sensorless low speed or when finding index pulse)
 
#undef  MCCONF_FOC_OPENLOOP_RPM_LOW
#define MCCONF_FOC_OPENLOOP_RPM_LOW		0.09		// Fraction of OPENLOOP_RPM at minimum motor current
 
#undef  MCCONF_FOC_D_GAIN_SCALE_START
#define MCCONF_FOC_D_GAIN_SCALE_START	0.9		// Start reducing D axis current controller gain at this modulation
 
#undef  MCCONF_FOC_D_GAIN_SCALE_MAX_MOD
#define MCCONF_FOC_D_GAIN_SCALE_MAX_MOD	0.9		// D axis current controller gain at maximum modulation
 
#undef  MCCONF_FOC_SL_OPENLOOP_HYST
#define MCCONF_FOC_SL_OPENLOOP_HYST		0.1		// Time below min RPM to activate openloop (s)
 
#undef  MCCONF_FOC_SL_OPENLOOP_TIME
#define MCCONF_FOC_SL_OPENLOOP_TIME		0.1	// Time to remain in openloop after ramping (s)
 
#undef  MCCONF_FOC_SL_OPENLOOP_BOOST_Q
#define MCCONF_FOC_SL_OPENLOOP_BOOST_Q	0.0		// Q-axis current boost during the open loop procedure
 
#undef  MCCONF_FOC_SL_OPENLOOP_MAX_Q
#define MCCONF_FOC_SL_OPENLOOP_MAX_Q	-1.0		// Q-axis maximum current during the open loop procedure
 
#undef  MCCONF_FOC_SL_OPENLOOP_T_LOCK
#define MCCONF_FOC_SL_OPENLOOP_T_LOCK	0.1		// Time to lock motor in beginning of open loop sequence
 
#undef  MCCONF_FOC_SL_OPENLOOP_T_RAMP
#define MCCONF_FOC_SL_OPENLOOP_T_RAMP	0.1		// Time to ramp up motor to openloop speed
 
#undef  MCCONF_FOC_OFFSETS_CURRENT_0
#define MCCONF_FOC_OFFSETS_CURRENT_0	2042.28 // Current 0 offset

#undef  MCCONF_FOC_OFFSETS_CURRENT_1
#define MCCONF_FOC_OFFSETS_CURRENT_1	2041.77 // Current 1 offset

#undef  MCCONF_FOC_OFFSETS_CURRENT_2
#define MCCONF_FOC_OFFSETS_CURRENT_2	2045.4 // Current 2 offset


// Optional: For no_limits variant, add #define HW_NO_LIMITS in conf/general_conf.h
//可选：对于no_limits变量，在conf/general_conf.h中添加#define HW_no_limits
// HW-specific functions
bool hw_sample_shutdown_button(void);

#endif /* HW_IKESC_6kw_LEFT_H_ */
