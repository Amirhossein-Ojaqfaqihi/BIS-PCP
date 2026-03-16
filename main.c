/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <math.h>
#include "arm_const_structs.h"
#include <stdio.h>
#include <stdint.h>
#include "string.h"
#include "goertzel.h"
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// EMG Digital Potentiometer Structure
typedef struct {
    GPIO_TypeDef* mode_port;
    uint16_t mode_pin;
    GPIO_TypeDef* dacsel_port;
    uint16_t dacsel_pin;
    GPIO_TypeDef* ud_port;
    uint16_t ud_pin;
    GPIO_TypeDef* clk_port;
    uint16_t clk_pin;
    uint8_t position;
} EMG_DigitalPot_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EMG_LENGTH 1024
#define NUM_CHANNELS 4
#define Full_Data EMG_LENGTH*NUM_CHANNELS
uint16_t adc_buffer [NUM_CHANNELS * EMG_LENGTH];


#define SIGNAL_LENGTH 1024
#define SIZE_F 10

uint32_t Excitation[SIGNAL_LENGTH] = {2362, 2329, 2442, 2667, 2919, 3090, 3089, 2878, 2493, 2038, 1655, 1475, 1578, 1959, 2533,
		3156, 3672, 3961, 3971, 3730, 3325, 2873, 2475, 2185, 2001, 1876, 1747, 1572, 1346, 1112,
		942, 907, 1043, 1336, 1718, 2092, 2368, 2494, 2478, 2387, 2323, 2381, 2612, 2998, 3454,
		3854, 4074, 4034, 3731, 3242, 2700, 2250, 2007, 2016, 2243, 2588, 2926, 3143, 3177, 3031,
		2761, 2449, 2168, 1957, 1805, 1668, 1494, 1251, 953, 658, 448, 399, 546, 864, 1263,
		1620, 1813, 1769, 1486, 1039, 556, 179, 13, 95, 382, 770, 1131, 1359, 1407, 1300,
		1124, 990, 996, 1188, 1548, 2001, 2443, 2781, 2962, 2986, 2901, 2773, 2657, 2570, 2486,
		2350, 2103, 1720, 1230, 718, 301, 95, 168, 515, 1056, 1650, 2142, 2412, 2410, 2169,
		1796, 1426, 1184, 1140, 1291, 1564, 1854, 2058, 2113, 2017, 1823, 1613, 1462, 1411, 1447,
		1516, 1544, 1474, 1289, 1025, 758, 573, 533, 647, 867, 1103, 1253, 1251, 1089, 829,
		588, 494, 645, 1071, 1717, 2459, 3137, 3610, 3794, 3684, 3356, 2930, 2534, 2262, 2147,
		2164, 2248, 2331, 2369, 2365, 2361, 2415, 2576, 2849, 3192, 3520, 3739, 3780, 3625, 3325,
		2978, 2699, 2581, 2658, 2890, 3178, 3399, 3449, 3283, 2932, 2499, 2119, 1920, 1976, 2282,
		2763, 3293, 3742, 4016, 4086, 3988, 3803, 3622, 3513, 3495, 3540, 3592, 3593, 3517, 3382,
		3244, 3172, 3216, 3380, 3607, 3795, 3827, 3613, 3127, 2421, 1616, 865, 308, 29, 32,
		241, 531, 774, 880, 826, 660, 478, 383, 448, 685, 1047, 1440, 1763, 1942, 1954,
		1834, 1652, 1484, 1381, 1347, 1344, 1309, 1191, 975, 702, 458, 345, 447, 789, 1324,
		1937, 2482, 2823, 2883, 2666, 2258, 1799, 1438, 1283, 1375, 1673, 2078, 2470, 2750, 2870,
		2847, 2744, 2640, 2597, 2632, 2715, 2782, 2767, 2631, 2386, 2090, 1830, 1689, 1706, 1867,
		2097, 2296, 2367, 2261, 1996, 1653, 1350, 1204, 1279, 1567, 1984, 2395, 2656, 2667, 2399,
		1905, 1306, 744, 345, 177, 241, 478, 800, 1125, 1410, 1656, 1901, 2192, 2552, 2967,
		3375, 3690, 3832, 3764, 3506, 3138, 2778, 2536, 2481, 2612, 2857, 3096, 3206, 3102, 2769,
		2269, 1723, 1266, 1007, 986, 1168, 1456, 1724, 1866, 1828, 1622, 1323, 1030, 836, 791,
		889, 1079, 1287, 1453, 1554, 1610, 1676, 1806, 2031, 2329, 2626, 2821, 2818, 2564, 2080,
		1458, 841, 382, 191, 305, 676, 1186, 1689, 2057, 2219, 2180, 2012, 1823, 1712, 1736,
		1889, 2108, 2305, 2398, 2345, 2162, 1911, 1677, 1532, 1508, 1587, 1709, 1803, 1818, 1747,
		1638, 1574, 1644, 1901, 2334, 2867, 3374, 3719, 3804, 3600, 3163, 2618, 2120, 1804, 1747,
		1942, 2307, 2717, 3048, 3212, 3191, 3025, 2796, 2589, 2462, 2421, 2431, 2431, 2372, 2240,
		2067, 1926, 1897, 2040, 2360, 2803, 3266, 3635, 3819, 3790, 3585, 3302, 3059, 2953, 3020,
		3224, 3461, 3600, 3526, 3183, 2592, 1853, 1108, 499, 126, 17, 132, 380, 657, 885,
		1034, 1120, 1194, 1304, 1475, 1684, 1874, 1976, 1939, 1758, 1488, 1227, 1089, 1161, 1467,
		1961, 2531, 3038, 3359, 3431, 3270, 2963, 2643, 2437, 2427, 2621, 2954, 3315, 3586, 3683,
		3586, 3344, 3046, 2796, 2668, 2682, 2804, 2962, 3076, 3094, 3011, 2865, 2719, 2633, 2626,
		2670, 2695, 2611, 2354, 1910, 1338, 754, 299, 98, 212, 619, 1216, 1850, 2363, 2643,
		2653, 2433, 2086, 1731, 1466, 1333, 1310, 1330, 1309, 1186, 949, 637, 326, 96, 0,
		45, 183, 340, 441, 447, 371, 276, 251, 372, 672, 1118, 1616, 2041, 2277, 2262,
		2011, 1613, 1206, 934, 895, 1112, 1526, 2018, 2445, 2691, 2697, 2480, 2115, 1705, 1341,
		1076, 910, 803, 702, 570, 411, 272, 224, 331, 623, 1071, 1590, 2066, 2389, 2498,
		2397, 2162, 1909, 1761, 1797, 2029, 2392, 2771, 3041, 3109, 2952, 2624, 2239, 1933, 1817,
		1942, 2284, 2759, 3249, 3647, 3889, 3968, 3928, 3835, 3748, 3690, 3640, 3550, 3365, 3063,
		2667, 2251, 1920, 1770, 1853, 2155, 2590, 3027, 3330, 3404, 3227, 2860, 2424, 2065, 1898,
		1980, 2284, 2719, 3160, 3495, 3660, 3659, 3553, 3436, 3392, 3464, 3640, 3858, 4034, 4094,
		4007, 3794, 3515, 3249, 3052, 2941, 2878, 2795, 2616, 2300, 1863, 1383, 982, 781, 864,
		1238, 1829, 2496, 3076, 3430, 3483, 3247, 2811, 2307, 1862, 1564, 1432, 1424, 1456, 1447,
		1344, 1149, 909, 697, 575, 570, 658, 775, 842, 799, 632, 384, 142, 0, 30,
		244, 591, 967, 1254, 1361, 1260, 998, 688, 464, 446, 687, 1161, 1771, 2373, 2833,
		3060, 3037, 2821, 2515, 2231, 2055, 2016, 2092, 2224, 2349, 2431, 2474, 2520, 2627, 2834,
		3139, 3485, 3776, 3903, 3790, 3419, 2844, 2181, 1571, 1134, 932, 949, 1099, 1259, 1313,
		1194, 910, 539, 202, 18, 65, 351, 811, 1335, 1803, 2125, 2266, 2253, 2155, 2048,
		1988, 1987, 2012, 2008, 1923, 1744, 1505, 1280, 1159, 1210, 1448, 1820, 2217, 2509, 2584,
		2393, 1967, 1412, 880, 516, 417, 601, 1006, 1513, 1990, 2332, 2497, 2511, 2450, 2403,
		2440, 2579, 2786, 2988, 3106, 3088, 2933, 2689, 2435, 2249, 2178, 2217, 2312, 2381, 2355,
		2206, 1964, 1716, 1573, 1630, 1930, 2437, 3044, 3601, 3962, 4031, 3793, 3317, 2734, 2196,
		1825, 1682, 1752, 1962, 2212, 2412, 2518, 2539, 2526, 2541, 2629, 2793, 2991, 3150, 3200,
		3105, 2880, 2593, 2342, 2216, 2264, 2468, 2749, 2990, 3079, 2947, 2599, 2111, 1608, 1222,
		1047, 1104, 1337, 1631, 1852, 1896, 1721, 1361, 912, 497, 228, 167, 312, 610, 979,
		1344, 1667, 1953, 2238, 2566, 2956, 3382, 3776, 4040, 4090, 3884, 3450, 2880, 2311, 1879,
		1678, 1730, 1973, 2289, 2543, 2627, 2498, 2194, 1811, 1474, 1289, 1307, 1506, 1805, 2092,
		2267, 2274, 2121, 1869, 1606, 1411, 1326, 1341, 1408, 1459, 1445, 1360, 1243, 1167, 1205,
		1401, 1737, 2138, 2488, 2672, 2614, 2312, 1842, 1339, 957, 819, 979, 1403, 1987, 2584,
		3061, 3331, 3382, 3267, 3080, 2916, 2835, 2845, 2904, 2943, 2902, 2757, 2534, 2301, 2139,
		2113, 2241, 2485, 2766, 2990, 3089, 3046, 2905, 2756, 2700, 2807, 3085, 3471, 3841, 4058,
		4008, 3648, 3017, 2231, 1445, 806, 408, 271, 341, 513, 676, 750, 706, 575, 421,
		310, 283, 334, 417, 470, 441, 322, 154, 17, 3, 179, 551, 1063, 1604, 2042,
		2272, 2251, 2011, 1652, 1306, 1093, 1078, 1250, 1527, 1789, 1916, 1838, 1554, 1135, 698,
		364, 222, 298, 554, 906, 1262, 1551, 1748, 1874, 1980, 2113, 2289, 2481, 2627, 2649,
		2495, 2165, 1720, 1278, 970, 908, 1136, 1619, 2249, 2877, 3362, 3611, 3614, 3435, 3191,
		3007, 2972, 3111, 3378, 3679, 3907, 3985, 3889, 3657, 3368, 3112, 2953, 2910, 2947, 2997,
		2994, 2898, 2720, 2516};
int f_freqbins_map[SIZE_F] = {5, 7, 10, 14, 19, 27, 37, 52, 73, 102};


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// EMG Digital Potentiometers - Based on your exact pin mapping
EMG_DigitalPot_t emg_pots[4] = {
    {GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, 64},
    {GPIOC, GPIO_PIN_6, GPIOC, GPIO_PIN_7, GPIOC, GPIO_PIN_8, GPIOC, GPIO_PIN_9, 64},
    {GPIOA, GPIO_PIN_8, GPIOA, GPIO_PIN_9, GPIOA, GPIO_PIN_10, GPIOA, GPIO_PIN_11, 64},
    // Fix EMG4:
    {GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7, GPIOC, GPIO_PIN_4, GPIOC, GPIO_PIN_5, 64}
};

uint32_t Emg1Buffer1[EMG_LENGTH] = {0};
uint32_t Emg2Buffer1[EMG_LENGTH] = {0};
uint32_t Emg3Buffer1[EMG_LENGTH] = {0};
uint32_t Emg4Buffer1[EMG_LENGTH] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
// Function prototypes
void EMG_Init(void);
void EMG_SetPosition(uint8_t channel, uint8_t position);
void EMG_MoveSteps(uint8_t channel, int16_t steps);
void EMG_SetGainLevel(uint8_t channel, uint8_t gain_level);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void serialPrint(UART_HandleTypeDef *uart, uint8_t *message)
{
  HAL_UART_Transmit(uart, (uint8_t *)message, strlen(message), 500);
}

uint16_t AdcBuffer1[SIGNAL_LENGTH] = { 0 };
uint16_t AdcBuffer2[SIGNAL_LENGTH] = { 0 };
float32_t accPhase[SIZE_F];
float32_t accMagnitude[SIZE_F];
float32_t accFactor = 0.99;

 uint8_t DmaCpltFlag_BIS = 0;
 uint8_t DmaCpltFlag_EMG = 0;
char msg[100];
// Simple microsecond delay (adjust for your clock speed)
void delay_us(uint32_t us) {
    volatile uint32_t count = us * 20; // Adjust this multiplier for your system
    while(count--);
}

// Initialize all EMG digital potentiometers
void EMG_Init(void) {
	for(int i = 0; i < 4; i++) { // All 4 channels (EMG1, EMG2, EMG3, EMG4)
        // MODE = 0 (both wipers move together)
        HAL_GPIO_WritePin(emg_pots[i].mode_port, emg_pots[i].mode_pin, GPIO_PIN_RESET);

        // DACSEL = 0 (select first wiper)
        HAL_GPIO_WritePin(emg_pots[i].dacsel_port, emg_pots[i].dacsel_pin, GPIO_PIN_RESET);

        // U/D = 0 (direction down initially)
        HAL_GPIO_WritePin(emg_pots[i].ud_port, emg_pots[i].ud_pin, GPIO_PIN_RESET);

        // CLK = 0 (CRITICAL: Keep LOW to prevent changes since CS is tied to GND)
        HAL_GPIO_WritePin(emg_pots[i].clk_port, emg_pots[i].clk_pin, GPIO_PIN_RESET);

        // Track mid-scale position (AD5222 powers up at position 64)
        emg_pots[i].position = 64;
    }
    HAL_Delay(50);
}

// Send one clock pulse (very carefully since CS is always active)
static void EMG_ClockPulse(uint8_t channel) {
    if(channel >= 4) return;

    __disable_irq(); // Critical section

    // Ensure starting from LOW
    HAL_GPIO_WritePin(emg_pots[channel].clk_port, emg_pots[channel].clk_pin, GPIO_PIN_RESET);
    delay_us(10);

    // Pulse HIGH then LOW (falling edge triggers)
    HAL_GPIO_WritePin(emg_pots[channel].clk_port, emg_pots[channel].clk_pin, GPIO_PIN_SET);
    delay_us(50);
    HAL_GPIO_WritePin(emg_pots[channel].clk_port, emg_pots[channel].clk_pin, GPIO_PIN_RESET);
    delay_us(50);

    __enable_irq();
}

// Move by number of steps (+up, -down)
void EMG_MoveSteps(uint8_t channel, int16_t steps) {
    if(channel >= 4 || steps == 0) return;

    // Calculate new position with bounds checking
    int16_t new_pos = emg_pots[channel].position + steps;
    if(new_pos < 0) {
        steps = -emg_pots[channel].position;
        new_pos = 0;
    }
    if(new_pos > 127) {
        steps = 127 - emg_pots[channel].position;
        new_pos = 127;
    }

    if(steps == 0) return;

    // Set direction
    if(steps > 0) {
        HAL_GPIO_WritePin(emg_pots[channel].ud_port, emg_pots[channel].ud_pin, GPIO_PIN_SET); // UP
    } else {
        HAL_GPIO_WritePin(emg_pots[channel].ud_port, emg_pots[channel].ud_pin, GPIO_PIN_RESET); // DOWN
        steps = -steps; // Make positive for loop
    }

    delay_us(20); // Setup time

    // Send clock pulses
    for(int i = 0; i < steps; i++) {
        EMG_ClockPulse(channel);
    }

    emg_pots[channel].position = new_pos;

    // Ensure ALL clocks stay LOW
    for(int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(emg_pots[i].clk_port, emg_pots[i].clk_pin, GPIO_PIN_RESET);
    }
}

// Set absolute position (0-127)
void EMG_SetPosition(uint8_t channel, uint8_t position) {
    if(channel >= 4 || position > 127) return;

    int16_t steps = position - emg_pots[channel].position;
    EMG_MoveSteps(channel, steps);
}

// Set predefined gain levels
void EMG_SetGainLevel(uint8_t channel, uint8_t gain_level) {
    uint8_t positions[] = {10, 30, 64, 100}; // Max, High, Medium, Low gain

    if(channel >= 4 || gain_level >= 4) return;

    EMG_SetPosition(channel, positions[gain_level]);
}

// Get current resistance W-to-B in ohms
uint16_t EMG_GetResistance(uint8_t channel) {
    if(channel >= 4) return 0;
    return (emg_pots[channel].position * 78); // Each step ≈ 78Ω for 10kΩ pot
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  #define FFT
//	#define GOERTZEL
	//	#define SIGNAL_OUT
	 //#define EMG_SIGNAL_OUT
	//#define EMG
	#define SIGNAL_OUT_SERIALPLOT_STYLE
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  	EMG_SetGainLevel(0, 3); // EMG1 - medium gain
    EMG_SetGainLevel(1, 3); // EMG2 - medium gain
    EMG_SetGainLevel(2, 3); // EMG3 - medium gain
    EMG_SetGainLevel(3, 3); // EMG4 - medium gain


//	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
  //  HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc_buffer, Full_Data);
    //HAL_TIM_Base_Start(&htim3);

	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, Excitation, SIGNAL_LENGTH,DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(&htim4);


	// Calibrate ADCs bio
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

	// Start DMA & ADC 1/2 bio
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) AdcBuffer1, SIGNAL_LENGTH);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) AdcBuffer2, SIGNAL_LENGTH);
	HAL_TIM_Base_Start(&htim2);


////	  	 Start DMA / DAC 3 bio
//	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, Excitation, SIGNAL_LENGTH,DAC_ALIGN_12B_R);
//	HAL_TIM_Base_Start(&htim4);
//
//
//	// Calibrate ADCs bio
//		HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//		HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
//
//	// Start DMA & ADC 1/2 bio
//		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) AdcBuffer1, SIGNAL_LENGTH);
//		HAL_ADC_Start_DMA(&hadc2, (uint32_t*) AdcBuffer2, SIGNAL_LENGTH);
//		HAL_TIM_Base_Start(&htim2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//      if (DmaCpltFlag_EMG == 1 && DmaCpltFlag_BIS==1) {
//          DmaCpltFlag_EMG = 0;
//          DmaCpltFlag_BIS = 0;
	   //HAL_TIM_Base_Stop(&htim2);
	    if (DmaCpltFlag_BIS == 1) {
	    	DmaCpltFlag_BIS = 0;
	    	HAL_TIM_Base_Stop(&htim2);


	    	////////////////////////////////////////////////////// Out signal /////////////////////////////////
	    	#ifdef SIGNAL_OUT
	    	      char message[50];
	    	      int cnt = 0;

	    	      serialPrint(&huart2, "Signal writing...\r\n");
	    	      for (cnt = 0; cnt < SIGNAL_LENGTH; cnt++)
	    	      {

	    	#ifdef SIGNAL_OUT_SERIALPLOT_STYLE
	    	        sprintf(message, "%d,%d,%d\r\n",cnt, AdcBuffer1[cnt], AdcBuffer2[cnt]); //, Emg1Buffer1[cnt], Emg2Buffer1[cnt], Emg3Buffer1[cnt], Emg4Buffer1[cnt] /, %u, %u, %u, %u
	    	        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 500);

	    	#else
	    	        sprintf(message, "ADC1(%4d)=%4d;\r\n", cnt + 1, AdcBuffer1[cnt]);
	    	        HAL_UART_Transmit(&huart2, (uint8_t *)message, 18, 500);

	    	        sprintf(message, "ADC2(%4d)=%4d;\r\n", cnt + 1, AdcBuffer2[cnt]);
	    	        HAL_UART_Transmit(&huart2, (uint8_t *)message, 18, 500);

	    	#endif

	    	        //

	    	        //	  	  		 	HAL_Delay(10);
	    	      }
	    	      serialPrint(&huart2, "Signal writing done\r\n");
	    	      HAL_TIM_Base_Start(&htim2);
	    	#endif


//	    }
#ifdef GOERTZEL

      uint8_t message[50];

      float32_t Magnitude[SIZE_F];
      float32_t Phase[SIZE_F];
      goertzel_struct g1[SIZE_F];
      goertzel_struct_singlesideres gres1[SIZE_F];
      goertzel_struct g2[SIZE_F];
      goertzel_struct_singlesideres gres2[SIZE_F];

	  for(int k = 0; k< SIZE_F; k++) {
		  goertzel_init2(f_freqbins_map[k], SIGNAL_LENGTH, &g1[k]);
		  goertzel_init2(f_freqbins_map[k], SIGNAL_LENGTH, &g2[k]);

		  for(int i=0; i<SIGNAL_LENGTH; i++) {
			  goertzel(&g1[k], (double)AdcBuffer1[i]);
			    goertzel(&g2[k], (double)AdcBuffer2[i]);

			  //goertzel(&g1[k], (double)AdcBuffer1[i]/ACQ_LENGTH * powf(arm_sin_f32(pi*(float)i/ACQ_LENGTH),2));
			  //goertzel(&g2[k], (double)AdcBuffer2[i]/FFT_LENGTH * powf(arm_sin_f32(pi*(float)i/ACQ_LENGTH),2));
			//  goertzel(&g1[k], (float)AdcBuffer1[i]/ACQ_LENGTH * powf(arm_sin_f32(pi*(float)i/ACQ_LENGTH),2));
			//  goertzel(&g2[k], (float)AdcBuffer2[i]/ACQ_LENGTH * powf(arm_sin_f32(pi*(float)i/ACQ_LENGTH),2));

		  }

		  goertzel_end(&g1[k], &gres1[k], SIGNAL_LENGTH);
		  goertzel_end(&g2[k], &gres2[k], SIGNAL_LENGTH);

		  Magnitude[k] = 1000.0 *gres1[k].M / gres2[k].M;
//		 Phase[k] =  fmod((float)((gres1[k].Phi-gres2[k].Phi)*(180/PI))+180 - 360, 360);  //+90;
	  Phase[k] =  fmod((float)((gres1[k].Phi-gres2[k].Phi)*(180/PI))+360-360, 360);  //+90;



		if (accPhase[k] == 0) accPhase[k] = Phase[k];
		if (accMagnitude[k] == 0) accMagnitude[k] = Magnitude[k];
		accPhase[k] = accPhase[k] * accFactor + Phase[k] * (1-accFactor);
		accMagnitude[k] = accMagnitude[k] * accFactor + Magnitude[k] * (1-accFactor);
	}

		      for (int cnt = 0; cnt < SIZE_F; cnt++) {

		#ifdef SIGNAL_OUT_SERIALPLOT_STYLE
		    	  //Magnitude, Phase, accumulated magnitude, accumulated phase
		        sprintf(message, "%d, %d.%d,%d.%d,%d.%d,%d.%d\r\n", cnt+1, (int)(Magnitude[cnt]),  (int)((fabs(Magnitude[cnt]) - (int)fabs(Magnitude[cnt])) * 1000),
		        		(int)Phase[cnt], (int)((fabs(Phase[cnt]) - (int)fabs(Phase[cnt])) * 1000),
						(int)(accMagnitude[cnt]),  (int)((fabs(accMagnitude[cnt]) - (int)fabs(accMagnitude[cnt])) * 1000),
						(int)accPhase[cnt], (int)((fabs(accPhase[cnt]) - (int)fabs(accPhase[cnt])) * 1000)

		        );
		        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 500);

		#else
		        // Magnitude, Phase from UART
		        sprintf(message, "Phas(%02d)=%d.%d;\r\n", cnt + 1, (int)Phase[cnt], (int)((fabs(Phase[cnt]) - (int)fabs(Phase[cnt])) * 1000));
		        HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
		        sprintf(message, "Magn(%02d)=%d.%d;\r\n", cnt + 1, (int)Magnitude[cnt], (int)((fabs(Magnitude[cnt]) - (int)fabs(Magnitude[cnt])) * 1000));
		        HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
		        sprintf(message, "AccPhas(%02d)=%d.%d;\r\n", cnt + 1, (int)accPhase[cnt], (int)((fabs(accPhase[cnt]) - (int)fabs(accPhase[cnt])) * 1000));
			   HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
			   sprintf(message, "AccMagn(%02d)=%d.%d;\r\n", cnt + 1, (int)accMagnitude[cnt], (int)((fabs(accMagnitude[cnt]) - (int)fabs(accMagnitude[cnt])) * 1000));
			   HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
		#endif
		      }



#endif
				////////////////////////////////////////////////////////// FFT ///////////////////////////////////////////////////////
		#ifdef FFT

					      uint8_t message[500];

					      float32_t Magnitude[SIZE_F];
					      float32_t Magnitude1[SIZE_F];
					      float32_t Magnitude2[SIZE_F];
					      float32_t Phase[SIZE_F];
					      float32_t Theta1[SIZE_F];
					      float32_t Theta2[SIZE_F];
					      float32_t in1;
					      float32_t in2;
					      float32_t input1;
						  float32_t input2;
						  int idxtest;


					     float32_t fft_inputbuf[SIGNAL_LENGTH * 2];
					     // int16_t fft_inputbuf[SIGNAL_LENGTH * 2];
					      for (int i = 0; i < SIGNAL_LENGTH; i++)
					      {

					        fft_inputbuf[i * 2] = AdcBuffer1[i];
					        fft_inputbuf[i * 2 + 1] = 0;
					      }
					      arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf, 0, 1);
					      /*
					      if(SIGNAL_LENGTH==2048)       arm_cfft_f32(&arm_cfft_sR_f32_len2048, fft_inputbuf, 0, 1);
					      else if(SIGNAL_LENGTH == 4096) arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_inputbuf, 0, 1);
		*/
					  //     if(ACQ_LENGTH==2048)       arm_cfft_q15(&arm_cfft_sR_q15_len2048, fft_inputbuf, 0, 1);
					    //     else if(ACQ_LENGTH == 4096) arm_cfft_q15(&arm_cfft_sR_q15_len4096, fft_inputbuf, 0, 1);


					      for (int idx = 0; idx < SIZE_F; idx++)
					      			      {

					         in1 = fft_inputbuf[f_freqbins_map[idx] * 2];
					         in2 = fft_inputbuf[f_freqbins_map[idx] * 2 + 1];
					        Theta1[idx] = atan2(in2, in1);

					         input1 = fft_inputbuf[f_freqbins_map[idx] * 2];
					         input2 = fft_inputbuf[f_freqbins_map[idx] * 2 + 1];
					        Magnitude1[idx] = (float)sqrt((input1 * input1) + (input2 * input2));

					      			      }

					      // FFT_ADC2_ START
					      for (int i = 0; i < SIGNAL_LENGTH; i++)
					      {

					        fft_inputbuf[i * 2] = AdcBuffer2[i];
					        fft_inputbuf[i * 2 + 1] = 0;
					      }

					      arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf, 0, 1);

					//      if(SIGNAL_LENGTH==2048) arm_cfft_f32(&arm_cfft_sR_f32_len2048, fft_inputbuf, 0, 1);
					   //   else if(SIGNAL_LENGTH == 4096) arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_inputbuf, 0, 1);

					      //     if(ACQ_LENGTH==2048)       arm_cfft_q15(&arm_cfft_sR_q15_len2048, fft_inputbuf, 0, 1);
					        //     else if(ACQ_LENGTH == 4096) arm_cfft_q15(&arm_cfft_sR_q15_len4096, fft_inputbuf, 0, 1);


					      for (int idx = 0; idx < SIZE_F; idx++)
					      			      {
					        in1 = fft_inputbuf[f_freqbins_map[idx] * 2];
					        in2 = fft_inputbuf[f_freqbins_map[idx] * 2 + 1];
					        Theta2[idx] = atan2(in2, in1);

					        input1 = fft_inputbuf[f_freqbins_map[idx] * 2];
					        input2 = fft_inputbuf[f_freqbins_map[idx] * 2 + 1];

					        Magnitude2[idx] = (float)sqrt((input1 * input1) + (input2 * input2));

					        Phase[idx] = (float)((Theta1[idx] - Theta2[idx]) * (180 / PI)) + 180; //+90;
					        Phase[idx] = fmod(Phase[idx] - 360, 360);
					        Magnitude[idx] = (float)(Magnitude1[idx] / Magnitude2[idx]) * 1000;
					      			      }


					      for (int cnt = 0; cnt < SIZE_F; cnt++) {
					     		#ifdef SIGNAL_OUT_SERIALPLOT_STYLE
					     		    	//Magnitude, Phase, accumulated magnitude, accumulated phase
					     		        sprintf(message, "%d, %d.%d, %d.%d, %d.%d, %d.%d \r\n",
					     		        		cnt+1,
												(int)(Magnitude[cnt]),  (int)((fabs(Magnitude[cnt]) - (int)fabs(Magnitude[cnt])) * 1000),
					     		        		(int)Phase[cnt], (int)((fabs(Phase[cnt]) - (int)fabs(Phase[cnt])) * 1000),
					     						(int)(accMagnitude[cnt]),  (int)((fabs(accMagnitude[cnt]) - (int)fabs(accMagnitude[cnt])) * 1000),
					     						(int)accPhase[cnt], (int)((fabs(accPhase[cnt]) - (int)fabs(accPhase[cnt])) * 1000)
					     		        );

					     		        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 500);

					     		#else
					     		        // Magnitude, Phase from UART
					     		        sprintf(message, "Phas(%02d)=%d.%d;\r\n", cnt + 1, (int)Phase[cnt], (int)((fabs(Phase[cnt]) - (int)fabs(Phase[cnt])) * 1000));
					     		        HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
					     		        sprintf(message, "Magn(%02d)=%d.%d;\r\n", cnt + 1, (int)Magnitude[cnt], (int)((fabs(Magnitude[cnt]) - (int)fabs(Magnitude[cnt])) * 1000));
					     		        HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
					     		        sprintf(message, "AccPhas(%02d)=%d.%d;\r\n", cnt + 1, (int)accPhase[cnt], (int)((fabs(accPhase[cnt]) - (int)fabs(accPhase[cnt])) * 1000));
					     			   HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
					     			   sprintf(message, "AccMagn(%02d)=%d.%d;\r\n", cnt + 1, (int)accMagnitude[cnt], (int)((fabs(accMagnitude[cnt]) - (int)fabs(accMagnitude[cnt])) * 1000));
					     			   HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 500);
					     		#endif
					     		      }


					      HAL_TIM_Base_Start(&htim2);
		#endif

					  //    HAL_TIM_Base_Start(&htim2);
	    //}




	          // Handle EMG data

#ifdef EMG
					      HAL_TIM_Base_Stop(&htim3);
					      HAL_TIM_Base_Stop(&htim2);

					      for (int i = 0; i < EMG_LENGTH; ++i) {
					          int n = snprintf(msg, sizeof(msg), "%u,%u,%u,%u\r\n",
					                           adc_buffer[i*4 + 0],
					                           adc_buffer[i*4 + 1],
					                           adc_buffer[i*4 + 2],
					                           adc_buffer[i*4 + 3]);
					          HAL_UART_Transmit(&huart2, (uint8_t*)msg, n, 500);
					      }

					      HAL_TIM_Base_Start(&htim3);
					      HAL_TIM_Base_Start(&htim2);

#endif
	          }



	           // Restart timer for next acquisition
	         // HAL_Delay(1);
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 4;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 31;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 31;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 PA8 PA9
                           PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC7
                           PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//  DmaCpltFlag1 = 1;
//}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if(hadc->Instance == ADC1 || hadc->Instance == ADC2) {
        DmaCpltFlag_BIS = 1;
    }
    if(hadc->Instance == ADC1 || hadc->Instance == ADC2) {
        DmaCpltFlag_BIS = 1;
    } else if(hadc->Instance == ADC3) {
        DmaCpltFlag_EMG = 1;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
