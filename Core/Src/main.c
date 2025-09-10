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
#include <string.h> // For memcpy
#include "../Inc/jv_bt+packet_lib/jv_bt+packet.h"
#include "../Inc/jv_bt+packet_lib/jv_bt+bsc.h"
#include "../Inc/jv_bt+packet_lib/crc.h"

#define BLE_CHANNEL 37

#define BLE_PACKET_TYPE UNCODED_1MBPS
//#define BLE_PACKET_TYPE UNCODED_2MBPS
//#define BLE_PACKET_TYPE CODED_S2
//#define BLE_PACKET_TYPE CODED_S8

volatile uint8_t spiTxComplete = 0;

// Variables for BLE Packet Generation (mimicking different_project)
uint8_t AdvA[] = {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc}; // Example AdvA
//uint8_t AdvA[] = {0xab, 0xab, 0xab, 0xab, 0xab, 0xab};



uint8_t AdvData[] = {0xb5, 0xef, 0xbe, 0x01, 0x00, 0x0f, //Data
					0x6d, 0x16, 0xbb, 0xa9, 0xa2, 0x7c,
					0xa6, 0x80, 0x42, 0x60, 0x59, 0x20,
					0x01, 0x6c, 0x41, 0x15, 0x02,
					0x00, 0x4c, //Company ID (Apple, Inc.)
					0xff, //Type (Manufacturer Specific)
					0x1a, //length of (Data)+(CompanyID)+(Type) = 26
					0x04, //sets "BR/EDR Not Supported" to be true  	<-- ??idk what that term means??
						  	  	  //
					0x01, //Type (Flags) 								<-- ??idk what this means??
					0x02}; //length of flags = 2


uint8_t AdvData[30]; // Declared as a fixed-size array to hold dynamic data

jv_ble_pdu pdu;
jv_ble_packet packet;
uint32_t ble_tx_buffer[200]; // Buffer for upscaled data
uint32_t upscaled_length;    // Length of upscaled data in bytes
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_VREF_VOLTAGE 2.5f

#include <stdio.h> // For sprintf, if you want to print to console
//#include <string.h> // library already included above

#define STS40_I2C_ADDR (0x46 << 1) // STM32 HAL uses 8-bit address

#define DATA_BUFFER_SIZE 100
float temperature_buffer[DATA_BUFFER_SIZE];
float voltage_buffer[DATA_BUFFER_SIZE];     // Voltage buffer
uint16_t data_buffer_index = 0;             // Using one index for both buffers


// STS40 Measurement Command (High Precision)
const uint8_t cmd_measure_temp_high_precision[] = {0xFD}; // Example command byte

// STS40 Soft Reset Command
const uint8_t cmd_soft_reset[] = {0x94}; // Example command byte
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Calculates CRC-8 checksum for Sensirion sensors.
  * @param  data Pointer to the data array.
  * @param  len  Length of the data array (should be 2 for temperature).
  * @retval CRC-8 checksum.
  */
uint8_t sensirion_common_crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0xFF; // Initialization value
    const uint8_t polynomial = 0x31;

    for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}
/* USER CODE END 0 */

/* USER CODE BEGIN 4 */

/**
  * @brief  Performs a soft reset on the STS40 sensor.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  * the configuration information for the specified I2C.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef sts40_soft_reset(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(hi2c, STS40_I2C_ADDR, (uint8_t*)cmd_soft_reset, sizeof(cmd_soft_reset), HAL_MAX_DELAY);
    if (status == HAL_OK) {
        HAL_Delay(2); // Wait for reset to complete (typically ~1ms for STS4x)
    }
    return status;
}

/**
  * @brief  Reads temperature from the STS40 sensor.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  * the configuration information for the specified I2C.
  * @param  temperature Pointer to a float where the temperature in Celsius will be stored.
  * @retval HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise (includes CRC mismatch).
  */
HAL_StatusTypeDef sts40_read_temperature(I2C_HandleTypeDef *hi2c, float *temperature) {
    HAL_StatusTypeDef status;
    uint8_t i2c_data[3]; // 2 bytes for temp, 1 byte for CRC

    // 1. Send Measurement Command
    status = HAL_I2C_Master_Transmit(hi2c, STS40_I2C_ADDR, (uint8_t*)cmd_measure_temp_high_precision, sizeof(cmd_measure_temp_high_precision), HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // 2. Wait for measurement to complete (High precision typically ~8.5 ms, check datasheet)
    // For STS40, typical measurement duration for high precision is 8.5ms.
    HAL_Delay(10); // Wait 10ms

    // 3. Read Temperature Data (2 bytes) and CRC (1 byte)
    status = HAL_I2C_Master_Receive(hi2c, STS40_I2C_ADDR, i2c_data, 3, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // 4. CRC Check
    uint8_t received_crc = i2c_data[2];
    uint8_t calculated_crc = sensirion_common_crc8(i2c_data, 2); // CRC is calculated over the two data bytes

    if (received_crc != calculated_crc) {
        return HAL_ERROR; // CRC Mismatch
    }

    // 5. Convert Raw Data to Temperature
    // Temperature raw data is MSB first
    uint16_t raw_temp_ticks = ((uint16_t)i2c_data[0] << 8) | i2c_data[1];

    // Formula for STS4x: Temperature_C = -45 + 175 * (raw_temp_ticks / (2^16 - 1))
    // Or, if the sensor output is scaled ticks: Temperature_C = (raw_temp_ticks * 175 / 65535.0) - 45;
    // A common Sensirion scaling is also T = raw / 200.0 for some sensors,
    // or T_ticks / 65535 * 175 – 45
    // For STS4x specifically, the formula is: T [°C] = -45 + 175 * (S_T / (2^16 - 1))
    *temperature = -45.0f + 175.0f * (float)raw_temp_ticks / 65535.0f;

    return HAL_OK;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/*
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	spiTxComplete = 1;
}*/

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t raw_adc_value;
	float current_voltage;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //only transmitting a fixed number of dynamic BLE packets
      //HAL_Delay(7000);

  /*uint16_t transmit_num = 10000;
        while (1)
            {
      	  uint16_t i = 0;
      	  while (i < transmit_num) {
                float current_temperature;
                HAL_StatusTypeDef sensor_status = sts40_read_temperature(&hi2c2, &current_temperature);



                	// --- ADC Voltage Reading ---
          		HAL_ADC_Start(&hadc1); // Start ADC conversion
          		if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) // Poll for end of conversion
          		{
          			raw_adc_value = HAL_ADC_GetValue(&hadc1); // Get raw ADC value
          			current_voltage = (raw_adc_value / 4095.0f) * ADC_VREF_VOLTAGE; // Calculate voltage
          		}
          		else
          		{
          			// Handle ADC conversion error if necessary
          			current_voltage = -1.0f; // Indicate error or default value
          			// Optional: Print ADC error to UART
          			// char error_uart_buf[50];
          			// sprintf(error_uart_buf, "ADC Read Error\r\n");
          			// HAL_UART_Transmit(&huart2, (uint8_t*)error_uart_buf, strlen(error_uart_buf), HAL_MAX_DELAY);
          		}
          		HAL_ADC_Stop(&hadc1); // Stop ADC (optional if not much else is time critical right after)
          		// --- End ADC Voltage Reading ---



                if (sensor_status == HAL_OK) {
                    // Store temperature and voltage in buffer
          		  temperature_buffer[data_buffer_index] = current_temperature;
          		  voltage_buffer[data_buffer_index] = current_voltage;

          		  // Increment and manage buffer index
          			data_buffer_index++;
          			if (data_buffer_index >= DATA_BUFFER_SIZE) {
          				data_buffer_index = 0; // Reset index, overwrite oldest
          			}

          			// Print temperature and voltage via UART
          			//char uart_buf[100]; // Adjusted buffer size for more data
          			//sprintf(uart_buf, "Temp: %.2f C, Volt: %.2f V\r\n", current_temperature, current_voltage);
          			//HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);


                    // Optional: Print temperature (e.g., via UART)
                    //char uart_buf[50];
                    //sprintf(uart_buf, "Temperature: %.2f C\r\n", current_temperature);
                    //HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY); // Assuming huart1 is configured


          			// **** START OF ADDED CODE FOR AdvData AND BLE TRANSMISSION ****
      			  // Store current_temperature and current_voltage in AdvData array
      			  // Temperature (4 bytes) at AdvData[0-3]
      			  memcpy(&AdvData[0], &current_temperature, sizeof(float));

      			  // Delimiter (1 byte) at AdvData[4]
      			  AdvData[4] = 0x00; // Set delimiter byte to zero

      			  // Voltage (4 bytes) at AdvData[5-8]
      			  memcpy(&AdvData[5], &current_voltage, sizeof(float));

      			  // Fill the rest of AdvData with zeros (from index 9 to 29)
      			  // sizeof(AdvData) is 30, so 30 - 9 = 21 bytes to zero out
      			  memset(&AdvData[9], 0x00, sizeof(AdvData) - 9);

      			  // --- BLE Packet Generation ---
      			  // 1. Create PDU (re-create each time with updated AdvData)
      			  if (create_legacy_advertising_pdu(&pdu, AdvA, sizeof(AdvA), AdvData, sizeof(AdvData)) != 0) {
      				  Error_Handler(); // Handle error if PDU creation fails
      			  }

      			  // 2. Initialize Packet (includes whitening, CRC)
      			  if (init_packet(&packet, BLE_CHANNEL, &pdu, BLE_PACKET_TYPE) != 0) {
      				  Error_Handler(); // Handle error if packet init fails
      			  }

      			//memset(packet.whitened_packet, 0x00, packet.packet_len);
      			//memset(packet.whitened_packet, 0xFF, packet.packet_len);

      			  // 3. Upscale packet for backscatter transmission
      			  #if BLE_PACKET_TYPE == UNCODED_1MBPS
      				  upscaled_length = jv_bsc_upscale_1Mbps(ble_tx_buffer, packet.whitened_packet, packet.packet_len);
      			  #elif BLE_PACKET_TYPE == UNCODED_2MBPS
      				  upscaled_length = jv_bsc_upscale_2Mbps(ble_tx_buffer, packet.whitened_packet, packet.packet_len);
      			  #else
      				  #error "Unsupported BLE_PACKET_TYPE for jv_bsc_upscale"
      			  #endif

      			  //REVERSE BITS
      			  uint8_t *p_tx_buffer_byte = (uint8_t *)ble_tx_buffer;
    			  for (uint32_t k = 0; k < upscaled_length; k++) {
    				  p_tx_buffer_byte[k] = ~p_tx_buffer_byte[k]; // Apply bitwise NOT to each byte
    			  }

      			  // --- SPI DMA Transmission for a single packet ---
      			  spiTxComplete = 0; // Reset transmission complete flag

      			  // Transmit the upscaled packet buffer.
      			  if (HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)ble_tx_buffer, (uint16_t)upscaled_length) != HAL_OK)
      			  {
      				Error_Handler();
      			  }

      			  // Wait for DMA transmission to complete (flag set in HAL_SPI_TxCpltCallback)
      			  while (spiTxComplete == 0)
      			  {
      				 // You can add a small delay or WFE instruction here if needed
      			  }
      			  // **** END OF ADDED CODE ****


                } else if (sensor_status == HAL_ERROR) {
                    // Handle CRC error or other I2C communication error with sensor
              	  //char uart_buf[100]; // Adjusted buffer size
          		  // Also print the last known or error voltage if desired
          		  //sprintf(uart_buf, "Sensor Error, Last Volt: %.2f V\r\n", current_voltage);
          	      //HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

                    //char uart_buf[50];
                    //sprintf(uart_buf, "Sensor read error (CRC or I2C)\r\n");
                    //HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
                    // Consider a sensor reset or other recovery mechanism
                    // sts40_soft_reset(&hi2c2);
                    // HAL_Delay(100);
                } else {
                    // Handle other HAL I2C errors (e.g., NACK, Bus Busy)
              	  //char uart_buf[100]; // Adjusted buffer size
          		  //sprintf(uart_buf, "I2C Bus Error: %d, Last Volt: %.2f V\r\n", sensor_status, current_voltage);
          		  //HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

                    //char uart_buf[50];
                    //sprintf(uart_buf, "I2C Bus Error: %d\r\n", sensor_status);
                    //HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
                    // Error_Handler(); // Or attempt I2C recovery
                }

                i++;

                HAL_Delay(10); // Read temperature every 1 second

            }
      	  //HAL_Delay(10000);
         }*/



   //infinite loop
  /*while (1)
      {
    	  // 1. Create PDU
    	    if (create_legacy_advertising_pdu(&pdu, AdvA, sizeof(AdvA), AdvData, sizeof(AdvData)) != 0) {
    	        //pdu = [Pdu_Type, AdvA_len+AdvData_len, Advertising Address, Advertising Data]
    	    		//all sections are bit reversed

    	    	Error_Handler(); // Handle error if PDU creation fails
    	    }

    	    // 2. Initialize Packet (includes whitening, CRC)
    	    if (init_packet(&packet, BLE_CHANNEL, &pdu, BLE_PACKET_TYPE) != 0) {
    	    	//whitened_packet = [Preamble, Access Address]
    	    		//all sections are bit reversed

    	        Error_Handler(); // Handle error if packet init fails
    	    }

    	    //memset(packet.whitened_packet, 0xFF, packet.packet_len);   // all 1s
    	    //memset(packet.whitened_packet, 0x00, packet.packet_len); // all 0s

    	    // 3. Upscale packet for backscatter transmission
    	    // Ensure jv_bsc_upscale points to jv_bsc_upscale_1Mbps if using that encoding
    	    #if BLE_PACKET_TYPE == UNCODED_1MBPS
    	        upscaled_length = jv_bsc_upscale_1Mbps(ble_tx_buffer, packet.whitened_packet, packet.packet_len);
    	    #elif BLE_PACKET_TYPE == UNCODED_2MBPS
    	        upscaled_length = jv_bsc_upscale_2Mbps(ble_tx_buffer, packet.whitened_packet, packet.packet_len);
    	    #else
    	        #error "Unsupported BLE_PACKET_TYPE for jv_bsc_upscale"
    	        // Handle other encodings if necessary
    	    #endif


    	    // --- SPI DMA Transmission ---

    		spiTxComplete = 0; // Reset transmission complete flag

    		// Transmit the upscaled packet buffer.
    		// HAL_SPI_Transmit_DMA takes uint8_t pointer and size in data items.
    		// Since SPI is 8-bit, size is number of bytes. upscaled_length is in bytes.
    		if (HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)ble_tx_buffer, (uint16_t)upscaled_length) != HAL_OK)
    		{
    		  Error_Handler();
    		}
    		// Wait for all DMA transmissions to complete (flag set in HAL_SPI_TxCpltCallback)
    		while (spiTxComplete == 0)
    		{
    		   // You can add a small delay or WFE instruction here if needed
    		   // HAL_Delay(1);
    		   // __WFE(); // Requires enabling events if using sleep modes
    		}
      }*/


         //infinite loop fake dynatomics demo data
  float base_temp_celsius = 25.5f;
  float current_temp_celsius;
  float current_voltage_v = 3.2f;

  // Timer for voltage updates
  uint32_t last_voltage_update_tick = 0;

  while (1)
    {
      /* --- 1. Handle timing for voltage update --- */
      // Check if 700ms have passed since the last update
      if (HAL_GetTick() - last_voltage_update_tick >= 700)
      {
          last_voltage_update_tick = HAL_GetTick(); // Reset the timer

          // Increment voltage by 10mV
          current_voltage_v += 0.010f;

          // If voltage reaches or exceeds 4.2V, reset it to 3.2V
          if (current_voltage_v >= 4.2f) {
              current_voltage_v = 3.2f;
          }
      }

      /* --- 2. Update temperature with random fluctuation --- */
      // Generate a random float between -0.25 and +0.25
      float random_offset = ((float)rand() / (float)RAND_MAX) * 0.5f - 0.25f;
      current_temp_celsius = base_temp_celsius + random_offset;

      /* --- 3. Convert current temperature and voltage to raw hex values --- */
      // Formula for tempValue from tempCelsius: tempValue = (tempCelsius + 45.0) * 65535.0 / 175.0
      uint16_t raw_temp = (uint16_t)((current_temp_celsius + 45.0f) * 65535.0f / 175.0f);

      // Formula for adcValue from voltageInVolts: adcValue = voltageInVolts * 1000.0 * 150.0 / 328.0
      uint16_t raw_adc = (uint16_t)(current_voltage_v * 1000.0f * 150.0f / 328.0f);

      /* --- 4. Populate AdvData array with new values --- */
      // Voltage (MSB first, then LSB)
      AdvData[0] = (raw_adc >> 8) & 0xFF;
      AdvData[1] = raw_adc & 0xFF;
      // Temperature (MSB first, then LSB)
      AdvData[2] = (raw_temp >> 8) & 0xFF;
      AdvData[3] = raw_temp & 0xFF;

      // The line "memset(&AdvData[4], 0x00, sizeof(AdvData) - 4);" has been removed as requested.

      /* --- 5. Transmit the BLE packet --- */
      // 1. Create PDU
      if (create_legacy_advertising_pdu(&pdu, AdvA, sizeof(AdvA), AdvData, sizeof(AdvData)) != 0) {
          Error_Handler();
      }

      // 2. Initialize Packet (includes whitening, CRC)
      if (init_packet(&packet, BLE_CHANNEL, &pdu, BLE_PACKET_TYPE) != 0) {
          Error_Handler();
      }

      // 3. Upscale packet for backscatter transmission
      #if BLE_PACKET_TYPE == UNCODED_1MBPS
          upscaled_length = jv_bsc_upscale_1Mbps(ble_tx_buffer, packet.whitened_packet, packet.packet_len);
      #elif BLE_PACKET_TYPE == UNCODED_2MBPS
          upscaled_length = jv_bsc_upscale_2Mbps(ble_tx_buffer, packet.whitened_packet, packet.packet_len);
      #else
          #error "Unsupported BLE_PACKET_TYPE for jv_bsc_upscale"
      #endif

      // --- SPI DMA Transmission ---
      spiTxComplete = 0; // Reset transmission complete flag

      if (HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)ble_tx_buffer, (uint16_t)upscaled_length) != HAL_OK)
      {
        Error_Handler();
      }

      // Wait for DMA transmission to complete (flag set in HAL_SPI_TxCpltCallback)
      while (spiTxComplete == 0)
      {
         // Wait for interrupt
      }
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00B07CB4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
