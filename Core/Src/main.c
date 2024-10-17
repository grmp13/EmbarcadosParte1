/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define STDOUT_FILENO   1
#define STDERR_FILENO   2
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Variaveis do sistema
volatile int16_t potenciometroQuantidade = 0;
volatile int16_t potenciometroBalanca = 0;
volatile int16_t distanciaGlobal = 0;
volatile uint8_t received_command = 0;


//variaveis a serem incrementadas
volatile int16_t potenciometroQuantidadeSelecionado = 0;

// semaforo
SemaphoreHandle_t xSemaphore = NULL;

// Flags sistema
volatile uint8_t flagBotao = 0;
volatile uint8_t flagMotor = 0;
volatile uint8_t flagBuzzer = 0;
volatile uint8_t flagLed = 0;
volatile uint8_t flagQuantidadeSelecionada = 0;
volatile uint8_t flagRacao = 0;

// estados tasks
TaskHandle_t xTaskPotenciometroQuantidade = 0;
TaskHandle_t xTaskRacao = 0;
TaskHandle_t xTaskBotao = 0;
TaskHandle_t xTaskMotor = 0;
TaskHandle_t xTaskSensorUltrassonico = 0;
TaskHandle_t xTaskBuzzer = 0;
TaskHandle_t xTaskLed = 0;
TaskHandle_t xTaskPotenciometroQtd = 0;
TaskHandle_t xTaskBalanca = 0;
TaskHandle_t xTaskBotaoSerialMode = 0;
TaskHandle_t xTaskPrints = 0;
TaskHandle_t xTaskSerialMOde = 0;

#define TRIGGER_PIN GPIO_PIN_8
#define TRIGGER_PORT GPIOA
#define ECHO_PIN GPIO_PIN_10
#define ECHO_PORT GPIOA
#define DEBOUNCE_DELAY 50

void HCSR04_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = TRIGGER_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TRIGGER_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ECHO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);
}

uint32_t HCSR04_Read(void) {
	uint32_t local_time = 0;
	uint32_t start_time = 0;
	uint32_t stop_time = 0;

	HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_SET);
	HAL_Delay(0.01);
	HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);

	while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
		;

	start_time = HAL_GetTick();

	while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
		;

	stop_time = HAL_GetTick();
	local_time = stop_time - start_time;
	uint32_t distance = (local_time * 34300) / 2000;

	return distance;
}

void Buzzer_Init(void) {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void SetBuzzerIntensity(uint16_t dutyCycle) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dutyCycle);
}

int16_t leituraPotenciometro1(void) {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(&hadc1);
}
int16_t leituraPotenciometro2(void) {
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(&hadc2);
}

void vTaskLed(void *pvParameters) {
	for (;;) {
		if (distanciaGlobal == 0) {
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
			vTaskDelay(500);
		} else {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		}
	}
}

void vTaskBuzzer(void *pvParameters) {
	Buzzer_Init();

	for (;;) {
		if (distanciaGlobal == 0) {
			SetBuzzerIntensity(30);
			HAL_Delay(200);
			SetBuzzerIntensity(0);
			HAL_Delay(1000);
		}
	}
}
void vTaskSensorUltrassonico(void *pvParameters) {
	volatile int16_t distancia = 0;

	for (;;) {
		distancia = HCSR04_Read();

		if (distancia <= 20) {
			distanciaGlobal = 0;
			if ( xTaskBuzzer == 0) {
				xTaskCreate(vTaskBuzzer, "Buzzer", 128, NULL, 1, &xTaskBuzzer);
				xTaskCreate(vTaskLed, "Led", 128, NULL, 1, &xTaskLed);
			}
		} else if (distancia > 20) {
			distanciaGlobal = 1;
		}

		vTaskDelay(pdMS_TO_TICKS(700));
	}
}

void vTaskMotorPassos(void *pvParameters) {
	flagMotor = 0;
	for (;;) {

		if (distanciaGlobal == 1) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_Delay(2);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_Delay(2);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_Delay(2);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
			HAL_Delay(2);
			if(flagMotor == 0){
				printf("Sensor livre\n\r");
				flagMotor = 1;
			}
		}else{
			if(flagMotor == 1){
				printf("Sensor obstruido\n\r");
				flagMotor = 0;

			}
		}
	}

}

void vTaskReceiveCommand(void *pvParameters) {
    while (1) {
        HAL_UART_Receive(&huart2, (uint8_t*)&received_command, 1, HAL_MAX_DELAY);

    }
}

void vTaskPotenciometroQtd(void *pvParameters) {
	int16_t valorPotenciometro;
	for (;;) {
		valorPotenciometro = leituraPotenciometro1();
		potenciometroQuantidade = valorPotenciometro / 10;
		HAL_Delay(1000);
	}
}

void vTaskPotenciometroBalanca(void *pvParameters) {
	int16_t valorPotenciometroBalanca;

	for (;;) {
		valorPotenciometroBalanca = leituraPotenciometro2();
		potenciometroBalanca = valorPotenciometroBalanca / 10;
		HAL_Delay(1000);
	}
}

void vTaskQuantidadeRacao(void *pvParameters) {
	for (;;) {
		printf("Escolha a quantidade: %i\n\r", potenciometroQuantidade);
		potenciometroQuantidadeSelecionado = potenciometroQuantidade;
		vTaskDelay(1000);
	}
}

void vTaskEquilibrio(void *pvParameters) {

	volatile uint8_t flagComida = 0;
	volatile int16_t balancaCaindo = 0;
	volatile int16_t balancaSubindo = 0;

	for (;;) {
		if (distanciaGlobal == 0) {
			if (flagComida == 0) {
				balancaCaindo = potenciometroBalanca;
				flagComida = 1;
			}

		} else if (distanciaGlobal == 1) {
			potenciometroQuantidadeSelecionado = potenciometroQuantidadeSelecionado - balancaCaindo;
			balancaCaindo = 0;

			if (flagComida == 1) {
				balancaSubindo = potenciometroBalanca;
				potenciometroQuantidadeSelecionado = potenciometroQuantidadeSelecionado + balancaSubindo;
				flagComida = 0;
				balancaSubindo = 0;
			}
		}
	}
}

void vTaskRacao(void *pvParameters) {
	xTaskCreate(vTaskPotenciometroBalanca, "balancaPotenciometro", 128, NULL, 1,
			&xTaskBalanca);
	xTaskCreate(vTaskEquilibrio, "equilibrio", 128, NULL, 1, &xTaskBalanca);
	for (;;) {
		if (potenciometroBalanca >= potenciometroQuantidadeSelecionado) {
			flagRacao = 1;
		} else {
			flagRacao = 0;
		}
	}
}

void vTaskPrint(void *pvParameters) {
	for (;;) {
		if(flagBotao == 1){
			printf("Balanca: %i\n\r ", potenciometroBalanca);
			printf("Quantidade Racao: %i\n\r ", potenciometroQuantidadeSelecionado);
		}
		vTaskDelay(1500);

	}
}

void vTaskBotao(void *pvParameters) {
	static int estadoBotaoAnterior = GPIO_PIN_RESET;
	int estadoBotaoAtual;
	int flagStatus = 0;

	xTaskCreate(vTaskPotenciometroQtd, "quantidadePotenciometro", 128, NULL, 1,
			&xTaskPotenciometroQtd);
	xTaskCreate(vTaskQuantidadeRacao, "QuantidadeRacao", 128, NULL, 1,
			&xTaskPotenciometroQuantidade);
	xTaskCreate(vTaskPrint, "print", 128, NULL, 1,
			&xTaskPrints);


	for (;;) {
		estadoBotaoAtual = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);

		if (estadoBotaoAtual != estadoBotaoAnterior) {
			vTaskDelay(DEBOUNCE_DELAY);
			estadoBotaoAtual = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
			if (estadoBotaoAtual != estadoBotaoAnterior) {
				estadoBotaoAnterior = estadoBotaoAtual;
				if (estadoBotaoAtual == GPIO_PIN_SET) {
					flagBotao = !flagBotao;
				}
			}
		}

		if (flagBotao == 1 && flagStatus == 0) {
			if (xTaskPotenciometroQuantidade != 0) {
				vTaskDelete(xTaskPotenciometroQuantidade);
				xTaskPotenciometroQuantidade = 0;
			}
			if (xTaskPotenciometroQtd != 0) {
				vTaskDelete(xTaskPotenciometroQtd);
				xTaskPotenciometroQtd = 0;
			}
			if (xTaskSensorUltrassonico == 0) {
				xTaskCreate(vTaskSensorUltrassonico, "SensorUltra", 128, NULL,
						2, &xTaskSensorUltrassonico);
			}
			if (xTaskRacao == 0) {
				xTaskCreate(vTaskRacao, "racao", 128, NULL, 1, &xTaskRacao);
			}

			if (xTaskMotor == 0) {
				xTaskCreate(vTaskMotorPassos, "MotorPassos", 128, NULL, 1,
						&xTaskMotor);

			}
			flagStatus = 1;

		}
		if (flagBotao == 0 && flagStatus == 1 || flagRacao == 1) {
			printf("\r FINALIZADO COM SUCESSO\n\n\r");
			if (xTaskSensorUltrassonico != 0) {
				vTaskDelete(xTaskSensorUltrassonico);
				xTaskSensorUltrassonico = 0;
			}
			if (xTaskMotor != 0) {
				vTaskDelete(xTaskMotor);
				xTaskMotor = 0;
			}
			if (xTaskPrints != 0) {
				vTaskDelete(xTaskPrints);
				xTaskPrints = 0;
			}
			printf("\rEscolha o modo a ser iniciado:\n\r");
			printf("Modo Hardware - H\n\r");
			printf("Modo Serial - S\n\r");
        	printf("Parar - P\n\r");
			flagBotao = 0;
			flagStatus = 0;
			xTaskBotao = 0;
			vTaskDelete(NULL);
		}
		vTaskDelay(50);
	}
}

void vTaskBotaoSerialMode(void *pvParameters) {
	static int estadoBotaoAnterior = GPIO_PIN_RESET;
	int estadoBotaoAtual;
	int flagStatuss = 0;

	xTaskCreate(vTaskPotenciometroQtd, "quantidadePotenciometro", 128, NULL, 1,
			&xTaskPotenciometroQtd);
	xTaskCreate(vTaskPrint, "print", 128, NULL, 1,
			&xTaskPrints);


	for (;;) {
		estadoBotaoAtual = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);

		if (estadoBotaoAtual != estadoBotaoAnterior) {
			vTaskDelay(DEBOUNCE_DELAY);
			estadoBotaoAtual = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
			if (estadoBotaoAtual != estadoBotaoAnterior) {
				estadoBotaoAnterior = estadoBotaoAtual;
				if (estadoBotaoAtual == GPIO_PIN_SET) {
					flagBotao = !flagBotao;
				}
			}
		}

		if (flagBotao == 1 && flagStatuss == 0) {
			if (xTaskPotenciometroQtd != 0) {
				vTaskDelete(xTaskPotenciometroQtd);
				xTaskPotenciometroQtd = 0;
			}
			if (xTaskSensorUltrassonico == 0) {
				xTaskCreate(vTaskSensorUltrassonico, "SensorUltra", 128, NULL,
						2, &xTaskSensorUltrassonico);
			}
			if (xTaskRacao == 0) {
				xTaskCreate(vTaskRacao, "racao", 128, NULL, 1, &xTaskRacao);
			}

			if (xTaskMotor == 0) {
				xTaskCreate(vTaskMotorPassos, "MotorPassos", 128, NULL, 1,
						&xTaskMotor);

			}
			flagStatuss = 1;

		}
		if (flagBotao == 0 && flagStatuss == 1 || flagRacao == 1) {
			printf("\r FINALIZADO COM SUCESSO\n\n\r");
			if (xTaskSensorUltrassonico != 0) {
				vTaskDelete(xTaskSensorUltrassonico);
				xTaskSensorUltrassonico = 0;
			}
			if (xTaskMotor != 0) {
				vTaskDelete(xTaskMotor);
				xTaskMotor = 0;
			}
			if (xTaskPrints != 0) {
				vTaskDelete(xTaskPrints);
				xTaskPrints = 0;
			}
			printf("\rEscolha o modo a ser iniciado:\n\r");
			printf("Modo Hardware - H\n\r");
			printf("Modo Serial - S\n\r");
        	printf("Parar - P\n\r");
			flagBotao = 0;
			flagStatuss = 0;
			xTaskBotao = 0;
			vTaskDelete(NULL);
		}
		vTaskDelay(50);
	}
}


void vTaskSerialMode(void *pvParameters) {
	printf("\rEscolha a quantidade de ração desejada:\n\r");
	printf("100g - 1\n\r");
	printf("150g - 2\n\r");
	printf("200g - 3\n\r");
	printf("250g - 4\n\r");
	printf("300g - 5\n\r");
	printf("350g - 6\n\r");
	printf("400g - 7\n\r");
	for (;;) {
        switch (received_command) {
            case '1':
            	printf("Sua escolha foi de 100g\n\r");
            	potenciometroQuantidadeSelecionado = 100;
            	flagBotao = 1;
            	xTaskCreate(vTaskBotaoSerialMode, "botaoSerialMode", 128, NULL, 1, &xTaskBotaoSerialMode);
                received_command = 0;
                xTaskBotaoSerialMode =0;
            	vTaskDelete(NULL);
                break;
            case '2':
            	printf("Sua escolha foi de 150g\n\r");
            	potenciometroQuantidadeSelecionado = 150;
            	flagBotao = 1;
            	xTaskCreate(vTaskBotaoSerialMode, "botaoSerialMode", 128, NULL, 1, &xTaskBotaoSerialMode);
                received_command = 0;
                xTaskBotaoSerialMode = 0;
            	vTaskDelete(NULL);
                break;
            case '3':
            	printf("Sua escolha foi de 200g\n\r");
            	potenciometroQuantidadeSelecionado = 200;
            	flagBotao = 1;
            	xTaskCreate(vTaskBotaoSerialMode, "botaoSerialMode", 128, NULL, 1, &xTaskBotaoSerialMode);
                received_command = 0;
                xTaskBotaoSerialMode = 0;
            	vTaskDelete(NULL);
                break;
            case '4':
            	printf("Sua escolha foi de 250g\n\r");
            	potenciometroQuantidadeSelecionado = 250;
            	flagBotao = 1;
            	xTaskCreate(vTaskBotaoSerialMode, "botaoSerialMode", 128, NULL, 1, &xTaskBotaoSerialMode);
                received_command = 0;
                xTaskBotaoSerialMode = 0;
            	vTaskDelete(NULL);
                break;
            case '5':
            	printf("Sua escolha foi de 300g\n\r");
            	potenciometroQuantidadeSelecionado = 300;
            	flagBotao = 1;
            	xTaskCreate(vTaskBotaoSerialMode, "botaoSerialMode", 128, NULL, 1, &xTaskBotaoSerialMode);
            	flagBotao = 1;
            	received_command = 0;
            	xTaskBotaoSerialMode = 0;
            	vTaskDelete(NULL);
                break;
            case '6':
            	printf("Sua escolha foi de 350g\n\r");
            	potenciometroQuantidadeSelecionado = 350;
            	flagBotao = 1;
            	xTaskCreate(vTaskBotaoSerialMode, "botaoSerialMode", 128, NULL, 1, &xTaskBotaoSerialMode);
                received_command = 0;
                xTaskBotaoSerialMode = 0;
            	vTaskDelete(NULL);
                break;
            case '7':
            	printf("Sua escolha foi de 400g\n\r");
            	potenciometroQuantidadeSelecionado = 400;
            	flagBotao = 1;
            	xTaskCreate(vTaskBotaoSerialMode, "botaoSerialMode", 128, NULL, 1, &xTaskBotaoSerialMode);
                received_command = 0;
                xTaskBotaoSerialMode =0;
            	vTaskDelete(NULL);
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTaskControllerMode(void *pvParameters) {
	printf("\rEscolha o modo a ser iniciado:\n\r");
	printf("Modo Hardware - H\n\r");
	printf("Modo Serial - S\n\r");
	printf("Parar - P\n\r");
	for (;;) {
        switch (received_command) {
            case 'h':
            case 'H':
            	printf("Modo Hardware\n\r");
            	xTaskCreate(vTaskBotao, "botao", 128, NULL, 1, &xTaskBotao);
                break;

            case 's':
            case 'S':
            	printf("Modo Serial\n\r");
            	xTaskCreate(vTaskSerialMode, "modoSerial", 128, NULL, 1, &xTaskSerialMOde);
                break;
            case 'p':
            case 'P':
            	printf("CANCELADO!\n\r");
            	flagBotao = 0;
            	printf("\rEscolha o modo a ser iniciado:\n\r");
            	printf("Modo Hardware - H\n\r");
            	printf("Modo Serial - S\n\r");
            	printf("Parar - P\n\r");
                break;
        }
        received_command = 0;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

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
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	xSemaphore = xSemaphoreCreateBinary();

//	xTaskCreate(vTaskPotenciometroBalanca, "potenciometro balanca", 128, NULL, 1, NULL);
//	xTaskCreate(vTaskControllador, "Controller", 128, NULL, 1, NULL);
//	xTaskCreate(vTaskSensorUltrassonico, "SensorUltra", 128, NULL, 1,
//			&xTaskSensorUltrassonico);
	xTaskCreate(vTaskReceiveCommand, "comand", 128, NULL, 1, NULL);
	xTaskCreate(vTaskControllerMode, "MODE", 128, NULL, 1, NULL);

	vTaskStartScheduler();
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		//      uint32_t distance = Read_Distance();
		// Aqui você pode exibir ou utilizar a variável `distance`
		//     HAL_Delay(1000);  // Medição a cada 1 segundo

		//      printf("%i distancia",distance);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
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
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
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
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc2.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
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
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 127;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 2000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 400 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 10000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	GPIO_PIN_10 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA6 PA7 PA8 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB4 PB5 PB8
	 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8
			| GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int _write(int file, uint8_t *ptr, int len) {
	switch (file) {
	case STDOUT_FILENO:
		HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
		break;

	case STDERR_FILENO:
		HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
		break;

	default:
		return -1;
	}

	return len;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */
	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
