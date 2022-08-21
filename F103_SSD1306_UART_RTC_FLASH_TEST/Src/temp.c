/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
 
/* USER CODE BEGIN Includes */
 
/* USER CODE END Includes */
 
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
 
UART_HandleTypeDef huart2;
 
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
 
/* USER CODE END PV */
 
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
 
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
 
/* USER CODE END PFP */
 
/* USER CODE BEGIN 0 */
int _write(int32_t file, uint8_t *ptr, int32_t len) {
  HAL_UART_Transmit(&huart2, ptr, len, 10);
  return len;
}
 
#define ADDRESS     (0x53<<1)
#define POWER_CTL   0x2D
#define INT_ENABLE  0x2E
#define DATA_FORMAT 0x31
#define acc_value   0.00390625
 
uint8_t rx_data[6];
 
void TWI_write(uint8_t addr, uint8_t data) {
  uint8_t temp[2];
  temp[0]=addr;
  temp[1]=data;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, temp, 2, 1000)!=HAL_OK);
}
 
void TWI_read(uint8_t addr) {
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &addr, 1, 1000)!=HAL_OK);
  while(HAL_I2C_Master_Receive(&hi2c1, ADDRESS, rx_data, 6, 1000)!=HAL_OK);
}
 
void ADXL_init() {
  TWI_write(0x2D, 0x08);  // Measurement mode
  TWI_write(0x2E, 0x80);  // Data ready
  TWI_write(0x31, 0x0B);  // Full resolution, +-16g
}
 
/* USER CODE END 0 */
 
int main(void)
{
 
  /* USER CODE BEGIN 1 */
 
  /* USER CODE END 1 */
 
  /* MCU Configuration----------------------------------------------------------*/
 
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
 
  /* USER CODE BEGIN 2 */
  printf("Start.....\r\n");
  ADXL_init();
  HAL_Delay(500);
  /* USER CODE END 2 */
 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
 
  /* USER CODE BEGIN 3 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
    TWI_read(0x32);
    int16_t x = ((rx_data[1]<<8)&0xFF00) | (rx_data[0]&0xFF);
    int16_t y = ((rx_data[3]<<8)&0xFF00) | (rx_data[2]&0xFF);
    int16_t z = ((rx_data[5]<<8)&0xFF00) | (rx_data[4]&0xFF);
    printf("X = %4d,  Y = %4d,  Z = %4d \r\n", x, y, z);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
    HAL_Delay(1000);
  }
  /* USER CODE END 3 */
 
}
