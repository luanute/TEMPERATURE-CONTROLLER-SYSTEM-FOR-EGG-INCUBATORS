#ifndef STM32F1_GPIO_H
#define STM32F1_GPIO_H
#include "stm32f10x.h" 


// Enum cho tốc độ Output (tương ứng giá trị MODE bits khi MODE > 0)
typedef enum {
    GPIO_SPEED_FREQ_LOW    = 2, // Output mode, max speed 2 MHz
    GPIO_SPEED_FREQ_MEDIUM = 1, // Output mode, max speed 10 MHz
    GPIO_SPEED_FREQ_HIGH   = 3  // Output mode, max speed 50 MHz
} GPIOSpeed_TypeDef;

// Enum cho cấu hình Pull-up/Pull-down (chỉ dùng cho Input mode)
typedef enum {
    GPIO_NOPULL   = 0x00U,
    GPIO_PULLUP   = 0x01U,
    GPIO_PULLDOWN = 0x02U // Dùng giá trị này để phân biệt trong code xử lý ODR
} GPIOPull_TypeDef;

// Enum cho Mode và CNF kết hợp (bao quát các trường hợp cấu hình)
typedef enum {
    // Chế độ Input
    GPIO_MODE_ANALOG,         // Input Analog: MODE=00, CNF=00
    GPIO_MODE_INPUT_FLOATING, // Input Floating: MODE=00, CNF=01
    GPIO_MODE_INPUT_PUPD,     // Input Pull-up/Pull-down: MODE=00, CNF=10 (Cần thông tin Pull)
    // Chế độ Output
    GPIO_MODE_OUTPUT_PP,      // Output Push-Pull: MODE>0, CNF=00 (Cần thông tin Speed)
    GPIO_MODE_OUTPUT_OD,      // Output Open-Drain: MODE>0, CNF=01 (Cần thông tin Speed)
    // Chế độ Alternate Function
    GPIO_MODE_AF_PP,          // AF Push-Pull: MODE>0, CNF=10 (Cần thông tin Speed)
    GPIO_MODE_AF_OD,          // AF Open-Drain: MODE>0, CNF=11 (Cần thông tin Speed)
		// Chế độ Interrupt 
		GPIO_MODE_IT_FT,          // Interrupt on Falling Trigger
		GPIO_MODE_IT_RT,          // Interrupt on Rising Trigger
		GPIO_MODE_IT_RFT 					// Interrupt on Rising and Falling Trigger
} GPIOMode_TypeDef;


typedef struct
{
  GPIOMode_TypeDef  Mode;    // Chế độ hoạt động (kết hợp Mode/CNF)
  GPIOSpeed_TypeDef Speed;   // Tốc độ (chỉ áp dụng cho Output/AF)
  GPIOPull_TypeDef  Pull;    // Kéo trở (chỉ áp dụng cho Input PUPD)
	//uint8_t           Reserved; 
	uint8_t Pin;     // Pin number (0-15)
} GPIO_PinConfig_t;

typedef struct
{
  GPIO_TypeDef *pGPIOx; // Base address of GPIO port
  GPIO_PinConfig_t GPIO_PinConfig; // Pin configuration settings
} GPIO_Handle_t;

typedef enum {
    EXTI_TRIGGER_RISING,
    EXTI_TRIGGER_FALLING,
    EXTI_TRIGGER_RISING_FALLING
} EXTI_Trigger_t;

typedef struct {
    GPIO_TypeDef *GPIO_Port;   // Port: A, B, C...
    uint8_t GPIO_Pin;         // Pin: 0..15
    EXTI_Trigger_t Trigger;    // Kiểu trigger
    uint8_t IRQ_PreemptionPriority;
    uint8_t IRQ_SubPriority;
} EXTI_Config_t;


// ennable clock cho port GPIO
#define RCC_GPIOA_EN RCC->APB2ENR|=(1<<2)
#define RCC_GPIOB_EN RCC->APB2ENR|=(1<<3)
#define RCC_GPIOC_EN RCC->APB2ENR|=(1<<4)
#define RCC_GPIOD_EN RCC->APB2ENR|=(1<<5)
#define RCC_GPIOE_EN RCC->APB2ENR|=(1<<6)
#define RCC_GPIOF_EN RCC->APB2ENR|=(1<<7)
#define RCC_GPIOG_EN RCC->APB2ENR|=(1<<8)


// ennable clock cho port SPI
#define RCC_SPI1_EN RCC->APB2ENR|=(1<<12)
#define RCC_SPI2_EN RCC->APB1ENR|=(1<<14)
#define RCC_SPI3_EN RCC->APB1ENR|=(1<<15)

// ennable clock cho port I2C
#define RCC_I2C1_EN RCC->APB1ENR|=(1<<21)
#define RCC_I2C2_EN RCC->APB1ENR|=(1<<22)

// ennable clock cho port USART
#define RCC_USART1_EN RCC->APB2ENR|=(1<<14)
#define RCC_USART2_EN RCC->APB1ENR|=(1<<17)
#define RCC_USART3_EN RCC->APB1ENR|=(1<<18)
#define RCC_USART4_EN RCC->APB1ENR|=(1<<19)
#define RCC_USART5_EN RCC->APB1ENR|=(1<<20)
// ennable clock cho port EXTI
#define RCC_EXTI_EN RCC->APB2ENR|=(1<<0)
// ennable clock cho port AFIO
#define RCC_AFIO_EN RCC->APB2ENR|=(1<<0)


#define GPIO_BASSEADDR_TO_CODE(x) ( \
    ((x) == GPIOA) ? 0 : \
    ((x) == GPIOB) ? 1 : \
    ((x) == GPIOC) ? 2 : \
    ((x) == GPIOD) ? 3 : \
    ((x) == GPIOE) ? 4 : \
    -1 )


#define ENABLE 1
#define DISABLE 0
#define SET 1
#define RESET 0
#define FLAG_SET SET
#define FLAG_RESET RESET


#define GPIO_PIN_NUM_0         0  // Pin 0
#define GPIO_PIN_NUM_1         1  // Pin 1
#define GPIO_PIN_NUM_2         2  // Pin 2
#define GPIO_PIN_NUM_3         3  // Pin 3
#define GPIO_PIN_NUM_4         4  // Pin 4
#define GPIO_PIN_NUM_5         5  // Pin 5
#define GPIO_PIN_NUM_6         6  // Pin 6
#define GPIO_PIN_NUM_7         7  // Pin 7
#define GPIO_PIN_NUM_8         8  // Pin 8
#define GPIO_PIN_NUM_9         9  // Pin 9
#define GPIO_PIN_NUM_10        10 // Pin 10
#define GPIO_PIN_NUM_11        11 // Pin 11
#define GPIO_PIN_NUM_12        12 // Pin 12
#define GPIO_PIN_NUM_13        13 // Pin 13
#define GPIO_PIN_NUM_14        14 // Pin 14
#define GPIO_PIN_NUM_15        15 // Pin 15



void GPIO_ClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnorDis);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_TypeDef  *pGPIOx);
void GPIOx_INIT(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin, uint8_t Mode, uint8_t Pull, uint8_t Speed );
void GPIO_Write(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin, uint8_t State );
uint8_t GPIO_Read(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin);
void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin);
void EXTI_Init(EXTI_Config_t *exti_config);
//void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
//void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
//void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* __STM32F1_H */

