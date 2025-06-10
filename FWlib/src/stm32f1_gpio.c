#include "stm32f1_gpio.h"

void GPIO_ClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnorDis)
{
    if (EnorDis)
    {
        if (pGPIOx == GPIOA) RCC_GPIOA_EN;
        else if (pGPIOx == GPIOB) RCC_GPIOB_EN;
        else if (pGPIOx == GPIOC) RCC_GPIOC_EN;
        else if (pGPIOx == GPIOD) RCC_GPIOD_EN;
        else if (pGPIOx == GPIOE) RCC_GPIOE_EN;
        //else if (pGPIOx == GPIOF) RCC_GPIOF_EN;
        //else if (pGPIOx == GPIOG) RCC_GPIOG_EN;
    }
    else
    {
        // Tắt clock 
    }
}

void GPIO_DeInit(GPIO_TypeDef *pGPIOx){
    if(pGPIOx == GPIOA) RCC->APB2RSTR |= (1 << 2); // Reset GPIOA
    else if(pGPIOx == GPIOB) RCC->APB2RSTR |= (1 << 3); // Reset GPIOB
    else if(pGPIOx == GPIOC) RCC->APB2RSTR |= (1 << 4); // Reset GPIOC
    else if(pGPIOx == GPIOD) RCC->APB2RSTR |= (1 << 5); // Reset GPIOD
    else if(pGPIOx == GPIOE) RCC->APB2RSTR |= (1 << 6); // Reset GPIOE
    //else if(pGPIOx == GPIOF) RCC->APB2RSTR |= (1 << 7); // Reset GPIOF
    //else if(pGPIOx == GPIOG) RCC->APB2RSTR |= (1 << 8); // Reset GPIOG
}


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t mode_cnf_bits = 0; // Biến tạm lưu 4 bit MODE+CNF
    uint32_t mode_val = 0;      // Giá trị cho MODE[1:0]
    uint32_t cnf_val = 0;       // Giá trị cho CNF[1:0]
    __IO uint32_t* reg;         // Con trỏ tới thanh ghi CRL hoặc CRH
    uint32_t pin_offset;        // Vị trí bit bắt đầu của pin trong thanh ghi (0, 4, 8, ...)

    // 1. Enable clock for the GPIO port
    GPIO_ClockControl(pGPIOHandle->pGPIOx, ENABLE);

     /* ---- 2. Xác định thanh ghi (CRL/CRH) và vị trí bit ---- */
     if (pGPIOHandle->GPIO_PinConfig.Pin < 8) {
        // Pin 0-7 dùng thanh ghi CRL
        reg = &pGPIOHandle->pGPIOx->CRL;
        pin_offset = pGPIOHandle->GPIO_PinConfig.Pin * 4;
    } else {
        // Pin 8-15 dùng thanh ghi CRH
        reg = &pGPIOHandle->pGPIOx->CRH;
        pin_offset = (pGPIOHandle->GPIO_PinConfig.Pin - 8) * 4;
    }

    /* ---- 3. Tính toán giá trị MODE và CNF bits dựa trên struct ---- */
    switch (pGPIOHandle->GPIO_PinConfig.Mode) {
        case GPIO_MODE_ANALOG:
            mode_val = 0; // Input mode
            cnf_val = 0;  // Analog
            break;
        case GPIO_MODE_INPUT_FLOATING:
            mode_val = 0; // Input mode
            cnf_val = 1;  // Floating
            break;
        case GPIO_MODE_INPUT_PUPD:
            mode_val = 0; // Input mode
            cnf_val = 2;  // Input with pull-up/pull-down
            // Việc chọn pull-up hay pull-down sẽ xử lý qua ODR sau
            break;
        case GPIO_MODE_OUTPUT_PP:
            mode_val = pGPIOHandle->GPIO_PinConfig.Speed; // Lấy speed từ tham số
            cnf_val = 0;               // General purpose output push-pull
            break;
        case GPIO_MODE_OUTPUT_OD:
            mode_val = pGPIOHandle->GPIO_PinConfig.Speed; // Lấy speed từ tham số
            cnf_val = 1;               // General purpose output open-drain
            break;
        case GPIO_MODE_AF_PP:
            mode_val = pGPIOHandle->GPIO_PinConfig.Speed; // Lấy speed từ tham số
            cnf_val = 2;               // Alternate function output Push-pull
            break;
        case GPIO_MODE_AF_OD:
            mode_val = pGPIOHandle->GPIO_PinConfig.Speed; // Lấy speed từ tham số
            cnf_val = 3;               // Alternate function output Open-drain
						break;
				case GPIO_MODE_IT_RT:		
						EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.Pin);
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.Pin);
						mode_val = 0;
						cnf_val = 2;
						break;
				case GPIO_MODE_IT_FT:		
						EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.Pin);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.Pin);
						mode_val = 0;
						cnf_val = 2;
						break;
				case GPIO_MODE_IT_RFT:		
						EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.Pin);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.Pin);
						mode_val = 0;
						cnf_val = 1;			
    }
     
    // Kết hợp CNF và MODE thành 4 bit config (CNF nằm ở 2 bit cao)
    mode_cnf_bits = (cnf_val << 2) | mode_val;

    /* ---- 4. Ghi vào thanh ghi CRL/CRH ---- */
    // clear 4 bit cũ
    *reg &= ~(0x0FUL << pin_offset);

    // Đọc giá trị thanh ghi hiện tại, xóa 4 bit cũ, OR giá trị mới vào
    *reg |= (mode_cnf_bits << pin_offset);


    /* ---- 5. Cấu hình Pull-up/Pull-down (nếu là Input PUPD || EXTI) ---- */
    if ((pGPIOHandle->GPIO_PinConfig.Mode == GPIO_MODE_INPUT_PUPD) || (pGPIOHandle->GPIO_PinConfig.Mode == GPIO_MODE_IT_RT) || (pGPIOHandle->GPIO_PinConfig.Mode == GPIO_MODE_IT_RT)){
        if (pGPIOHandle->GPIO_PinConfig.Pull == GPIO_PULLUP) {
            // Set bit ODR để chọn Pull-up
            pGPIOHandle->pGPIOx->ODR |= (1UL << pGPIOHandle->GPIO_PinConfig.Pin);
            // Hoặc dùng BSRR (thường an toàn hơn trong ngữ cảnh ngắt):
            // initParams->Port->BSRR = odr_mask;
        } else if (pGPIOHandle->GPIO_PinConfig.Pull == GPIO_PULLDOWN) {
            // Clear bit ODR để chọn Pull-down
            pGPIOHandle->pGPIOx->ODR &= ~(1UL << pGPIOHandle->GPIO_PinConfig.Pin);
            // Hoặc dùng BRR:
            // initParams->Port->BRR = odr_mask;
        }
        // Nếu là GPIO_NOPULL thì không cần làm gì với ODR
    }
}




void GPIOx_INIT(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin, uint8_t Mode, uint8_t Pull, uint8_t Speed ){
    GPIO_Handle_t pGPIOHandle;
    pGPIOHandle.pGPIOx = GPIOx;
    pGPIOHandle.GPIO_PinConfig.Pin  = GPIO_Pin;
    pGPIOHandle.GPIO_PinConfig.Mode = Mode;
    pGPIOHandle.GPIO_PinConfig.Pull = Pull;
    pGPIOHandle.GPIO_PinConfig.Speed = Speed;
    GPIO_Init(&pGPIOHandle);
}

void GPIO_Write(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin, uint8_t State ){
      if(State){	GPIOx->BSRR|=(1<<GPIO_Pin);}
			else{GPIOx->BSRR|=(1<<(GPIO_Pin+16));}
}

uint8_t GPIO_Read(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin) {
	return (((GPIOx->IDR)&(1<<GPIO_Pin)) == 0)? 0:1;
}

void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin){
      GPIOx->ODR ^= (1<<GPIO_Pin);
}


void EXTI_Init(EXTI_Config_t *exti_config) {
    // 1. Enable clock GPIO & AFIO
     GPIO_ClockControl(exti_config->GPIO_Port, ENABLE);
     RCC_EXTI_EN;      // Enable AFIO


    // 3. Liên kết EXTI line với GPIO
    uint8_t exti_line = exti_config->GPIO_Pin;
    uint8_t reg_index = exti_line / 4;
    uint8_t pos = (exti_line % 4) * 4;
		uint8_t port_index = (uint8_t)GPIO_BASSEADDR_TO_CODE(exti_config->GPIO_Port);

    AFIO->EXTICR[reg_index] &= ~(0xFUL << pos);
    AFIO->EXTICR[reg_index] |= (uint32_t)(port_index << pos);

    // 4. Cấu hình trigger
    EXTI->IMR |= (1 << exti_line);  // Unmask interrupt

      switch (exti_config->Trigger) {
        case EXTI_TRIGGER_RISING:
            EXTI->RTSR |= (1 << exti_line);
            EXTI->FTSR &= ~(1 << exti_line);
            break;
        case EXTI_TRIGGER_FALLING:
            EXTI->FTSR |= (1 << exti_line);
            EXTI->RTSR &= ~(1 << exti_line);
            break;
        case EXTI_TRIGGER_RISING_FALLING:
            EXTI->RTSR |= (1 << exti_line);
            EXTI->FTSR |= (1 << exti_line);
            break;
      }
    // 5. Bật NVIC tương ứng
    IRQn_Type irq;
    if (exti_line <= 4)
        irq = (IRQn_Type)(EXTI0_IRQn + exti_line);
    else if (exti_line <= 9)
        irq = EXTI9_5_IRQn;
    else
        irq = EXTI15_10_IRQn;

    NVIC_SetPriority(irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                        exti_config->IRQ_PreemptionPriority,
                        exti_config->IRQ_SubPriority));
    NVIC_EnableIRQ(irq);
}

/*   void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
        // Set the priority for the specified IRQ number
        uint8_t iprx = IRQNumber / 4;
        uint8_t iprx_section = IRQNumber % 4;
        uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
        *(NVIC->IPR + iprx) |= (IRQPriority << shift_amount);
    }*/

