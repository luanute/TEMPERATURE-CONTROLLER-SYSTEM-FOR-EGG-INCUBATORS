#include "stm32f1_i2c.h"


/* --- Khai báo các hàm Static Helper --- */
static void I2C_GenerateStartCondition(I2C_TypeDef* pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef* pI2Cx, uint8_t SlaveAddr_7bit);
static void I2C_ExecuteAddressPhaseRead(I2C_TypeDef* pI2Cx, uint8_t SlaveAddr_7bit);
static void I2C_ClearADDRFlag(I2C_Handle_t* pI2CHandle); // Cần handle cho logic phức tạp hơn
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2CHandle); // Đổi tên cho rõ ràng
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t* pI2CHandle); // Đổi tên cho rõ ràng
uint8_t I2C_ScanFirstSlaveAddress(I2C_TypeDef *I2Cx);
/* --- Định nghĩa các hàm API --- */

/**
 * @brief Bật hoặc tắt Clock cho ngoại vi I2C.
 */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        if (pI2Cx == I2C1) { RCC->APB1ENR |= 1<<21; }
        else if (pI2Cx == I2C2) { RCC->APB1ENR |= 1<<22; }
    } else {
        if (pI2Cx == I2C1) { RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN; }
        else if (pI2Cx == I2C2) { RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN; }
    }
}


void I2C_Init(I2C_Handle_t *pI2CHandler)
{
    uint32_t tempreg = 0;
    uint32_t pclk1_freq = 0;
    uint16_t ccr_value = 0;
   
    // 1. Bật Clock ngoại vi
		RCC->APB2ENR |= 1<<0;
    I2C_PeriClockControl(pI2CHandler->pI2Cx, ENABLE);
			
    // 2. PE 
    pI2CHandler->pI2Cx->CR1 &= ~(1<<0);
    pI2CHandler->pI2Cx->CR1 |=(1<<10);
    // 3. Cấu hình CR2: FREQ bits
    pclk1_freq = Get_PCLK1_Frequency();
		
    tempreg = (pclk1_freq / 1000000U);
    if (tempreg < 2) { /* Lỗi PCLK1 < 2MHz */ return; } // Tối thiểu cho F1
    pI2CHandler->pI2Cx->CR2 &= ~I2C_CR2_FREQ; // Xóa bits cũ
    pI2CHandler->pI2Cx->CR2 |= (tempreg & 0x3FU);

    // 4. Cấu hình OAR1: Địa chỉ thiết bị (Own Address)
    tempreg = 0;
    tempreg = (uint32_t)pI2CHandler->I2CConfig.I2C_DeviceAddress << 1; // Địa chỉ 7-bit vào ADD[7:1]
    tempreg |= (1 << 14); // Bit 14 phải là 1
    pI2CHandler->pI2Cx->OAR1 = tempreg;
   
    // 5. Cấu hình CCR: Tốc độ SCL
    tempreg = 0;
    uint32_t scl_speed = pI2CHandler->I2CConfig.I2C_SCLSpeed;
    if (scl_speed <= I2C_SCL_SPEED_SM) { // Standard Mode
        ccr_value = (uint16_t)(pclk1_freq / (2 * scl_speed));
        if(ccr_value < 4) ccr_value = 4; // Tối thiểu cho F1 SM
        tempreg |= (ccr_value & 0xFFFU);
    } else { // Fast Mode
        tempreg |= I2C_CCR_FS; // Bật Fast Mode
        if (pI2CHandler->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
            ccr_value = (uint16_t)(pclk1_freq / (3 * scl_speed));
        } else {
            tempreg |= I2C_CCR_DUTY; // Bật Duty 16/9
            ccr_value = (uint16_t)(pclk1_freq / (25 * scl_speed));
        }
        if(ccr_value < 1) ccr_value = 1; // Tối thiểu cho F1 FM
        tempreg |= (ccr_value & 0xFFFU);
    }
    pI2CHandler->pI2Cx->CCR = tempreg;
    
    // 6. Cấu hình TRISE
    tempreg = 0;
    uint32_t pclk1_mhz = pclk1_freq / 1000000U;
    if (scl_speed <= I2C_SCL_SPEED_SM) {
        tempreg = pclk1_mhz + 1;
    } else {
        tempreg = ((pclk1_mhz * 300) / 1000) + 1;
    }
    pI2CHandler->pI2Cx->TRISE = (tempreg & 0x3FU);

    // 7. Cấu hình CR1: ACK Control (sau khi đã tắt PE)
    tempreg = pI2CHandler->pI2Cx->CR1;
    if (pI2CHandler->I2CConfig.I2C_ACKControl == I2C_ACK_ENABLE) {
        tempreg |= I2C_CR1_ACK;
    } else {
        tempreg &= ~I2C_CR1_ACK;
    }
    pI2CHandler->pI2Cx->CR1 = tempreg;

    // 8. Bật PE cuối cùng
    pI2CHandler->pI2Cx->CR1 |= 1<<0;
}

/**
 * @brief Hủy khởi tạo ngoại vi I2C (reset RCC).
 */
void I2C_DeInit(I2C_TypeDef *pI2Cx)
{
    if (pI2Cx == I2C1) {
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    } else if (pI2Cx == I2C2) {
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
    }
}

/**
 * @brief Bật/Tắt ngoại vi I2C (PE bit).
 */
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE) {
        pI2Cx->CR1 |= I2C_CR1_PE;
    } else {
        pI2Cx->CR1 &= ~I2C_CR1_PE;
    }
}

/**
 * @brief Gửi dữ liệu ở chế độ Master (Blocking).
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandler, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr_7bit, uint8_t Sr)
{
    // 1. Tạo START condition
    I2C_GenerateStartCondition(pI2CHandler->pI2Cx);
		//I2C_ManageAcking(pI2CHandler->pI2Cx, ENABLE);

    // 2. Chờ START được tạo (SB=1)
    while (!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_SB));

    // 3. Gửi địa chỉ Slave + W bit
    I2C_ExecuteAddressPhaseWrite(pI2CHandler->pI2Cx, SlaveAddr_7bit);

    // 4. Chờ địa chỉ được gửi (ADDR=1)
    while (!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_ADDR));

    // 5. Xóa cờ ADDR
    I2C_ClearADDRFlag(pI2CHandler);

    // 6. Gửi dữ liệu
    while (Len > 0) {
        // Chờ TXE=1 (DR trống)
        while (!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_TXE));
        pI2CHandler->pI2Cx->DR = *pTxbuffer;
        pTxbuffer++;
        Len--;
    }

    // 7. Chờ TXE=1 và BTF=1 trước khi gửi STOP (đảm bảo byte cuối cùng đã truyền)
    while (!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_TXE));
    while (!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_BTF));

    // 8. Gửi STOP condition nếu Sr=DISABLE
    if (Sr == I2C_SR_DISABLE) {
        I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
    }
}

/**
 * @brief Nhận dữ liệu ở chế độ Master (Blocking).
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandler, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr_7bit, uint8_t Sr)
{
    // 1. Tạo START condition
    I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

    // 2. Chờ START được tạo (SB=1)
    while (!I2C_GetFlagStatus(pI2CHandler->pI2Cx, 1<<0));

    // 3. Gửi địa chỉ Slave + R bit
    I2C_ExecuteAddressPhaseRead(pI2CHandler->pI2Cx, SlaveAddr_7bit);

    // 4. Chờ địa chỉ được gửi (ADDR=1)
    while (!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_ADDR));
    (void)pI2CHandler->pI2Cx->SR1;                 // Đọc SR1
    (void)pI2CHandler->pI2Cx->SR2;                 // Đọc SR2 để xóa cờ ADDR
    // 5. Nhận dữ liệu
    if (Len == 1) {
        // --- Nhận 1 byte ---
        // Tắt ACK
        I2C_ManageAcking(pI2CHandler->pI2Cx, DISABLE);
        // Xóa ADDR
        I2C_ClearADDRFlag(pI2CHandler);
        // Chờ RXNE=1
        while (!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_RXNE));
        // Gửi STOP nếu cần
        if (Sr == I2C_SR_DISABLE) {
            I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
        }
        // Đọc dữ liệu
        *pRxBuffer = pI2CHandler->pI2Cx->DR;
    }
    else if (Len > 1) {
        // --- Nhận nhiều byte ---
        // Xóa ADDR
        I2C_ClearADDRFlag(pI2CHandler);

        // Đọc Len-1 bytes đầu tiên
        for (uint32_t i = Len; i > 0; i--) {
             // Chờ RXNE = 1
            while (!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_RXNE));

            if (i == 2) { // Khi chỉ còn 2 byte cần đọc
                // Tắt ACK (để gửi NACK cho byte cuối cùng)
                I2C_ManageAcking(pI2CHandler->pI2Cx, DISABLE);
                // Gửi STOP nếu cần (trước khi đọc byte cuối)
                if (Sr == I2C_SR_DISABLE) {
                    I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
                }
            }
            // Đọc dữ liệu vào buffer
            *pRxBuffer = pI2CHandler->pI2Cx->DR;
            pRxBuffer++; // Di chuyển con trỏ buffer
        }
    }

    // Khôi phục trạng thái ACK nếu nó được bật trong cấu hình
    if (pI2CHandler->I2CConfig.I2C_ACKControl == I2C_ACK_ENABLE) {
        I2C_ManageAcking(pI2CHandler->pI2Cx, ENABLE);
    }
}

/**
 * @brief Gửi dữ liệu ở chế độ Master (Interrupt).
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandler, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr_7bit, uint8_t Sr)
{
    uint8_t busystate = pI2CHandler->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
        pI2CHandler->pTxBuffer = pTxbuffer;
        pI2CHandler->TxLen = Len;
        pI2CHandler->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandler->DevAddr = SlaveAddr_7bit;
        pI2CHandler->Sr = Sr;

        // Tạo START condition (SB interrupt sẽ được kích hoạt)
        I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

        // Bật các bit cho phép ngắt (Event, Buffer, Error)
        pI2CHandler->pI2Cx->CR2 |= I2C_CR2_ITEVTEN;
        pI2CHandler->pI2Cx->CR2 |= I2C_CR2_ITBUFEN;
        pI2CHandler->pI2Cx->CR2 |= I2C_CR2_ITERREN;
    }
    return busystate;
}

/**
 * @brief Nhận dữ liệu ở chế độ Master (Interrupt).
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandler, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr_7bit, uint8_t Sr)
{
    uint8_t busystate = pI2CHandler->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
        pI2CHandler->pRxBuffer = pRxBuffer;
        pI2CHandler->RxLen = Len;
        pI2CHandler->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandler->RxSize = Len; // Lưu tổng kích thước
        pI2CHandler->DevAddr = SlaveAddr_7bit;
        pI2CHandler->Sr = Sr;

        // Tạo START condition (SB interrupt sẽ được kích hoạt)
        I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

        // Bật các bit cho phép ngắt (Event, Buffer, Error)
        pI2CHandler->pI2Cx->CR2 |= I2C_CR2_ITEVTEN;
        pI2CHandler->pI2Cx->CR2 |= I2C_CR2_ITBUFEN;
        pI2CHandler->pI2Cx->CR2 |= I2C_CR2_ITERREN;
    }
    return busystate;
}

/**
 * @brief Gửi 1 byte dữ liệu từ Slave.
 */
void I2C_SlaveSendData(I2C_TypeDef *pI2C, uint8_t data)
{
    pI2C->DR = data;
}

/**
 * @brief Nhận 1 byte dữ liệu tại Slave.
 */
uint8_t I2C_SlaveReceiveData(I2C_TypeDef *pI2C)
{
    return (uint8_t)pI2C->DR;
}

/**
 * @brief Bật/tắt các ngắt I2C cho Slave events.
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_TypeDef* pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE) {
        pI2Cx->CR2 |= I2C_CR2_ITEVTEN;
        pI2Cx->CR2 |= I2C_CR2_ITBUFEN;
        pI2Cx->CR2 |= I2C_CR2_ITERREN;
    } else {
        pI2Cx->CR2 &= ~I2C_CR2_ITEVTEN;
        pI2Cx->CR2 &= ~I2C_CR2_ITBUFEN;
        pI2Cx->CR2 &= ~I2C_CR2_ITERREN;
    }
}


/* --- Các hàm cấu hình ngắt NVIC (Cần kiểm tra lại NO_PR_BITS_IMPLEMENTED cho F1) --- */
// Số bit priority được sử dụng thực tế trên F1 (thường là 4)
#define NO_PR_BITS_IMPLEMENTED          4

// Các thanh ghi NVIC base address (Thường được định nghĩa trong CMSIS core_cm3.h)
#define NVIC_ISER_BASEADDR              ((volatile uint32_t*)0xE000E100)
#define NVIC_ICER_BASEADDR              ((volatile uint32_t*)0xE000E180)
#define NVIC_PR_BASE_ADDR               ((volatile uint32_t*)0xE000E400)


void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi)
{
    volatile uint32_t *pNVIC_ISER = NVIC_ISER_BASEADDR;
    volatile uint32_t *pNVIC_ICER = NVIC_ICER_BASEADDR;

    if (EnorDi == ENABLE) {
        *(pNVIC_ISER + (IRQNumber / 32)) |= (1 << (IRQNumber % 32));
    } else {
         *(pNVIC_ICER + (IRQNumber / 32)) |= (1 << (IRQNumber % 32));
    }
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQpriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) &= ~(0xFF << shift_amount); // Clear priority field
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQpriority << shift_amount);
}


/* --- Các hàm xử lý ngắt --- */

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandler)
{
    uint32_t temp1, temp2, temp3;
    I2C_TypeDef *pI2Cx = pI2CHandler->pI2Cx;

    // Kiểm tra các cờ cho phép ngắt Event và Buffer
    temp1 = pI2Cx->CR2 & I2C_CR2_ITEVTEN;
    temp2 = pI2Cx->CR2 & I2C_CR2_ITBUFEN;

    // 1. Xử lý ngắt SB (Start bit - Chỉ Master)
    temp3 = pI2Cx->SR1 & I2C_SR1_SB;
    if (temp1 && temp3) {
        // Ngắt SB xảy ra: Gửi địa chỉ Slave
        if (pI2CHandler->TxRxState == I2C_BUSY_IN_TX) {
            I2C_ExecuteAddressPhaseWrite(pI2Cx, pI2CHandler->DevAddr);
        } else if (pI2CHandler->TxRxState == I2C_BUSY_IN_RX) {
            I2C_ExecuteAddressPhaseRead(pI2Cx, pI2CHandler->DevAddr);
        }
        // SB được xóa khi ghi địa chỉ vào DR
    }

    // 2. Xử lý ngắt ADDR (Address sent/matched)
    temp3 = pI2Cx->SR1 & I2C_SR1_ADDR;
    if (temp1 && temp3) {
        // Ngắt ADDR xảy ra: Xóa cờ ADDR
        I2C_ClearADDRFlag(pI2CHandler);
    }

    // 3. Xử lý ngắt BTF (Byte Transfer Finished)
    temp3 = pI2Cx->SR1 & I2C_SR1_BTF;
    if (temp1 && temp3) {
        if (pI2CHandler->TxRxState == I2C_BUSY_IN_TX) {
            // Nếu đang truyền và BTF=1, kiểm tra TXE=1
            if (pI2Cx->SR1 & I2C_SR1_TXE) {
                // Đảm bảo TxLen == 0 (đã gửi hết)
                if (pI2CHandler->TxLen == 0) {
                    // Tạo STOP nếu cần
                    if (pI2CHandler->Sr == I2C_SR_DISABLE) {
                        I2C_GenerateStopCondition(pI2Cx);
                    }
                    // Đóng giao dịch truyền
                    I2C_CloseSendData(pI2CHandler);
                    // Gọi callback báo truyền hoàn tất
                    I2C_ApplicationEventCallback(pI2CHandler, I2C_EV_TX_CMPLT);
                }
            }
        } else if (pI2CHandler->TxRxState == I2C_BUSY_IN_RX) {
            // BTF không dùng nhiều trong quá trình nhận thông thường
        }
    }

    // 4. Xử lý ngắt STOPF (Stop detection - Chỉ Slave)
    temp3 = pI2Cx->SR1 & I2C_SR1_STOPF;
    if (temp1 && temp3) {
        // Xóa cờ STOPF: đọc SR1 (đã làm) sau đó ghi vào CR1
        pI2Cx->CR1 |= 0x0000; // Ghi bất kỳ vào CR1 sẽ xóa STOPF
        // Gọi callback báo STOP
        I2C_ApplicationEventCallback(pI2CHandler, I2C_EV_STOP);
    }

    // 5. Xử lý ngắt TXE (Transmit buffer empty)
    temp3 = pI2Cx->SR1 & I2C_SR1_TXE;
    if (temp1 && temp2 && temp3) { // Cần cả ITEVTEN và ITBUFEN
         // Kiểm tra chế độ Master/Slave
        if (pI2Cx->SR2 & I2C_SR2_MSL) { // Master mode
             if (pI2CHandler->TxRxState == I2C_BUSY_IN_TX) {
                I2C_MasterHandleTXEInterrupt(pI2CHandler);
            }
        } else { // Slave mode
            // Nếu Slave đang ở chế độ truyền (TRA=1)
            if (pI2Cx->SR2 & I2C_SR2_TRA) {
                 I2C_ApplicationEventCallback(pI2CHandler, I2C_EV_DATA_REQ); // Yêu cầu dữ liệu từ App
            }
        }
    }

    // 6. Xử lý ngắt RXNE (Receive buffer not empty)
    temp3 = pI2Cx->SR1 & I2C_SR1_RXNE;
    if (temp1 && temp2 && temp3) { // Cần cả ITEVTEN và ITBUFEN
        // Kiểm tra chế độ Master/Slave
        if (pI2Cx->SR2 & I2C_SR2_MSL) { // Master mode
            if (pI2CHandler->TxRxState == I2C_BUSY_IN_RX) {
                I2C_MasterHandleRXNEInterrupt(pI2CHandler);
            }
        } else { // Slave mode
             // Nếu Slave đang ở chế độ nhận (TRA=0)
            if (!(pI2Cx->SR2 & I2C_SR2_TRA)) {
                I2C_ApplicationEventCallback(pI2CHandler, I2C_EV_DATA_RCV); // Báo App có dữ liệu
            }
        }
    }
}


/**
 * @brief Xử lý lỗi I2C trong ngắt.
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandler)
{
    uint32_t temp1, temp2;
    I2C_TypeDef *pI2Cx = pI2CHandler->pI2Cx;

    // Kiểm tra cờ cho phép ngắt lỗi ITERREN
    temp2 = (pI2Cx->CR2) & I2C_CR2_ITERREN;

    // Kiểm tra lỗi Bus Error (BERR)
    temp1 = (pI2Cx->SR1) & I2C_SR1_BERR;
    if (temp1 && temp2) {
        pI2Cx->SR1 &= ~I2C_SR1_BERR; // Xóa cờ BERR
        I2C_ApplicationEventCallback(pI2CHandler, I2C_ERROR_BERR);
    }

    // Kiểm tra lỗi Arbitration Lost (ARLO)
    temp1 = (pI2Cx->SR1) & I2C_SR1_ARLO;
    if (temp1 && temp2) {
        pI2Cx->SR1 &= ~I2C_SR1_ARLO; // Xóa cờ ARLO
        I2C_ApplicationEventCallback(pI2CHandler, I2C_ERROR_ARLO);
    }

    // Kiểm tra lỗi Acknowledge Failure (AF)
    temp1 = (pI2Cx->SR1) & I2C_SR1_AF;
    if (temp1 && temp2) {
        pI2Cx->SR1 &= ~I2C_SR1_AF; // Xóa cờ AF
        I2C_ApplicationEventCallback(pI2CHandler, I2C_ERROR_AF);
    }

    // Kiểm tra lỗi Overrun/Underrun (OVR)
    temp1 = (pI2Cx->SR1) & I2C_SR1_OVR;
    if (temp1 && temp2) {
        pI2Cx->SR1 &= ~I2C_SR1_OVR; // Xóa cờ OVR
        I2C_ApplicationEventCallback(pI2CHandler, I2C_ERROR_OVR);
    }

    // Kiểm tra lỗi Timeout (TIMEOUT) - Chỉ có trên một số dòng F1/F4 có SMBus
    // Trên F103 chuẩn không có cờ này, cần kiểm tra lại datasheet F103 cụ thể.
    // Giả sử F103 không có cờ này, comment đoạn sau:
    /*
    temp1 = (pI2Cx->SR1) & I2C_SR1_TIMEOUT;
    if(temp1  && temp2)
    {
        pI2Cx->SR1 &= ~I2C_SR1_TIMEOUT; // Xóa cờ TIMEOUT
        I2C_ApplicationEventCallback(pI2CHandler,I2C_ERROR_TIMEOUT);
    }
    */
}


/* --- Các hàm Static Helper --- */

static void I2C_GenerateStartCondition(I2C_TypeDef* pI2Cx)
{
    pI2Cx->CR1 |= I2C_CR1_START;
}

void I2C_GenerateStopCondition(I2C_TypeDef* pI2Cx)
{
    pI2Cx->CR1 |= I2C_CR1_STOP;
}

uint8_t I2C_GetFlagStatus(I2C_TypeDef *pI2Cx, uint32_t FlagName)
{
    // FlagName ở đây là bit mask chuẩn từ stm32f1xx.h
    if (pI2Cx->SR1 & FlagName) {
        return 1; // SET
    }
    return 0; // RESET
}

static void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef* pI2Cx, uint8_t SlaveAddr_7bit)
{
    uint8_t addr_8bit = (SlaveAddr_7bit << 1); // Dịch trái 1 bit
    addr_8bit &= ~(1); // Bit 0 = 0 (Write)
    pI2Cx->DR = addr_8bit;
}

static void I2C_ExecuteAddressPhaseRead(I2C_TypeDef* pI2Cx, uint8_t SlaveAddr_7bit)
{
     uint8_t addr_8bit = (SlaveAddr_7bit << 1); // Dịch trái 1 bit
     addr_8bit |= 1; // Bit 0 = 1 (Read)
     pI2Cx->DR = addr_8bit;
}

void I2C_ManageAcking(I2C_TypeDef* pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == I2C_ACK_ENABLE) {
        pI2Cx->CR1 |= I2C_CR1_ACK;
    } else {
        pI2Cx->CR1 &= ~I2C_CR1_ACK;
    }
}

static void I2C_ClearADDRFlag(I2C_Handle_t* pI2CHandle)
{
    // Quy trình xóa ADDR trên F1: đọc SR1 sau đó đọc SR2
    uint32_t dummy_read;
    // Xử lý đặc biệt khi nhận 1 byte (theo RM0008)
    if((pI2CHandle->pI2Cx->SR2 & I2C_SR2_MSL) && // Master mode
       (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) && // Receiving
       (pI2CHandle->RxSize == 1))
    {
        // Khi nhận 1 byte, phải tắt ACK *trước* khi xóa ADDR
        I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
        // Sau đó mới đọc SR1, SR2 để xóa ADDR
        dummy_read = pI2CHandle->pI2Cx->SR1;
        dummy_read = pI2CHandle->pI2Cx->SR2;
        (void)dummy_read; // Tránh warning unused variable
    } else {
        // Các trường hợp khác (Master Tx, Master Rx > 1 byte, Slave)
        dummy_read = pI2CHandle->pI2Cx->SR1;
        dummy_read = pI2CHandle->pI2Cx->SR2;
        (void)dummy_read;
    }
}


// Các hàm xử lý ngắt cho Master TXE và RXNE
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2CHandle)
{
    if (pI2CHandle->TxLen > 0) {
        // Gửi byte tiếp theo
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
        pI2CHandle->TxLen--;
        pI2CHandle->pTxBuffer++;
    }
    // Nếu TxLen == 0, ngắt TXE sẽ tự động bị vô hiệu hóa bởi phần cứng
    // hoặc sẽ chờ BTF để kết thúc giao dịch trong EV_IRQHandling
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t* pI2CHandle)
{
     // Đọc byte vào buffer
     if(pI2CHandle->RxLen > 0) { // Kiểm tra lại để chắc chắn
         // Xử lý khi nhận 2 byte cuối cùng
         if (pI2CHandle->RxLen == 2) {
             // Tắt ACK trước khi đọc byte áp chót
             // Việc này sẽ làm phần cứng gửi NACK sau khi byte cuối cùng được nhận
             I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
         }

         // Đọc dữ liệu từ DR
         *pI2CHandle->pRxBuffer = (uint8_t)pI2CHandle->pI2Cx->DR;
         pI2CHandle->pRxBuffer++;
         pI2CHandle->RxLen--;
     }

    // Nếu đã nhận đủ byte
    if (pI2CHandle->RxLen == 0) {
        // Tạo STOP nếu cần
        if (pI2CHandle->Sr == I2C_SR_DISABLE) {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }
        // Đóng giao dịch nhận
        I2C_CloseReceiveData(pI2CHandle);
        // Gọi callback báo nhận hoàn tất
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
}


// Hàm đóng giao dịch truyền/nhận
void I2C_CloseSendData(I2C_Handle_t* pI2C_Handler)
{
    // Tắt các cờ cho phép ngắt liên quan
    pI2C_Handler->pI2Cx->CR2 &= ~I2C_CR2_ITBUFEN;
    pI2C_Handler->pI2Cx->CR2 &= ~I2C_CR2_ITEVTEN;
    // Không cần tắt ITERREN trừ khi muốn dừng xử lý lỗi

    // Reset trạng thái handle
    pI2C_Handler->TxRxState = I2C_READY;
    pI2C_Handler->pTxBuffer = NULL;
    pI2C_Handler->TxLen = 0;
}

void I2C_CloseReceiveData(I2C_Handle_t* pI2C_Handler)
{
    // Tắt các cờ cho phép ngắt liên quan
    pI2C_Handler->pI2Cx->CR2 &= ~I2C_CR2_ITBUFEN;
    pI2C_Handler->pI2Cx->CR2 &= ~I2C_CR2_ITEVTEN;
    // Không cần tắt ITERREN trừ khi muốn dừng xử lý lỗi

    // Reset trạng thái handle
    pI2C_Handler->TxRxState = I2C_READY;
    pI2C_Handler->pRxBuffer = NULL;
    pI2C_Handler->RxLen = 0;
    pI2C_Handler->RxSize = 0;

    // Đảm bảo ACK được bật lại nếu cấu hình yêu cầu
    if (pI2C_Handler->I2CConfig.I2C_ACKControl == I2C_ACK_ENABLE) {
        I2C_ManageAcking(pI2C_Handler->pI2Cx, ENABLE);
    }
}

uint8_t I2C_ScanFirstSlaveAddress(I2C_TypeDef *I2Cx)
{
    for (uint8_t addr = 1; addr < 127; addr++) {
        // 1. START
        I2Cx->CR1 |= I2C_CR1_START;
        while (!(I2Cx->SR1 & I2C_SR1_SB));

        // 2. Gửi địa chỉ
        I2Cx->DR = (addr << 1); // Write bit = 0
        while (!(I2Cx->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)));

        if (I2Cx->SR1 & I2C_SR1_ADDR) {
            // Có ACK → slave tồn tại
            (void)I2Cx->SR1;
            (void)I2Cx->SR2; // Clear ADDR flag

            I2Cx->CR1 |= I2C_CR1_STOP;
            return addr;
        }

        // Clear AF nếu NACK
        I2Cx->SR1 &= ~I2C_SR1_AF;

        // STOP
        I2Cx->CR1 |= I2C_CR1_STOP;

        // Delay ngắn nếu cần
        for (volatile int i = 0; i < 1000; i++);
    }

    return 0; // Không tìm thấy
}


/* --- Callback mặc định (weak) --- */
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandler, uint8_t ApplicationEvent)
{
    // Hàm này nên được người dùng định nghĩa lại (override) trong file ứng dụng
    // để xử lý các sự kiện như truyền/nhận hoàn tất, lỗi, yêu cầu/nhận dữ liệu slave...
}