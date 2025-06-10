#include "stm32f1_spi_driver.h" // Include header tương ứng

/* ================================================================== */
/* Static Helper Function Prototypes (Private to this file)           */
/* ================================================================== */
static void spi_wait_for_flag(SPI_RegDef_t *pSPIx, uint32_t Flag);
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/* ================================================================== */
/* Function Implementations (APIs)                         */
/* ================================================================== */

/* Peripheral Clock Setup */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        if (pSPIx == SPI1) { RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; }
        else if (pSPIx == SPI2) { RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; }
        // Add SPI3 if applicable
    } else {
        if (pSPIx == SPI1) { RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN; }
        else if (pSPIx == SPI2) { RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN; }
        // Add SPI3 if applicable
    }
}

/* Init */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    uint32_t tempreg_cr1 = 0;
    uint32_t tempreg_cr2 = 0;

    // Bật clock trước khi cấu hình thanh ghi
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // --- Cấu hình SPI_CR1 ---
    tempreg_cr1 |= (uint32_t)pSPIHandle->SPIConfig.SPI_DeviceMode << 2;
    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
        tempreg_cr1 &= ~(1 << 15);
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        tempreg_cr1 |= (1 << 15);
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
        tempreg_cr1 &= ~(1 << 15);
        tempreg_cr1 |= (1 << 10);
    }
    tempreg_cr1 |= (uint32_t)pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;
    tempreg_cr1 |= (uint32_t)pSPIHandle->SPIConfig.SPI_DFF << 11;
    tempreg_cr1 |= (uint32_t)pSPIHandle->SPIConfig.SPI_CPOL << 1;
    tempreg_cr1 |= (uint32_t)pSPIHandle->SPIConfig.SPI_CPHA << 0;
    if (pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN) {
        tempreg_cr1 |= (1 << 9);
        tempreg_cr1 |= (1 << 8);
    } else {
        tempreg_cr1 &= ~(1 << 9);
    }
    pSPIHandle->pSPIx->CR1 = tempreg_cr1;

     // --- Cấu hình SPI_CR2 ---
    if ((pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER) && \
        (pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_DI)) {
        tempreg_cr2 |= (1 << 2); // Bật SSOE nếu là Master NSS Cứng
    }
     pSPIHandle->pSPIx->CR2 = tempreg_cr2; // Ghi CR2 (chỉ có SSOE ở đây, các bit khác = 0)

     // Khởi tạo trạng thái handle cho IT mode
     pSPIHandle->TxRxState = SPI_READY;
     pSPIHandle->pTxBuffer = NULL;
     pSPIHandle->pRxBuffer = NULL;
     pSPIHandle->TxLen = 0;
     pSPIHandle->RxLen = 0;
}

/* De-init */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
    if (pSPIx == SPI1) {
        RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    } else if (pSPIx == SPI2) {
        RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;
    }
    // Add SPI3 if needed
}

/* Data Send (Blocking) */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    if(!(pSPIx->CR1 & SPI_CR1_SPE)) { return; } // Chỉ gửi nếu SPI đang bật

    while (Len > 0) {
        spi_wait_for_flag(pSPIx, SPI_SR_TXE);
        if (pSPIx->CR1 & SPI_CR1_DFF) { // 16-bit
            if (Len < 2) break; // Cần ít nhất 2 byte
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len -= 2;
            pTxBuffer += 2; // Tăng con trỏ lên 2 byte
        } else { // 8-bit
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
				
				// Quan trọng: Sau khi ghi vào DR để truyền, một frame dữ liệu cũng sẽ được nhận vào DR (do full-duplex).
        // Phải đợi cờ RXNE (Receive Buffer Not Empty) được set và sau đó đọc DR để xóa RXNE.
        // Điều này giúp tránh lỗi OVR cho lần truyền/nhận tiếp theo.
        spi_wait_for_flag(pSPIx, SPI_SR_RXNE); // Đợi dữ liệu được nhận vào DR
        volatile uint16_t dummy_read = pSPIx->DR; // Đọc DR để xóa cờ RXNE (dữ liệu này thường là "rác")
        (void)dummy_read;                         // Tránh cảnh báo "unused variable"
    }
    // Chờ gửi xong byte cuối và bus hết bận
    spi_wait_for_flag(pSPIx, SPI_SR_TXE);
    while(pSPIx->SR & SPI_SR_BSY);
}

/* Data Receive (Blocking) */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
     if(!(pSPIx->CR1 & SPI_CR1_SPE)) { return; } // Chỉ nhận nếu SPI đang bật

    while (Len > 0) {
        // Chờ RXNE = 1
        spi_wait_for_flag(pSPIx, SPI_SR_RXNE);

        if (pSPIx->CR1 & SPI_CR1_DFF) { // 16-bit
            if (Len < 2) break;
            *((uint16_t*)pRxBuffer) = (uint16_t)pSPIx->DR;
            Len -= 2;
            pRxBuffer += 2;
        } else { // 8-bit
            *pRxBuffer = *((volatile uint8_t*)&pSPIx->DR);
            Len--;
            pRxBuffer++;
        }
    }
}

void SPI_MasterReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    if (!(pSPIx->CR1 & (1 << 6))) { /* Check SPE bit (bit 6) */
        return; 
    }

    while (Len > 0) {
        if (pSPIx->CR1 & (1 << 11)) { /* Check DFF bit (bit 11 for 16-bit) */
            // 16-bit DFF
            if (Len < 2) { // Yêu cầu Len phải chẵn
                // Tùy chọn: log lỗi hoặc xử lý khác
                break; 
            }
            spi_wait_for_flag(pSPIx, SPI_SR_TXE); // Đợi TXE trống
            pSPIx->DR = 0xFFFF; // Gửi dummy word 16-bit

            spi_wait_for_flag(pSPIx, SPI_SR_RXNE); // Đợi RXNE
            *((uint16_t*)pRxBuffer) = (uint16_t)pSPIx->DR;
            Len -= 2;
            pRxBuffer += 2;
        } else { 
            // 8-bit DFF
            spi_wait_for_flag(pSPIx, SPI_SR_TXE); // Đợi TXE trống
            pSPIx->DR = 0xFF; // Gửi dummy byte 8-bit (có thể là 0x00)

            spi_wait_for_flag(pSPIx, SPI_SR_RXNE); // Đợi RXNE
            *pRxBuffer = (uint8_t)pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
    // Đảm bảo byte dummy cuối cùng đã được truyền và bus không bận
    // trước khi có thể thay đổi trạng thái CS (thường là nhả CS)
    spi_wait_for_flag(pSPIx, SPI_SR_TXE);
    while(pSPIx->SR & SPI_SR_BSY);
}

/* Data Send (Interrupt) */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxRxState;
    if (state == SPI_READY)
    {
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;
        pSPIHandle->TxRxState = SPI_BUSY_IN_TX;
        pSPIHandle->pRxBuffer = NULL;
        pSPIHandle->RxLen = 0;

        // Đảm bảo SPI được bật trước khi bật ngắt
        if(!(pSPIHandle->pSPIx->CR1 & SPI_CR1_SPE)) {
            pSPIHandle->pSPIx->CR1 |= SPI_CR1_SPE;
        }
        // Bật ngắt TXEIE
        pSPIHandle->pSPIx->CR2 |= (1 << 7);

        state = SPI_BUSY_IN_TX;
    }
    return state;
}

/* Data Receive (Interrupt) */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxRxState;
    if (state == SPI_READY)
    {
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;
        pSPIHandle->TxRxState = SPI_BUSY_IN_RX;
        pSPIHandle->pTxBuffer = NULL;
        pSPIHandle->TxLen = 0;

         // Đảm bảo SPI được bật
         if(!(pSPIHandle->pSPIx->CR1 & SPI_CR1_SPE)) {
            pSPIHandle->pSPIx->CR1 |= SPI_CR1_SPE;
         }
        // Bật ngắt RXNEIE
        pSPIHandle->pSPIx->CR2 |= ( 1 << 6);

        state = SPI_BUSY_IN_RX;
    }
    return state;
}

/* IRQ Configuration and ISR Handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE) { NVIC_EnableIRQ((IRQn_Type)IRQNumber); }
    else { NVIC_DisableIRQ((IRQn_Type)IRQNumber); }
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
     NVIC_SetPriority((IRQn_Type)IRQNumber, IRQPriority);
}

// Hàm xử lý ngắt chung - gọi từ ISR cụ thể (vd: SPI1_IRQHandler)
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
    uint8_t check_txe = (pHandle->pSPIx->SR & SPI_SR_TXE);
    uint8_t check_txeie = (pHandle->pSPIx->CR2 & SPI_CR2_TXEIE);
    if (check_txe && check_txeie) {
        spi_txe_interrupt_handle(pHandle);
    }

    uint8_t check_rxne = (pHandle->pSPIx->SR & SPI_SR_RXNE);
    uint8_t check_rxneie = (pHandle->pSPIx->CR2 & SPI_CR2_RXNEIE);
    if (check_rxne && check_rxneie) {
        spi_rxne_interrupt_handle(pHandle);
    }

    uint8_t check_ovr = (pHandle->pSPIx->SR & SPI_SR_OVR);
    uint8_t check_errie = (pHandle->pSPIx->CR2 & SPI_CR2_ERRIE); // Kiểm tra cả bit cho phép ngắt lỗi
    if (check_ovr && check_errie) {
        spi_ovr_err_interrupt_handle(pHandle);
    }
    // Thêm xử lý lỗi khác (MODF, CRCERR) nếu ERRIE được bật và cờ tương ứng set
}

/* Other Peripheral Control APIs */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        pSPIx->CR1 |= (1 << 6);
    } else {
        // Chờ cho đến khi SPI không bận trước khi tắt
        while( pSPIx->SR & SPI_SR_BSY );
        pSPIx->CR1 &= ~(1 << 6);
    }
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if(pSPIx->SR & FlagName) {
        return SET;
    }
    return RESET;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
     uint8_t temp_dr;
     uint8_t temp_sr;
     temp_dr = *((volatile uint8_t*)&pSPIx->DR); // Đọc DR
     temp_sr = (uint8_t)pSPIx->SR;                // Đọc SR
     (void)temp_dr; // Tránh warning unused
     (void)temp_sr;
}

// Đóng giao tiếp TX (thường gọi trong callback TX_CMPLT)
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
    pSPIHandle->pSPIx->CR2 &= ~( 1 << 7); // Tắt ngắt TXEIE
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxRxState = SPI_READY;
}

// Đóng giao tiếp RX (thường gọi trong callback RX_CMPLT)
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
    pSPIHandle->pSPIx->CR2 &= ~( 1 << 6); // Tắt ngắt RXNEIE
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->TxRxState = SPI_READY;
}


/*==========================================================================================
 * Static Helper Function Implementations
 *==========================================================================================*/

static void spi_wait_for_flag(SPI_RegDef_t *pSPIx, uint32_t Flag) {
    while(!(pSPIx->SR & Flag));
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
    if (pSPIHandle->pSPIx->CR1 & SPI_CR1_DFF) { // 16-bit
        if(pSPIHandle->TxLen >= 2) {
             pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
             pSPIHandle->TxLen -= 2;
             pSPIHandle->pTxBuffer += 2; // Tăng con trỏ 2 byte
        } else if (pSPIHandle->TxLen == 1) { // Gửi nốt byte lẻ nếu DFF=16? Không chuẩn nhưng tạm xử lý
             pSPIHandle->pSPIx->DR = (*(pSPIHandle->pTxBuffer)) & 0x00FF;
             pSPIHandle->TxLen--;
             pSPIHandle->pTxBuffer++;
        }
    } else { // 8-bit
         if(pSPIHandle->TxLen > 0) {
            // Ghi 8 bit thấp vào DR (DR là 16 bit nhưng phần cứng xử lý)
            *((volatile uint8_t*)&pSPIHandle->pSPIx->DR) = *(pSPIHandle->pTxBuffer);
            pSPIHandle->TxLen--;
            pSPIHandle->pTxBuffer++;
         }
    }

    if (pSPIHandle->TxLen == 0) {
        SPI_CloseTransmission(pSPIHandle); // Gọi hàm đóng giao tiếp TX
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {
     if (pSPIHandle->pSPIx->CR1 & SPI_CR1_DFF) { // 16-bit
         if (pSPIHandle->RxLen >= 2) {
            *((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
            pSPIHandle->RxLen -= 2;
            pSPIHandle->pRxBuffer += 2;
         } else if (pSPIHandle->RxLen == 1) { // Nhận nốt byte lẻ?
             *(pSPIHandle->pRxBuffer) = (uint8_t)(pSPIHandle->pSPIx->DR & 0x00FF);
             pSPIHandle->RxLen--;
             pSPIHandle->pRxBuffer++;
         }
    } else { // 8-bit
        if(pSPIHandle->RxLen > 0) {
            *(pSPIHandle->pRxBuffer) = *((volatile uint8_t*)&pSPIHandle->pSPIx->DR);
            pSPIHandle->RxLen--;
            pSPIHandle->pRxBuffer++;
        }
    }

    if (pSPIHandle->RxLen == 0) {
        SPI_CloseReception(pSPIHandle); // Gọi hàm đóng giao tiếp RX
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {
    // Chỉ xóa lỗi OVR nếu SPI không đang truyền để tránh ảnh hưởng TX buffer
    if (pSPIHandle->TxRxState != SPI_BUSY_IN_TX) {
       SPI_ClearOVRFlag(pSPIHandle->pSPIx); // Gọi hàm xóa cờ OVR
    }
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


/*==========================================================================================
 * Application Callback (Weak Implementation)
 *==========================================================================================*/
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    // This function should be implemented by the application.
    (void)pSPIHandle; // Tránh warning unused
    (void)AppEv;      // Tránh warning unused
}
