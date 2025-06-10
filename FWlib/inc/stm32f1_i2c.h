#ifndef STM32F1_I2C_H
#define STM32F1_I2C_H
#include "stm32f10x.h"
#include "get_clk.h"
#include <stdint.h>    // Thêm stdint.h nếu cần thiết
#include <stddef.h>
/*****************************************************************************************
 * I2C configuration and handler struct
 * I2C user configuration
 *****************************************************************************************/
/*
 * Cấu trúc cấu hình cho ngoại vi I2Cx
 */
typedef struct
{
    uint32_t    I2C_SCLSpeed;       /*!< Tốc độ SCL. Dùng các hằng số @I2C_SCLSpeed */
    uint8_t     I2C_DeviceAddress;  /*!< Địa chỉ thiết bị riêng (7-bit) nếu hoạt động như Slave */
    uint8_t     I2C_ACKControl;     /*!< Bật/tắt ACK. Dùng các hằng số @I2C_ACKControl */
    uint8_t     I2C_FMDutyCycle;    /*!< Duty Cycle cho Fast Mode. Dùng các hằng số @I2C_FMDutyCycle */
} I2C_Config_t;

/*
 * Cấu trúc Handle cho ngoại vi I2Cx
 */
typedef struct
{
    I2C_TypeDef* pI2Cx;          /*!< Giữ địa chỉ base của ngoại vi I2Cx (I2C1, I2C2) */
    I2C_Config_t    I2CConfig;      /*!< Giữ cấu hình I2C */
    uint8_t         *pTxBuffer;     /*!< Địa chỉ buffer truyền của ứng dụng */
    uint8_t         *pRxBuffer;     /*!< Địa chỉ buffer nhận của ứng dụng */
    uint32_t        TxLen;          /*!< Độ dài dữ liệu truyền */
    uint32_t        RxLen;          /*!< Độ dài dữ liệu nhận */
    uint8_t         TxRxState;      /*!< Trạng thái giao tiếp (@I2C_Application_States) */
    uint8_t         DevAddr;        /*!< Địa chỉ Slave (7-bit) */
    uint32_t        RxSize;         /*!< Tổng số byte cần nhận */
    uint8_t         Sr;             /*!< Lưu trữ giá trị Repeated Start (@I2C_Sr) */
} I2C_Handle_t;

/*
 * @I2C_Application_States - Trạng thái ứng dụng I2C
 */
#define I2C_READY           0
#define I2C_BUSY_IN_RX      1
#define I2C_BUSY_IN_TX      2

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM    100000U // 100kHz
#define I2C_SCL_SPEED_FM4K  400000U // 400kHz (Kiểm tra PCLK1 có đủ cao không)
#define I2C_SCL_SPEED_FM2K  200000U // 200kHz (Ít dùng)

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE      1
#define I2C_ACK_DISABLE     0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2       0 // Tlow/Thigh = 2
#define I2C_FM_DUTY_16_9    1 // Tlow/Thigh = 16/9 (Chỉ hợp lệ nếu PCLK1 đủ cao)

/*
 * @I2C_Sr - Repeated Start Control
 */
#define I2C_SR_DISABLE      0 // Gửi STOP sau giao dịch
#define I2C_SR_ENABLE       1 // Không gửi STOP (Repeated Start)

/*****************************************************************************************
 * Các định nghĩa cờ trạng thái liên quan đến I2C (Sử dụng trực tiếp từ stm32f1xx.h)
 * Ví dụ: I2C_SR1_SB, I2C_SR1_ADDR, I2C_SR1_TXE, I2C_SR1_RXNE, I2C_SR1_BTF,...
 * I2C_SR2_MSL, I2C_SR2_BUSY,...
 * I2C_CR1_PE, I2C_CR1_START, I2C_CR1_STOP, I2C_CR1_ACK,...
 * I2C_CR2_ITERREN, I2C_CR2_ITEVTEN, I2C_CR2_ITBUFEN,...
 *****************************************************************************************/


/*
 * Các sự kiện/lỗi có thể xảy ra của ứng dụng I2C
 */
#define I2C_EV_TX_CMPLT     0
#define I2C_EV_RX_CMPLT     1
#define I2C_EV_STOP         2
#define I2C_ERROR_BERR      3
#define I2C_ERROR_ARLO      4
#define I2C_ERROR_AF        5
#define I2C_ERROR_OVR       6
#define I2C_ERROR_TIMEOUT   7
#define I2C_EV_DATA_REQ     8  // Slave yêu cầu Master gửi dữ liệu (Slave Transmitter)
#define I2C_EV_DATA_RCV     9  // Slave nhận được dữ liệu từ Master (Slave Receiver)


/*****************************************************************************************
 * APIs được hỗ trợ bởi driver này
 * Xem định nghĩa hàm để biết thêm chi tiết
 *****************************************************************************************/
/*
 * Cài đặt Clock ngoại vi
 */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi);

/*
 * Khởi tạo và Hủy khởi tạo
 */
void I2C_Init(I2C_Handle_t *pI2CHandler);
void I2C_DeInit(I2C_TypeDef *pI2Cx); // Thay đổi tham số để giống F1 DeInit hơn

/*
 * Gửi và Nhận dữ liệu (Blocking)
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandler, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr_7bit, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandler, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr_7bit, uint8_t Sr);

/*
 * Gửi và Nhận dữ liệu (Interrupt)
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandler, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr_7bit, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandler, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr_7bit, uint8_t Sr);

/*
 * Gửi và Nhận dữ liệu (Slave)
 */
void I2C_SlaveSendData(I2C_TypeDef *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_TypeDef *pI2C);

/*
 * Cấu hình IRQ và xử lý ISR
 */
void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQpriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandler);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandler);

/*
 * Các API điều khiển ngoại vi khác
 */
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_TypeDef *pI2Cx, uint32_t FlagName); // Hàm kiểm tra cờ chung
void I2C_ManageAcking(I2C_TypeDef* pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_TypeDef* pI2Cx);

void I2C_CloseSendData(I2C_Handle_t* pI2C_Handler);
void I2C_CloseReceiveData(I2C_Handle_t* pI2C_Handler);

void I2C_SlaveEnableDisableCallbackEvents(I2C_TypeDef* pI2Cx, uint8_t EnorDi); // Đổi tên cho rõ ràng
uint8_t I2C_ScanFirstSlaveAddress(I2C_TypeDef *I2Cx);
/*
 * Callback của ứng dụng (Cần được định nghĩa bởi người dùng - weak function)
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandler, uint8_t ApplicationEvent);

/*
 * Hàm lấy PCLK1 - Cần được định nghĩa bởi người dùng
 */
extern uint32_t RCC_GetPCLK1Value(void);

/*
 * Định nghĩa ENABLE/DISABLE nếu chưa có
 */
#ifndef ENABLE
#define ENABLE 1
#endif
#ifndef DISABLE
#define DISABLE 0
#endif



#endif /* __STM32F1_H */

