#include "i2c.h"

///********************************* Defines ************************************/

//#define WAIT_FOR_FLAG(flag, value, timeout, errorcode)                        \
//    I2CTimeout = timeout;                                                     \
//    while (I2C_GetFlagStatus(SENSORS_I2C, flag) != value) {                   \
//        if ((I2CTimeout--) == 0) return I2Cx_TIMEOUT_UserCallback(errorcode); \
//    }

//#define CLEAR_ADDR_BIT                               \
//    I2C_ReadRegister(SENSORS_I2C, I2C_Register_SR1); \
//    I2C_ReadRegister(SENSORS_I2C, I2C_Register_SR2);

///********************************* Globals ************************************/

///********************************* Prototypes *********************************/
//static unsigned long ST_Sensors_I2C_WriteRegister(
//    unsigned char Address, unsigned char RegisterAddr,
//    unsigned short RegisterLen, const unsigned char *RegisterValue);
//static unsigned long ST_Sensors_I2C_ReadRegister(unsigned char Address,
//                                                 unsigned char RegisterAddr,
//                                                 unsigned short RegisterLen,
//                                                 unsigned char *RegisterValue);
//static unsigned long ST_Sensors_I2C_WriteNoRegister(unsigned char Address,
//                                                    unsigned char RegisterAddr);
//static unsigned long ST_Sensors_I2C_ReadNoRegister(
//    unsigned char Address, unsigned short RegisterLen,
//    unsigned char *RegisterValue);

///*******************************  Function ************************************/

//	/**
// * @brief  超时回调函数，检测I2C标志超时调用本函数，初始化I2C重新检测。
// * @param  错误代码，每个调用有独立的编号，便于查找错误
// * @retval 默认返回1，表示I2C检测出错
// */
//static uint32_t I2Cx_TIMEOUT_UserCallback(char value) {
//    /* The following code allows I2C error recovery and return to normal
//       communication if the error source doesn抰 still exist (ie. hardware
//       issue..) */
//    I2C_InitTypeDef I2C_InitStructure;

//    I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
//    I2C_SoftwareResetCmd(SENSORS_I2C, ENABLE);
//    I2C_SoftwareResetCmd(SENSORS_I2C, DISABLE);

//    I2C_DeInit(SENSORS_I2C);
//    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//    I2C_InitStructure.I2C_OwnAddress1 = I2C_OWN_ADDRESS;
//    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;

//    /* Enable the I2C peripheral */
//    I2C_Cmd(SENSORS_I2C, ENABLE);

//    /* Initialize the I2C peripheral */
//    I2C_Init(SENSORS_I2C, &I2C_InitStructure);

//    I2C_ERROR("I2C callback error code = %d", value);

//    return 1;
//}

///**
// * @brief  写寄存器(多次尝试)，这是提供给上层的接口
// * @param  slave_addr: 从机地址
// * @param 	reg_addr:寄存器地址
// * @param len：写入的长度
// *	@param data_ptr:指向要写入的数据
// * @retval 正常为0，不正常为非0
// */
//int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,
//                              unsigned short len,
//                              const unsigned char *data_ptr) {
//    char retries = 0;
//    int ret = 0;
//    unsigned short retry_in_mlsec = Get_I2C_Retry();

//tryWriteAgain:
//    ret = 0;
//    ret = ST_Sensors_I2C_WriteRegister(slave_addr, reg_addr, len, data_ptr);

//    if (ret && retry_in_mlsec) {
//        if (retries++ > 4) return ret;
//        Delay(retry_in_mlsec);
//        goto tryWriteAgain;
//    }
//    return ret;
//}

///**
// * @brief  读寄存器(多次尝试)，这是提供给上层的接口
// * @param  slave_addr: 从机地址
// * @param 	reg_addr:寄存器地址
// * @param len：要读取的长度
// *	@param data_ptr:指向要存储数据的指针
// * @retval 正常为0，不正常为非0
// */
//int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr,
//                             unsigned short len, unsigned char *data_ptr) {
//    char retries = 0;
//    int ret = 0;
//    unsigned short retry_in_mlsec = Get_I2C_Retry();

//tryReadAgain:
//    ret = 0;
//    ret = ST_Sensors_I2C_ReadRegister(slave_addr, reg_addr, len, data_ptr);

//    if (ret && retry_in_mlsec) {
//        if (retries++ > 4) return ret;

//        Delay(retry_in_mlsec);
//        goto tryReadAgain;
//    }
//    return ret;
//}

///**
// * @brief  写寄存器(单次尝试)，这是底层I2C接口
// * @param  slave_addr: 从机地址
// * @param 	reg_addr:寄存器地址
// * @param len：写入的长度
// *	@param data_ptr:指向要写入的数据
// * @retval 正常为0，不正常为非0
// */
//static unsigned long ST_Sensors_I2C_WriteRegister(
//    unsigned char Address, unsigned char RegisterAddr,
//    unsigned short RegisterLen, const unsigned char *RegisterValue) {
//    uint32_t result = 0;
//    uint32_t i = 0;  // i = RegisterLen;
//    __IO uint32_t I2CTimeout = I2Cx_LONG_TIMEOUT;

//    //  RegisterValue = RegisterValue + (RegisterLen - 1);

//    /* Wait for the busy flag to be cleared */
//    WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 1);

//    /* Start the config sequence */
//    I2C_GenerateSTART(SENSORS_I2C, ENABLE);

//    /* Wait for the start bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 2);

//    /* Transmit the slave address and enable writing operation */
//    I2C_Send7bitAddress(SENSORS_I2C, (Address << 1), I2C_Direction_Transmitter);

//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 3);

//    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
//    CLEAR_ADDR_BIT

//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 4);

//    /* Transmit the first address for write operation */
//    I2C_SendData(SENSORS_I2C, RegisterAddr);

//    for (i = 0; i < (RegisterLen); i++) {
//        /* Wait for address bit to be set */
//        WAIT_FOR_FLAG(I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);

//        /* Prepare the register value to be sent */
//        I2C_SendData(SENSORS_I2C, RegisterValue[i]);
//    }

//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 6);

//    /* End the configuration sequence */
//    I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

//    /* Return the verifying value: 0 (Passed) or 1 (Failed) */
//    return result;
//}

///**
// * @brief  读寄存器(单次尝试)，这是底层I2C接口
// * @param  slave_addr: 从机地址
// * @param 	reg_addr:寄存器地址
// * @param len：要读取的长度
// *	@param data_ptr:指向要存储数据的指针
// * @retval 正常为0，不正常为非0
// */
//static unsigned long ST_Sensors_I2C_ReadRegister(unsigned char Address,
//                                                 unsigned char RegisterAddr,
//                                                 unsigned short RegisterLen,
//                                                 unsigned char *RegisterValue) {
//    uint32_t i = 0, result = 0;
//    __IO uint32_t I2CTimeout = I2Cx_LONG_TIMEOUT;

//    /* Wait for the busy flag to be cleared */
//    WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 7);

//    /* Start the config sequence */
//    I2C_GenerateSTART(SENSORS_I2C, ENABLE);

//    /* Wait for the start bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 8);

//    /* Transmit the slave address and enable writing operation */
//    I2C_Send7bitAddress(SENSORS_I2C, (Address << 1), I2C_Direction_Transmitter);

//    /* Wait for the start bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 9);

//    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
//    CLEAR_ADDR_BIT;

//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 10);

//    /* Transmit the register address to be read */
//    I2C_SendData(SENSORS_I2C, RegisterAddr);

//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 11);

//    /*!< Send START condition a second time */
//    I2C_GenerateSTART(SENSORS_I2C, ENABLE);

//    /* Wait for the start bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 12);

//    /*!< Send address for read */
//    I2C_Send7bitAddress(SENSORS_I2C, (Address << 1), I2C_Direction_Receiver);

//    /* Wait for the start bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 13);

//    if (RegisterLen == 1) {
//        /*!< Disable Acknowledgment */
//        I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);

//        /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
//        CLEAR_ADDR_BIT;

//        /*!< Send STOP Condition */
//        I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

//        /* Wait for the RXNE bit to be set */
//        WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 14);

//        RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
//    } else if (RegisterLen == 2) {
//        /*!< Disable Acknowledgment */
//        I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);

//        /* Set POS bit */
//        SENSORS_I2C->CR1 |= I2C_CR1_POS;

//        /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
//        CLEAR_ADDR_BIT;

//        /* Wait for the buffer full bit to be set */
//        WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 15);

//        /*!< Send STOP Condition */
//        I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

//        /* Read 2 bytes */
//        RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
//        RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
//    } else if (RegisterLen == 3) {
//        CLEAR_ADDR_BIT;

//        /* Wait for the buffer full bit to be set */
//        WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
//        /*!< Disable Acknowledgment */
//        I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
//        /* Read 1 bytes */
//        RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
//        /*!< Send STOP Condition */
//        I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
//        /* Read 1 bytes */
//        RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
//        /* Wait for the buffer full bit to be set */
//        WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
//        /* Read 1 bytes */
//        RegisterValue[2] = I2C_ReceiveData(SENSORS_I2C);
//    } else /* more than 2 bytes */
//    {
//        /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
//        CLEAR_ADDR_BIT;

//        for (i = 0; i < (RegisterLen); i++) {
//            if (i == (RegisterLen - 3)) {
//                /* Wait for the buffer full bit to be set */
//                WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);

//                /*!< Disable Acknowledgment */
//                I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);

//                /* Read 1 bytes */
//                RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);

//                /*!< Send STOP Condition */
//                I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

//                /* Read 1 bytes */
//                RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);

//                /* Wait for the buffer full bit to be set */
//                WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);

//                /* Read 1 bytes */
//                RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
//                goto endReadLoop;
//            }

//            /* Wait for the RXNE bit to be set */
//            WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 18);
//            RegisterValue[i] = I2C_ReceiveData(SENSORS_I2C);
//        }
//    }

//endReadLoop:
//    /* Clear BTF flag */
//    I2C_ClearFlag(SENSORS_I2C, I2C_FLAG_BTF);
//    /* Wait for the busy flag to be cleared */
//    WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 19);
//    /*!< Re-Enable Acknowledgment to be ready for another reception */
//    I2C_AcknowledgeConfig(SENSORS_I2C, ENABLE);
//    // Disable POS -- TODO
//    SENSORS_I2C->CR1 &= ~I2C_CR1_POS;

//    /* Return the byte read from sensor */
//    return result;
//}

///**
// * @brief
// * 写命令(多次尝试)，这是底层I2C接口，专用于只向IIC设备发送命令的函数，命令不含数据
// * @param  slave_addr: 从机地址
// * @param 	reg_addr:寄存器地址（要写入的命令）
// * @retval 正常为0，不正常为非0
// */
//int Sensors_I2C_WriteNoRegister(unsigned char slave_addr,
//                                unsigned char reg_addr) {
//    char retries = 0;
//    int ret = 0;
//    unsigned short retry_in_mlsec = Get_I2C_Retry();

//tryWriteAgain:
//    ret = 0;
//    ret = ST_Sensors_I2C_WriteNoRegister(slave_addr, reg_addr);

//    if (ret && retry_in_mlsec) {
//        if (retries++ > 4) return ret;

//        Delay(retry_in_mlsec);
//        goto tryWriteAgain;
//    }
//    return ret;
//}

///**
// * @brief
// *读数据(多次尝试)，这是底层I2C接口，专用于只向IIC设备读取数据(不需要传送寄存器地址)
// * @param  slave_addr: 从机地址
// * @param 	RegisterLen:数据长度
// *	@param	RegisterValue：指向存储数据的指针
// * @retval 正常为0，不正常为非0
// */
//int Sensors_I2C_ReadNoRegister(unsigned char slave_addr, unsigned short len,
//                               unsigned char *data_ptr) {
//    char retries = 0;
//    int ret = 0;
//    unsigned short retry_in_mlsec = Get_I2C_Retry();

//tryReadAgain:
//    ret = 0;
//    ret = ST_Sensors_I2C_ReadNoRegister(slave_addr, len, data_ptr);

//    if (ret && retry_in_mlsec) {
//        if (retries++ > 4) return ret;

//        Delay(retry_in_mlsec);
//        goto tryReadAgain;
//    }
//    return ret;
//}

///**
// * @brief
// * 写命令(单次尝试)，这是底层I2C接口，专用于只向IIC设备发送命令的函数，命令不含数据
// * @param  slave_addr: 从机地址
// * @param 	reg_addr:寄存器地址（要写入的命令）
// * @retval 正常为0，不正常为非0
// */
//static unsigned long ST_Sensors_I2C_WriteNoRegister(
//    unsigned char Address, unsigned char RegisterAddr) {
//    uint32_t result = 0;
//    __IO uint32_t I2CTimeout = I2Cx_LONG_TIMEOUT;

//    /* Wait for the busy flag to be cleared */
//    WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 1);

//    /* Start the config sequence */
//    I2C_GenerateSTART(SENSORS_I2C, ENABLE);

//    /* Wait for the start bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 2);

//    /* Transmit the slave address and enable writing operation */
//    I2C_Send7bitAddress(SENSORS_I2C, (Address << 1), I2C_Direction_Transmitter);

//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 3);

//    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
//    CLEAR_ADDR_BIT

//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 4);

//    /* Transmit the first address for write operation */
//    I2C_SendData(SENSORS_I2C, RegisterAddr);

//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 6);

//    /* End the configuration sequence */
//    I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

//    /* Return the verifying value: 0 (Passed) or 1 (Failed) */
//    return result;
//}

///**
// * @brief
// *读数据(单次尝试)，这是底层I2C接口，专用于只向IIC设备读取数据(不需要传送寄存器地址)
// * @param  slave_addr: 从机地址
// * @param 	RegisterLen:数据长度
// *	@param	RegisterValue：指向存储数据的指针
// * @retval 正常为0，不正常为非0
// */
//static unsigned long ST_Sensors_I2C_ReadNoRegister(
//    unsigned char Address, unsigned short RegisterLen,
//    unsigned char *RegisterValue) {
//    uint32_t i = 0, result = 0;
//    __IO uint32_t I2CTimeout = I2Cx_LONG_TIMEOUT;

//    /* Wait for the busy flag to be cleared */
//    WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 7);

//    /*!< Send START condition a second time */
//    I2C_GenerateSTART(SENSORS_I2C, ENABLE);

//    /* Wait for the start bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 12);

//    /*!< Send address for read */
//    I2C_Send7bitAddress(SENSORS_I2C, (Address << 1), I2C_Direction_Receiver);

//    /* Wait for the start bit to be set */
//    WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 13);

//    if (RegisterLen == 1) {
//        /*!< Disable Acknowledgment */
//        I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);

//        /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
//        CLEAR_ADDR_BIT;

//        /*!< Send STOP Condition */
//        I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

//        /* Wait for the RXNE bit to be set */
//        WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 14);

//        RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
//    } else if (RegisterLen == 2) {
//        /*!< Disable Acknowledgment */
//        I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);

//        /* Set POS bit */
//        SENSORS_I2C->CR1 |= I2C_CR1_POS;

//        /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
//        CLEAR_ADDR_BIT;

//        /* Wait for the buffer full bit to be set */
//        WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 15);

//        /*!< Send STOP Condition */
//        I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

//        /* Read 2 bytes */
//        RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
//        RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
//    } else if (RegisterLen == 3) {
//        CLEAR_ADDR_BIT;

//        /* Wait for the buffer full bit to be set */
//        WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
//        /*!< Disable Acknowledgment */
//        I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
//        /* Read 1 bytes */
//        RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
//        /*!< Send STOP Condition */
//        I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
//        /* Read 1 bytes */
//        RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
//        /* Wait for the buffer full bit to be set */
//        WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
//        /* Read 1 bytes */
//        RegisterValue[2] = I2C_ReceiveData(SENSORS_I2C);
//    } else /* more than 2 bytes */
//    {
//        /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
//        CLEAR_ADDR_BIT;

//        for (i = 0; i < (RegisterLen); i++) {
//            if (i == (RegisterLen - 3)) {
//                /* Wait for the buffer full bit to be set */
//                WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);

//                /*!< Disable Acknowledgment */
//                I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);

//                /* Read 1 bytes */
//                RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);

//                /*!< Send STOP Condition */
//                I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

//                /* Read 1 bytes */
//                RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);

//                /* Wait for the buffer full bit to be set */
//                WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);

//                /* Read 1 bytes */
//                RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
//                goto endReadLoop;
//            }

//            /* Wait for the RXNE bit to be set */
//            WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 18);
//            RegisterValue[i] = I2C_ReceiveData(SENSORS_I2C);
//        }
//    }

//endReadLoop:
//    /* Clear BTF flag */
//    I2C_ClearFlag(SENSORS_I2C, I2C_FLAG_BTF);
//    /* Wait for the busy flag to be cleared */
//    WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 19);
//    /*!< Re-Enable Acknowledgment to be ready for another reception */
//    I2C_AcknowledgeConfig(SENSORS_I2C, ENABLE);
//    // Disable POS -- TODO
//    SENSORS_I2C->CR1 &= ~I2C_CR1_POS;

//    /* Return the byte read from sensor */
//    return result;
//}

//static unsigned short RETRY_IN_MLSEC = 55;

///**
// * @brief  设置重试等待毫秒数
// * @param  无
// * @retval 无
// */
//void Set_I2C_Retry(unsigned short ml_sec) { RETRY_IN_MLSEC = ml_sec; }

///**
// * @brief  获取重试等待毫秒数
// * @param  无
// * @retval 无
// */
//unsigned short Get_I2C_Retry(void) { return RETRY_IN_MLSEC; }

//////为HMC5883L而设置的：
////void I2C_SAND_BYTE(u8 SlaveAddr, u8 writeAddr,
////                   u8 pBuffer) {  // I2C发送一个字节（从地址，内部地址，内容）
////    I2C_GenerateSTART(I2C1, ENABLE);  //发送开始信号
////    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
////        ;  //等待完成
////    I2C_Send7bitAddress(
////        I2C1, SlaveAddr,
////        I2C_Direction_Transmitter);  //发送从器件地址及状态（写入）
////    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
////        ;                           //等待完成
////    I2C_SendData(I2C1, writeAddr);  //发送从器件内部寄存器地址
////    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
////        ;                         //等待完成
////    I2C_SendData(I2C1, pBuffer);  //发送要写入的内容
////    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
////        ;                            //等待完成
////    I2C_GenerateSTOP(I2C1, ENABLE);  //发送结束信号
////}

////void I2C_READ_BUFFER(
////    u8 SlaveAddr, u8 readAddr, u8 *pBuffer,
////    u16 NumByteToRead) {  // I2C读取数据串（器件地址，寄存器，内部地址，数量）
////    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
////        ;
////    I2C_GenerateSTART(I2C1, ENABLE);  //开启信号
////    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
////        ;  //清除 EV5
////    I2C_Send7bitAddress(I2C1, SlaveAddr,
////                        I2C_Direction_Transmitter);  //写入器件地址
////    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
////        ;  //清除 EV6
////    I2C_Cmd(I2C1, ENABLE);
////    I2C_SendData(I2C1, readAddr);  //发送读的地址
////    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
////        ;                             //清除 EV8
////    I2C_GenerateSTART(I2C1, ENABLE);  //开启信号
////    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
////        ;  //清除 EV5
////    I2C_Send7bitAddress(I2C1, SlaveAddr,
////                        I2C_Direction_Receiver);  //将器件地址传出，主机为读
////    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
////        ;  //清除EV6
////    while (NumByteToRead) {
////        if (NumByteToRead == 1) {  //只剩下最后一个数据时进入 if 语句
////            I2C_AcknowledgeConfig(I2C1, DISABLE);  //最后有一个数据时关闭应答位
////            I2C_GenerateSTOP(I2C1, ENABLE);  //最后一个数据时使能停止位
////        }
////        if (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {  //读取数据
////            *pBuffer = I2C_ReceiveData(I2C1);  //调用库函数将数据取出到 pBuffer
////            pBuffer++;                         //指针移位
////            NumByteToRead--;                   //字节数减 1
////        }
////    }
////    I2C_AcknowledgeConfig(I2C1, ENABLE);
////}


void I2C3_Configuration(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    /*I2C2外设时钟使能 */
    OLED_I2C_CLK_INIT(OLED_I2C_CLK, ENABLE);

    /*I2C2外设GPIO时钟使能 */
    RCC_AHB1PeriphClockCmd(OLED_I2C_SCL_GPIO_CLK | OLED_I2C_SDA_GPIO_CLK,
                           ENABLE);

    /*!< GPIO 配置 */
    GPIO_PinAFConfig(OLED_I2C_SCL_GPIO_PORT, OLED_I2C_SCL_SOURCE,
                     OLED_I2C_SCL_AF);
    GPIO_PinAFConfig(OLED_I2C_SDA_GPIO_PORT, OLED_I2C_SDA_SOURCE,
                     OLED_I2C_SDA_AF);

    /*!< 配置OLED_I2C引脚: SCL */
    
    
    GPIO_InitStructure.GPIO_Pin = OLED_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  // I2C必须开漏输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(OLED_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

    /*!<配置OLED_I2C引脚: SDA */
    GPIO_InitStructure.GPIO_Pin = OLED_I2C_SDA_PIN;
    GPIO_Init(OLED_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

    /*!< GPIO 配置 */
    GPIO_SetBits(OLED_I2C_SDA_GPIO_PORT, OLED_I2C_SCL_PIN | OLED_I2C_SDA_PIN);

    /* I2C 配置 */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle =
        I2C_DutyCycle_2; /* 高电平数据稳定，低电平数据变化 SCL 时钟线的占空比 */
    I2C_InitStructure.I2C_OwnAddress1 = OLED_ADDRESS;  //主机的I2C地址
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress =
        I2C_AcknowledgedAddress_7bit;             /* I2C的寻址模式 */
    I2C_InitStructure.I2C_ClockSpeed = I2C_Speed; /* 通信速率 */

    I2C_Init(OLED_I2C, &I2C_InitStructure); /* I2C2 初始化 */
    I2C_Cmd(OLED_I2C, ENABLE);              /* 使能 I2C2 */

    GPIO_PinAFConfig(OLED_I2C_SCL_GPIO_PORT, OLED_I2C_SCL_SOURCE,
                     OLED_I2C_SCL_AF);
    GPIO_PinAFConfig(OLED_I2C_SDA_GPIO_PORT, OLED_I2C_SDA_SOURCE,
                     OLED_I2C_SDA_AF);
}

/**
 * @brief  初始化I2C总线，使用I2C前需要调用
 * @param  无
 * @retval 无
 */
void I2C1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure1;

    /* Enable I2Cx clock */
    RCC_APB1PeriphClockCmd(SENSORS_I2C_RCC_CLK, ENABLE);

    /* Enable I2C GPIO clock */
    RCC_AHB1PeriphClockCmd(SENSORS_I2C_SCL_GPIO_CLK | SENSORS_I2C_SDA_GPIO_CLK,
                           ENABLE);

    /* Configure I2Cx pin: SCL ----------------------------------------*/
    GPIO_InitStructure.GPIO_Pin = SENSORS_I2C_SCL_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    /* Connect pins to Periph */
    GPIO_PinAFConfig(SENSORS_I2C_SCL_GPIO_PORT, SENSORS_I2C_SCL_GPIO_PINSOURCE,
                     SENSORS_I2C_AF);
    GPIO_Init(SENSORS_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

    /* Configure I2Cx pin: SDA ----------------------------------------*/
    GPIO_InitStructure.GPIO_Pin = SENSORS_I2C_SDA_GPIO_PIN;

    /* Connect pins to Periph */
    GPIO_PinAFConfig(SENSORS_I2C_SDA_GPIO_PORT, SENSORS_I2C_SDA_GPIO_PINSOURCE,
                     SENSORS_I2C_AF);
    GPIO_Init(SENSORS_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

    I2C_DeInit(SENSORS_I2C);
    I2C_InitStructure1.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure1.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure1.I2C_OwnAddress1 = I2C_OWN_ADDRESS;
    I2C_InitStructure1.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure1.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure1.I2C_ClockSpeed = I2C_SPEED;

    /* Enable the I2C peripheral */
    I2C_Cmd(SENSORS_I2C, ENABLE);

    /* Initialize the I2C peripheral */
    I2C_Init(SENSORS_I2C, &I2C_InitStructure1);

    return;
}

void I2C_WriteByte(I2C_TypeDef * I2Cx,uint8_t slave_addr,uint8_t reg_addr, uint8_t data) {
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
        ;

    I2C_GenerateSTART(I2Cx, ENABLE);  //开启I2Cx
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
        ; /*EV5,主模式*/

    I2C_Send7bitAddress(I2Cx, slave_addr,
                        I2C_Direction_Transmitter);  //器件地址
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ;

    I2C_SendData(I2Cx, reg_addr);  //寄存器地址
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;

    I2C_SendData(I2Cx, data);  //发送数据
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;

    I2C_GenerateSTOP(I2Cx, ENABLE);  //关闭I2Cx总线
}

void I2C_ReadData(I2C_TypeDef * I2Cx,
    uint8_t slave_addr, uint8_t reg_addr, uint8_t *pBuffer,
    uint16_t NumByteToRead) {  // I2C读取数据串（器件地址，寄存器，内部地址，数量）
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
        ;
    I2C_GenerateSTART(I2Cx, ENABLE);  //开启信号
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
        ;  //清除 EV5
    I2C_Send7bitAddress(I2Cx, slave_addr,
                        I2C_Direction_Transmitter);  //写入器件地址
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ;  //清除 EV6
    I2C_Cmd(I2Cx, ENABLE);
    I2C_SendData(I2Cx, reg_addr);  //发送读的地址
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;                             //清除 EV8
    I2C_GenerateSTART(I2Cx, ENABLE);  //开启信号
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
        ;  //清除 EV5
    I2C_Send7bitAddress(I2Cx, slave_addr,
                        I2C_Direction_Receiver);  //将器件地址传出，主机为读
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
        ;  //清除EV6
    while (NumByteToRead) {
        if (NumByteToRead == 1) {  //只剩下最后一个数据时进入 if 语句
            I2C_AcknowledgeConfig(I2Cx, DISABLE);  //最后有一个数据时关闭应答位
            I2C_GenerateSTOP(I2Cx, ENABLE);  //最后一个数据时使能停止位
        }
        if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {  //读取数据
            *pBuffer = I2C_ReceiveData(I2Cx);  //调用库函数将数据取出到 pBuffer
            pBuffer++;                         //指针移位
            NumByteToRead--;                   //字节数减 1
        }
    }
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
}
	
