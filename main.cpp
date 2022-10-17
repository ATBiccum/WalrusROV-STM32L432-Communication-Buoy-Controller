/* Walrus ROV
 * STM32L4 Com Buoy
 * 17/10/2022
 * Alexander, Tony, Clinton
 * 
 * Code Description:
 * The purpose of this code is to receive data from the nRF transceiver and pass that to UART to RS485 modules.
 * It does this using SPI for the nRF module and memcpy the data into the variable for UART.
 * UART communication is done using DMA to Tx and Rx to put less strain on the CPU and make com more robust.
 * 
 * Control Station nRF Pins for STM32F407:
 * GND = GND = 1
 * VCC = 3.3V = 2
 * MOSI = PC_12 = 6 
 * MISO = PC_11 = 7 
 * SCK = PC_10 = 5 
 * CSN = PC_9 = 4
 * CE = PC_8 = 3
 * IRQ = PC_7 = 8
 *
 * Com Buoy nRF Pins for STM32L432:
 * GND = GND = 1
 * VCC = 3.3V = 2
 * MOSI = PA_7 = 6 
 * MISO = PA_6 = 7 
 * SCK = PA_5 = 5 
 * CSN = PA_4 = 4
 * CE = PA_1 = 3
 * IRQ = PA_0 = 8

 * PS3 Controls Packet Format: 
 *  1  4   7  10   13  16  19 20 21 22 23 24 25 28
 * "# 000 000 000 000 000 000 0  0  0  0  0  0  000" = 28
 * Pound, L1, L2, LeftHatX, LefthatY, RightHatX, RightHatY, R1, R2, Triangle, Circle, Cross, Square, Checksum
 *
 */
#include <mbed.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "nRF24L01P.h"
#include "cmsis_os.h"

SPI_HandleTypeDef hspi1;   //Tranceivers

Thread nRF24Thread;

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

//nRF Tx and Rx Variables
char nRF_RxData[32]; //"#000000134111154132000000"
int nRF_RxDataCnt = 0;
char nRF_TxData[32];
bool nRF_NewData;

//RS485 Tx and Rx Variables
uint8_t RS485_RxData[32] = {0};
uint8_t RS485_TxData[32] = {0}; //"##99000000134111154132000000999"
size_t old_posRx;
size_t posRx;
bool RS485_NewData;

int main()
{
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    SystemClock_Config();              
    GPIO_Init();
    USART1_UART_Init();
    SPI1_Init();

    USART1_DMA1_Start_Transmit();
    //USART1_DMA1_Start_Transmit();
    //nRF24Thread.start(nRF24);
    while(1)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        HAL_Delay(500);
        /*for (int i = 0; i < 33; i++)
        {
            HAL_Delay(1);
            LL_USART_TransmitData8( USART1, RS485_TxData[i]);
        }*/
        
        //printf("DMA Buffer: %s\n", RS485_RxData); //DMA Buffer
        //printf("nRF Transmitting: %s\n", nRF_TxData); //Processed Data
        //printf("Length: %d\n", length);

        /*uint8_t length = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
        if(length==1){printf("nRF Transmitting: %s\n", nRF_TxData);}

        uint8_t len2 = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_4);
        printf("DMA1 Length: %d\n", len2);*/
    }
}

void USART1_DMA1_Start_Transmit(void)
{
    //Start USART Transmission through DMA
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
    LL_USART_ClearFlag_TC(USART1);
    LL_DMA_ClearFlag_TC4(DMA1);
    // set length to be tranmitted
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, ARRAY_LEN(RS485_TxData));
    
    // configure address to be transmitted by DMA
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, (uint32_t)RS485_TxData, (uint32_t)LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)RS485_TxData);

    // Enable DMA again
    LL_USART_EnableDirectionTx(USART1);
    LL_USART_EnableDMAReq_TX(USART1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}

void USART_Rx_Check(void) 
{
    //Function called on every TC interrupt
    posRx = ARRAY_LEN(RS485_RxData) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
    //See github tutorial for explanation
    if (posRx > old_posRx) 
    {                 
        USART_Process_Data(&RS485_RxData[old_posRx], posRx - old_posRx);
        old_posRx = posRx;
    } 
    else if (posRx < old_posRx)
    {
        USART_Process_Data(&RS485_RxData[old_posRx], ARRAY_LEN(RS485_RxData) - old_posRx);
        old_posRx = posRx;                       
    }
    else if (posRx == 0)
    {
        //This will be the typical case: waits until a full packet is ready to be processed
        USART_Process_Data(&RS485_RxData[0], 32);
        old_posRx = posRx;
    }
}

void USART_Process_Data(uint8_t* data, size_t len) 
{
    memcpy(nRF_TxData, data, 32);
    RS485_NewData = true;
}

//Active Functions: (Called Often)
void nRF24()
{
    /*****Transceiver Initialization*****/
    nRF24L01P nRF(PA_7, PA_6, PA_5, PA_4, PA_1, PA_0); //mosi, miso, sck, csn, ce, irq
    
    nRF.powerUp();
    nRF.setTransferSize(32);

    printf("nRF24L01+ Frequency     : %d MHz\r\n",  nRF.getRfFrequency());
    printf("nRF24L01+ Output power  : %d dBm\r\n",  nRF.getRfOutputPower());
    printf("nRF24L01+ Data Rate     : %d kbps\r\n", nRF.getAirDataRate());
    printf("nRF24L01+ TX Address    : 0x%010llX\r\n", nRF.getTxAddress());
    printf("nRF24L01+ RX Address    : 0x%010llX\r\n", nRF.getRxAddress());
    printf("nRF24L01+ Transfer Size : %d\r\n", nRF.getTransferSize());
    
    nRF.setReceiveMode(); 
    nRF.enable(); 
    /***********************************/
    while(1)
    {
        //Receive data if there is data to be received
        if (nRF.readable())
        {
            nRF_RxDataCnt = nRF.read(NRF24L01P_PIPE_P0, nRF_RxData, sizeof(nRF_RxData));
            nRF_NewData = true;
        }
        if (nRF_NewData == true)
        {
            //printf("Received: %s\n", nRF_RxData);
            memcpy(RS485_TxData, nRF_RxData, 32); //Put nRF Rx data into RS485 Tx data
            nRF_NewData = false;
        }
        if(RS485_NewData == true)
        {
            nRF.write(NRF24L01P_PIPE_P0, nRF_TxData, sizeof(nRF_TxData));
        }
    }
}

void Error_Handler(void)
{
    //User can add his own implementation to report the HAL error return state
    __disable_irq();
    while (1)
    {
    }
}

//Initializations and Config Functions: (Called Once)
static void GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    //GPIO Ports Clock Enable
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    //Configure GPIO pins for nRF module and RS485 Module (PA_8):
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_4|LL_GPIO_PIN_1|LL_GPIO_PIN_0;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //Configure GPIO pin for onboard LED:
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //Configure GPIO pins for UART1 (PA_9 = Tx , PA_10 = Rx) RS485
    GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10; //Tx, Rx 
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //SPI1 GPIO Configuration (PA_5 = SCK, PA_6 = MISO, PA_7 = MOSI)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7; //SCK, MISO, MOSI
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void SPI1_Init(void)
{
    //spi1 parameter configuration
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_SLAVE;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void USART1_UART_Init(void)
{
    LL_USART_InitTypeDef USART_InitStruct = {0};

    //Peripheral clock enable
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    // USART1 DMA init for Channel 5 = Rx
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMA_REQUEST_2);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)RS485_RxData);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, ARRAY_LEN(RS485_RxData));

    //Enable Transfer Complete Interrupt
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);

    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    // USART1 DMA1 init for Channel 4 = Tx
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_2);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT));
    //LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)RS485_TxData);
    //LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, ARRAY_LEN(RS485_TxData));

    // Enable TC interrupts 
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);

    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1));
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    /* USART configuration */
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART1);
    LL_USART_EnableDMAReq_RX(USART1);
    //LL_USART_EnableDMAReq_TX(USART1);
    LL_USART_EnableIT_IDLE(USART1);

    //USART interrupt
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART1_IRQn);

    // Enable USART and DMA 
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5); //Enable DMA Channel for Rx
    //LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4); //Enable DMA Channel for Tx
    LL_USART_Enable(USART1);
}


//Interrupt Handlers:
void DMA1_Channel5_IRQHandler(void) 
{
    //Channel 5 Rx Interrupt Handler
    // Check transfer-complete interrupt 
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_5) && LL_DMA_IsActiveFlag_TC5(DMA1)) {
        LL_DMA_ClearFlag_TC5(DMA1);             // Clear transfer complete flag 
        USART_Rx_Check();                       // Check for data to process 
    }
}

void DMA1_Channel4_IRQHandler(void) 
{
    //Channel 4 Tx Interrupt Handler 
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_4) && LL_DMA_IsActiveFlag_TC4(DMA1)) 
    {
        LL_DMA_ClearFlag_TC4(DMA1);    // Clear transfer complete flag
        USART1_DMA1_Start_Transmit();
    }
}


void USART1_IRQHandler(void) 
{
    // Check for IDLE line interrupt
    if (LL_USART_IsEnabledIT_IDLE(USART1) && LL_USART_IsActiveFlag_IDLE(USART1)) {
        LL_USART_ClearFlag_IDLE(USART1);        // Clear IDLE line flag 
        USART_Rx_Check();                       // Check for data to process 
    }

    //Implement other events when needed
}

void SystemClock_Config(void)
{
    /* Configure flash latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
        while (1) {}
    }

    /* Configure voltage scaling */
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    /* Configure MSI */
    LL_RCC_MSI_Enable();
    while (LL_RCC_MSI_IsReady() != 1) {}
    LL_RCC_MSI_EnableRangeSelection();
    LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_11);
    LL_RCC_MSI_SetCalibTrimming(0);

    /* Configure system clock */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI) {}
    
    /* Configure prescalers */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    /* Configure systick */
    LL_Init1msTick(48000000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SetSystemCoreClock(48000000);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
}