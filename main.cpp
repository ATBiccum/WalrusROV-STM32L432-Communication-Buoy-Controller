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
 * MOSI = PA_7 = 6 
 * MISO = PA_6 = 7 
 * SCK = PA_5 = 5 
 * CSN = PB_0 = 4
 * CE = PA_1 = 3
 * IRQ = PA_0 = 8
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
 * https://www.st.com/resource/en/user_manual/dm00231744-stm32-nucleo32-boards-mb1180-stmicroelectronics.pdf
 * https://www.st.com/resource/en/reference_manual/rm0432-stm32l4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
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

//nRF Vairables
char nRF_RxData[32];
int nRF_RxDataCnt = 0;
char nRF_TxData[32] = {0};
bool nRF_NewData = false;
bool nRF_Initialized = false;

//RS485 Tx and Rx Variables
uint8_t RS485_RxData[32] = {0};
char RS485_TxData[32] = {0}; //"#99000000134111154132000000999#"
size_t old_posRx;
size_t posRx;
bool RS485_NewData;

int main()
{
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    GPIO_Init();
    USART1_UART_Init();
    SPI1_Init();

    USART1_DMA1_Start_Transmit();
    nRF24Thread.start(nRF24);

}

void USART1_DMA1_Start_Transmit(void)
{
    //Start USART Transmission through DMA
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)RS485_TxData);

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, ARRAY_LEN(RS485_TxData));

    LL_USART_EnableDMAReq_TX(USART1);

    LL_USART_EnableDirectionTx(USART1);

    LL_USART_ClearFlag_TC(USART1);
    LL_DMA_ClearFlag_TC4(DMA1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}

void USART_Rx_Check(void) 
{
    //Function called on every TC interrupt or IDLE line interrupt
    posRx = ARRAY_LEN(RS485_RxData) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
    if(posRx != old_posRx) //If new data
    {
        if (posRx > old_posRx) 
        {                 
            //Check if the new position in the buffer is greater than the old (Bad timed packet)
            USART_Process_Data(&RS485_RxData[old_posRx], posRx - old_posRx);
        } 
        else 
        {
            //Check if the new position in the buffer is less than the old (Bad packet)
            USART_Process_Data(&RS485_RxData[old_posRx], ARRAY_LEN(RS485_RxData) - old_posRx);
            if(posRx > 0)
            {
                USART_Process_Data(&RS485_RxData[0], posRx);
            }
                                   
        }
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
    nRF24L01P nRF(PA_7, PA_6, PA_5, PB_0, PA_1, PA_0); //mosi, miso, sck, csn, ce, irq

    while(1)
    {
         /*****Transceiver Initialization*****/
        /* Re-try nRF Initialization Steps 
        1. Enable power to module  
        2. Set transfer size
        3. Check output from nRF config functions
            If No Error
            Module is correctly initialized, proceed to step 4
            If Error:
            Disable power to module, restart at step 1
        4. Set receive mode (module will change to tx mode when transmitting)
        5. Enable the module 
        */
        HAL_Delay(5000);
        nRF.powerUp();
        nRF.setTransferSize(32);
        uint8_t nRF_InitCheck = 0;
        //Check if nRF module is powered and communicating properly, will loop until no errors
        if(nRF.getRfFrequency() < 2525 || nRF.getRfFrequency() > 2400){printf("nRF24L01+ Frequency     : %d MHz\r\n",  nRF.getRfFrequency());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(1);}
        if(nRF.getRfOutputPower() != 1){printf("nRF24L01+ Output power  : %d dBm\r\n",  nRF.getRfOutputPower());nRF_InitCheck++;}
        else{nRF.getRfOutputPower();nRF.disable();nRF.powerDown();nRF_Error_Handler(2);}
        if(nRF.getAirDataRate() != 0){printf("nRF24L01+ Data Rate     : %d kbps\r\n", nRF.getAirDataRate());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(3);}
        if(nRF.getTxAddress() != 0){printf("nRF24L01+ TX Address    : 0x%010llX\r\n", nRF.getTxAddress());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(4);}
        if(nRF.getRxAddress() != 0){printf("nRF24L01+ RX Address    : 0x%010llX\r\n", nRF.getRxAddress());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(5);}
        if(nRF.getTransferSize() == 32){printf("nRF24L01+ Transfer Size : %d\r\n", nRF.getTransferSize());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(6);}
        if(nRF.getCrcWidth() != 0){printf("nRF24L01P+ CRC Width    :  %d\r\n", nRF.getCrcWidth());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(7);}

        if (nRF_InitCheck == 7)
        {
            nRF.setReceiveMode(); 
            nRF.enable();
            printf("nRF Initialized!\n"); 
            nRF_Initialized = true;
        }
        while (nRF_Initialized)
        {
            //Receive data if there is data to be received
            if (nRF.readable())
            {
                nRF_RxDataCnt = nRF.read(NRF24L01P_PIPE_P0, RS485_TxData, sizeof(RS485_TxData));
                //memcpy(RS485_TxData, nRF_RxData, 32);
                USART1_DMA1_Start_Transmit();
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
            }
            //memcpy(nRF_TxData, RS485_RxData, 32);
            //nRF.write(NRF24L01P_PIPE_P1, nRF_TxData, sizeof(nRF_TxData));
        }
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

    //SPI1 GPIO Configuration 
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7; //PA_5 = SCK, PA_6 = MISO, PA_7 = MOSI
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_0; //PB_0 = CSN
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_0; //PA_1 = CE, PA_0 = IRQ
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void SPI1_Init(void)
{
    LL_SPI_InitTypeDef SPI_InitStruct = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BitOrder = LL_SPI_LSB_FIRST;
    SPI_InitStruct.BaudRate = LL_RCC_APB2_DIV_4;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 10;

    LL_SPI_Init(SPI1, &SPI_InitStruct);
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
    NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
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

    // Enable USART and DMA 
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5); //Enable DMA Channel for Rx
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
        //USART1_DMA1_Start_Transmit();
    }
}

void nRF_Error_Handler(uint8_t value)
{
    switch (value)
    {
        case 1:
        //RF Frequency Error
            printf("nRF24L01P: Unknown RF Frequency value.\n");
            break;
        case 2:
        //RF Output Power Error
            printf("nRF24L01P: Unknown RF Output Power value.\n");
            break;
        case 3:
        //Air Data Rate Error
            printf("nRF24L01P: Unknown Air Data Rate value.\n");
            break;
        case 4:
        //Transmit Address Error
            printf("nRF24L01P: Unknown Transmit Address value.\n");
            break;
        case 5:
        //Receive Address Error
            printf("nRF24L01P: Unknown Receive Address value.\n");
            break;
        case 6:
        //Transfer Size Error
            printf("nRF24L01P: Unknown Transfer Size value.\n");
            break;
        case 7:
        //CRC Size Error 
            printf("nRF24L01P: Unknown CRC Width value.\n");
            break;
    }
}
