#include "spi.h"

// connect one on one MISO - MISO, MOSI - MOSI
void SPI_Configure(uint16_t direction, uint16_t mode, uint16_t datasize, uint16_t cpol, uint16_t cpha, uint16_t nss, uint16_t baudrate_prescaler, uint16_t first_bit, uint16_t crc_poly, uint16_t fifo, uint8_t nss_pin)
{
    SPI_InitTypeDef SPI_InitStructure;
    // Enable SPI1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    // SPI1 configuration
    SPI_InitStructure.SPI_Direction = direction; // Initially Tx
    SPI_InitStructure.SPI_Mode = mode;
    SPI_InitStructure.SPI_DataSize = datasize;
    SPI_InitStructure.SPI_CPOL = cpol; // Clock steady high
    SPI_InitStructure.SPI_CPHA = cpha; // Data write on rising (second) edge
    SPI_InitStructure.SPI_NSS = nss;
    SPI_InitStructure.SPI_BaudRatePrescaler = baudrate_prescaler;
    SPI_InitStructure.SPI_FirstBit = first_bit;
    SPI_InitStructure.SPI_CRCPolynomial = crc_poly;
    SPI_Init(SPI1, &SPI_InitStructure);
    SPI_RxFIFOThresholdConfig(SPI1, fifo);
    SPI_Cmd(SPI1, ENABLE);
    pinMode(nss_pin, OUT);
} 


SPI_InitTypeDef     SPI_InitStructure;
GPIO_InitTypeDef    GPIO_InitStructure;

/* GPIOA Periph clock enable */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
/* Configure PA5, PA7 in AF pushpool mode */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
/* Initialize GPIOA */
GPIO_Init(GPIOA, &GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
/* Initialize GPIOA */
GPIO_Init(GPIOA, &GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
/* Initialize GPIOA */
GPIO_Init(GPIOA, &GPIO_InitStructure);

//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

//GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);

SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
SPI_InitStructure.SPI_CRCPolynomial = 7;
/* Initialize SPI1 */
SPI_Init(SPI1, &SPI_InitStructure);
/* enable SPI1 */
SPI_Cmd(SPI1, ENABLE);

}

void send_byte(uint8_t val, uint8_t nss_pin)
{
  digitalWrite(nss_pin, LOW);
    //GPIO_ResetBits(GPIOA, GPIO_Pin_4); // CS low
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); //wait buffer empty
    SPI_SendData8(SPI1, val);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET); //wait finish sending
    //GPIO_SetBits(GPIOA, GPIO_Pin_4); // CS high
  digitalWrite(nss_pin, HIGH);
}

uint8_t send_and_read_byte(uint8_t cmd, uint8_t nss_pin)
{
    uint8_t result;
    //GPIO_ResetBits(GPIOA, GPIO_Pin_4); // CS low
    digitalWrite(nss_pin, LOW);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); //wait buffer empty
    SPI_SendData8(SPI1, cmd);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET); //wait finish sending
    // Read receiving FIFO until it is empty
    while (SPI_GetReceptionFIFOStatus(SPI1) != SPI_ReceptionFIFOStatus_Empty)
        SPI_ReceiveData8(SPI1);
    SPI_BiDirectionalLineConfig(SPI1, SPI_Direction_Rx);
    //while (!(SPI1->SR & SPI_I2S_FLAG_RXNE)) ; // wait data received
    while( SPI1->SR & SPI_I2S_FLAG_BSY );
    //GPIO_SetBits(GPIOA, GPIO_Pin_4); // CS high
      digitalWrite(nss_pin, HIGH);
    SPI1->CR1 |= SPI_Direction_Tx;  // Set Tx mode to stop Rx clock
    result = SPI_ReceiveData8(SPI1);
    return result;
}
