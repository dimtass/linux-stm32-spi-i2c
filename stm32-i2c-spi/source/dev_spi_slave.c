#include "dev_spi_slave.h"

static struct spi_controller m_devices[] = {
	[DEV_SPI1_GPIOA] = {SPI1, GPIOA, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_5, GPIO_Pin_4},
	[DEV_SPI1_GPIOB] = {SPI1, GPIOA, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_5, GPIO_Pin_4},
	[DEV_SPI2] = {SPI2, GPIOB, GPIO_Pin_14, GPIO_Pin_15, GPIO_Pin_13, GPIO_Pin_12},
};

static struct dma_channel m_dma_channels[] = {
	[DEV_SPI1_GPIOA] = {DMA1_Channel3, DMA1_Channel3_IRQn, DMA1_Channel2, DMA1_Channel2_IRQn, {0}},
	[DEV_SPI1_GPIOB] = {DMA1_Channel3, DMA1_Channel3_IRQn, DMA1_Channel2, DMA1_Channel2_IRQn, {0}},
	[DEV_SPI2] = {DMA1_Channel5, DMA1_Channel5_IRQn, DMA1_Channel4, DMA1_Channel4_IRQn, {0}},
};

uint16_t SPIReceivedValue[3];
uint16_t SPITransmittedValue[3] = { 0, 0x0, 0x0};

static receive_irq_t _recv_cbk;
static struct spi_device * _spi = NULL;

void SPI_Slave_and_DMA_Configuration(struct spi_device * spi)
{
	// GPIO_InitTypeDef GPIO_InitStructure;
    // SPI_InitTypeDef SPI_InitStructure; //Variable used to setup the SPI
    DMA_InitTypeDef DMA_InitStructure; //Variable used to setup the DMA
	
	// RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE ) ;
	// /* GPIO Alternate function */
	// /* SPI1: PB12 -> NSS */
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	// GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// /* SPI1: PB13 -> SCK */
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	// GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// /* SPI1: PB14 -> MISO */
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	// GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// /* SPI1: PB15 -> MOSI */
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	// GPIO_Init(GPIOB, &GPIO_InitStructure);

    // //--Enable the SPI2 periph
    // RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    // // Reset SPI Interface
    // SPI_I2S_DeInit(SPI2);

    // //== SPI2 configuration
    // SPI_StructInit(&SPI_InitStructure);
    // SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    // SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
    // SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    // SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    // SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    // SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    // SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    // SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    // SPI_InitStructure.SPI_CRCPolynomial = 7;
    // SPI_Init(SPI2, &SPI_InitStructure);

    //--Enable DMA1 clock--
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    //==Configure DMA1 - Channel4== (SPI -> memory)
    DMA_DeInit(DMA1_Channel4); //Set DMA registers to default values
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR; //Address of peripheral the DMA must map to
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&SPIReceivedValue[0]; //Variable to which received data will be stored
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 3; //Buffer size
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure); //Initialise the DMA
    DMA_Cmd(DMA1_Channel4, ENABLE); //Enable the DMA1 - Channel4

    //==Configure DMA1 - Channel5== (memory -> SPI)
    DMA_DeInit(DMA1_Channel5); //Set DMA registers to default values
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR; //Address of peripheral the DMA must map to
	struct spi_buffers * buffers = (struct spi_buffers *) spi->data;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buffers->tx_buffer; //Variable from which data will be transmitted
    DMA_InitStructure.DMA_BufferSize = buffers->tx_buffer_len; //Buffer size
    // DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&SPITransmittedValue[0]; //Variable from which data will be transmitted
    // DMA_InitStructure.DMA_BufferSize = 3; //Buffer size
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure); //Initialise the DMA
    DMA_Cmd(DMA1_Channel5, ENABLE); //Enable the DMA1 - Channel5


    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    // NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    // DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

    // Enable SPI2
    SPI_Cmd(SPI2, ENABLE);

    // Enable the SPI2 RX & TX DMA requests
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);
}

void spi_init_slave(enum en_spi_port port,
                    struct spi_buffers * buffers,
                    receive_irq_t callback,
                    struct spi_device * spi)
{
    TRACEL(TRACE_LEVEL_SPI, ("Setting up the SPI slave\n"));
    TRACEL(TRACE_LEVEL_SPI, ("SPI slave: bits per word: %d\n", spi->bits_per_word));
    TRACEL(TRACE_LEVEL_SPI, ("SPI slave: mode: %d\n", spi->mode));

    /* Setup GPIOs */
	if (port == DEV_SPI1_GPIOA) {
		spi->controller = &m_devices[DEV_SPI1_GPIOA];
		spi->controller->dma = &m_dma_channels[DEV_SPI1_GPIOA];
		/* RCC configuration */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);
	}
	else if (port == DEV_SPI1_GPIOB) {
		spi->controller = &m_devices[DEV_SPI1_GPIOB];
		spi->controller->dma = &m_dma_channels[DEV_SPI1_GPIOB];
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_SPI1, ENABLE);
	}
	else if (port == DEV_SPI2) {
		spi->controller = &m_devices[DEV_SPI2];
		spi->controller->dma = &m_dma_channels[DEV_SPI2];
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	}
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = spi->controller->sck | spi->controller->mosi;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(spi->controller->port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = spi->controller->nss;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(spi->controller->port, &GPIO_InitStructure);
	spi->controller->port->ODR |= spi->controller->nss;

	GPIO_InitStructure.GPIO_Pin = spi->controller->miso;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(spi->controller->port, &GPIO_InitStructure);

    /* use pointers for simplification */
    SPI_TypeDef * dev = spi->controller->spi;
	SPI_InitTypeDef * spi_conf = &spi->controller->config;
	DMA_InitTypeDef * dma_conf = &spi->controller->dma->config;
	struct dma_channel * dma = spi->controller->dma;

	/* in case of SPI slave, controller = slave */
	spi->slave = spi->controller;
    spi->master = NULL;
    /* store buffers to the generic data pointer */
    spi->data = (void*) buffers;

    if (callback) {
        spi->rcv_callback = callback;
        _recv_cbk = callback;
    }
    /* We need a pointer to the spi_device for the interrupt */
    _spi = spi;

    /* Setup SPI interface */
    SPI_I2S_DeInit(dev);
	SPI_CalculateCRC(dev, DISABLE);
    SPI_StructInit(spi_conf);
    spi_conf->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi_conf->SPI_Mode = SPI_Mode_Slave;
    spi_conf->SPI_NSS = SPI_NSS_Hard;
    spi_conf->SPI_FirstBit = SPI_FirstBit_MSB;
    spi_conf->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    spi_conf->SPI_CRCPolynomial = 7;
	if (spi->bits_per_word == 16) {
    	spi_conf->SPI_DataSize = SPI_DataSize_16b;
    }
	else {
    	spi_conf->SPI_DataSize = SPI_DataSize_8b;
    }
    switch (spi->mode) {
    case SPI_MODE_0:
        spi_conf->SPI_CPOL = SPI_CPOL_Low;
        spi_conf->SPI_CPHA = SPI_CPHA_1Edge;
    	break;
    case SPI_MODE_1:
        spi_conf->SPI_CPOL = SPI_CPOL_Low;
        spi_conf->SPI_CPHA = SPI_CPHA_2Edge;
    	break;
    case SPI_MODE_2:
        spi_conf->SPI_CPOL = SPI_CPOL_High;
        spi_conf->SPI_CPHA = SPI_CPHA_1Edge;
    	break;
    case SPI_MODE_3:
        spi_conf->SPI_CPOL = SPI_CPOL_High;
        spi_conf->SPI_CPHA = SPI_CPHA_2Edge;
    	break;
    default:
        spi_conf->SPI_CPOL = SPI_CPOL_Low;
        spi_conf->SPI_CPHA = SPI_CPHA_2Edge;
        break;
    };
    SPI_Init(dev, spi_conf);

    /* DMA configuration */
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* Initialize DMA struct */
	DMA_StructInit(dma_conf);
	/* Init default DMA struct */
	dma_conf->DMA_PeripheralBaseAddr = (uint32_t) &dev->DR;
	dma_conf->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_conf->DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_conf->DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma_conf->DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma_conf->DMA_Priority = DMA_Priority_High;
	dma_conf->DMA_M2M = DMA_M2M_Disable;
    dma_conf->DMA_Mode = DMA_Mode_Circular;

	/* SPI_MASTER_Rx_DMA_Channel configuration */
	DMA_DeInit(dma->rx_ch);
    // dma_conf->DMA_MemoryBaseAddr = (uint32_t)&SPIReceivedValue[0]; //Variable to which received data will be stored
    // dma_conf->DMA_BufferSize = 2;
    dma_conf->DMA_MemoryBaseAddr = (uint32_t)buffers->rx_buffer; //Variable to which received data will be stored
    dma_conf->DMA_BufferSize = buffers->rx_buffer_len;
    dma_conf->DMA_DIR = DMA_DIR_PeripheralSRC;
    /* Initialize DMA */
	DMA_Init(dma->rx_ch, dma_conf);
	/* Enable DMA channels */
	DMA_Cmd(dma->rx_ch, ENABLE);
    
	/* SPI_MASTER_Tx_DMA_Channel configuration */
	DMA_DeInit(dma->tx_ch);
    // dma_conf->DMA_MemoryBaseAddr = (uint32_t)&SPITransmittedValue[0]; //Variable from which data will be transmitted
    // dma_conf->DMA_BufferSize = 2; //Buffer size
    dma_conf->DMA_MemoryBaseAddr = (uint32_t)buffers->tx_buffer; //Variable from which data will be transmitted
    dma_conf->DMA_BufferSize = buffers->tx_buffer_len; //Buffer size
    dma_conf->DMA_DIR = DMA_DIR_PeripheralDST;
    dma_conf->DMA_Mode = DMA_Mode_Normal;
    /* Initialize DMA */
	DMA_Init(dma->tx_ch, dma_conf);
	/* Enable DMA channels */
	DMA_Cmd(dma->tx_ch, ENABLE);

    NVIC_EnableIRQ(dma->rx_iqrn);
    DMA_ITConfig(dma->rx_ch, DMA_IT_TC, ENABLE);
    NVIC_EnableIRQ(dma->tx_iqrn);
    DMA_ITConfig(dma->tx_ch, DMA_IT_TC, ENABLE);

	/* EnableSPI, Tx DMA, DMA Tx request */
    SPI_Cmd(dev, ENABLE);
	SPI_I2S_DMACmd(dev, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);
}

void DMA1_Channel4_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC4) == SET) {
        DMA_Cmd(DMA1_Channel4, DISABLE);
		_recv_cbk(_spi);
        DMA_Cmd(DMA1_Channel4, ENABLE);
        DMA_ClearITPendingBit(DMA1_IT_TC4);
    }
}

void DMA1_Channel5_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC5) == SET) {
        DMA_Cmd(DMA1_Channel5, DISABLE);
		// TRACE(("SPI2.5: %02X\n", SPITransmittedValue[0]));
	    DMA_Init(_spi->controller->dma->tx_ch, &_spi->controller->dma->config);
        DMA_Cmd(DMA1_Channel5, ENABLE);
        DMA_ClearITPendingBit(DMA1_IT_TC5);
    }
}

// void DMA1_Channel2_IRQHandler(void) {
//     if (DMA_GetITStatus(DMA1_IT_TC2) == SET) {
//         DMA_Cmd(DMA1_Channel2, DISABLE);
//         DMA_ClearITPendingBit(DMA1_IT_TC2);
//     }
// }

// void DMA1_Channel3_IRQHandler(void) {
//     if (DMA_GetITStatus(DMA1_IT_TC3) == SET) {
//         DMA_Cmd(DMA1_Channel3, DISABLE);
//         DMA_ClearITPendingBit(DMA1_IT_TC3);
//     }
// }
