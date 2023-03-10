/*
 * W25Qxx_Flash.h
 *
 *  Created on: Feb 26, 2023
 *      Author: swift
 */



#ifndef _W25QXX_FLASH_H
#define _W25QXX_FLASH_H

#include "main.h"
#include <stdbool.h>

#define _W25QXX_DEBUG           1
#define W25QXX_SPI_CHANNEL		SPI3

#define W25QXX_SPI_SCLK_PIN		LL_GPIO_PIN_10
#define W25QXX_SPI_SCLK_PORT		GPIOC
#define W25QXX_SPI_SCLK_CLK		LL_AHB1_GRP1_PERIPH_GPIOC

#define W25QXX_SPI_MISO_PIN		LL_GPIO_PIN_11
#define W25QXX_SPI_MISO_PORT		GPIOC
#define W25QXX_SPI_MISO_CLK		LL_AHB1_GRP1_PERIPH_GPIOC

#define W25QXX_SPI_MOSI_PIN		LL_GPIO_PIN_12
#define W25QXX_SPI_MOSI_PORT		GPIOC
#define W25QXX_SPI_MOSI_CLK		LL_AHB1_GRP1_PERIPH_GPIOC

#define W25QXX_SPI_CS_PIN		LL_GPIO_PIN_3
#define W25QXX_SPI_CS_PORT		GPIOB
#define W25QXX_SPI_CS_CLK		LL_AHB1_GRP1_PERIPH_GPIOB


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define CHIP_SELECT(W25QXX)		LL_GPIO_ResetOutputPin(W25QXX_SPI_CS_PORT, W25QXX_SPI_CS_PIN)
#define CHIP_DESELECT(W25QXX)		LL_GPIO_SetOutputPin(W25QXX_SPI_CS_PORT, W25QXX_SPI_CS_PIN)


#define W25QXX_DUMMY_BYTE 		0xA5
#define REG_CHIP_ERASE	 		0xC7
#define REG_SECTOR_ERASE	 	0x20
#define REG_BLOCK_ERASE	 		0xD8
#define REG_WRITE_ENBALE 		0x06
#define REG_WRITE_DISABLE 		0x04
#define REG_FAST_READ 			0x0B
#define REG_PAGE_PROGRAM		0x02
#define REG_READ_STATUS_1 		0x05
#define REG_READ_STATUS_2 		0x35
#define REG_READ_STATUS_3 		0x15
#define REG_WRITE_STATUS_1 		0x01
#define REG_WRITE_STATUS_2 		0x31
#define REG_WRITE_STATUS_3 		0x11
#define REG_PAGE_PROGRAM 		0x02
#define REG_READ_DATA 			0x03
#define REG_ADDR_JEDEC_ID 		0x9F
#define REG_ADDR_UNIQUE_ID 		0x4B

#define JEDEC_ID 				0x00EF4018




	typedef enum
	{
		W25Q10 = 1,
		W25Q20,
		W25Q40,
		W25Q80,
		W25Q16,
		W25Q32,
		W25Q64,
		W25Q128,
		W25Q256,
		W25Q512,

	} W25QXX_ID_t;

	typedef struct
	{
		W25QXX_ID_t ID;
		uint8_t UniqID[8];
		uint16_t PageSize;
		uint32_t PageCount;
		uint32_t SectorSize;
		uint32_t SectorCount;
		uint32_t BlockSize;
		uint32_t BlockCount;
		uint32_t CapacityInKiloByte;
		uint8_t StatusRegister1;
		uint8_t StatusRegister2;
		uint8_t StatusRegister3;
		uint8_t Lock;

	} w25qxx_t;

	extern w25qxx_t w25qxx;




//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ICM-20602 Register Map
 */




/**
 * @brief W25QXX structure definition.
 */


/**
 * @brief W25QXX structure definition.
 */


/**
 * @brief W25QXX function prototype definition.
 */
	bool W25qxx_Init(void);

	void W25QXX_SPI_Initialization(void);
	unsigned char W25qxx_Spi(unsigned char data);
	void W25qxx_Spi_Transmit(unsigned char* data, unsigned int len);
	void W25qxx_Spi_Receive(unsigned char* data, unsigned int len);

	void W25qxx_EraseChip(void);
	void W25qxx_EraseSector(uint32_t SectorAddr);
	void W25qxx_EraseBlock(uint32_t BlockAddr);

	uint32_t W25qxx_PageToSector(uint32_t PageAddress);
	uint32_t W25qxx_PageToBlock(uint32_t PageAddress);
	uint32_t W25qxx_SectorToBlock(uint32_t SectorAddress);
	uint32_t W25qxx_SectorToPage(uint32_t SectorAddress);
	uint32_t W25qxx_BlockToPage(uint32_t BlockAddress);

	bool W25qxx_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize);
	bool W25qxx_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize);
	bool W25qxx_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize);

	void W25qxx_WriteByte(uint8_t pBuffer, uint32_t Bytes_Address);
	void W25qxx_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize);
	void W25qxx_WriteSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize);
	void W25qxx_WriteBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize);

	void W25qxx_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address);
	void W25qxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
	void W25qxx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize);
	void W25qxx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize);
	void W25qxx_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize);

#endif
