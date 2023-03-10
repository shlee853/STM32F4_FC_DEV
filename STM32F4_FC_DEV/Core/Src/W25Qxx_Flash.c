/*
 * W25QXX_Flash.c
 *
 *  Created on: Feb 26, 2023
 *      Author: swift
 *
 *
 *
 * original code information
/*
  Author:     Nima Askari
  WebSite:    http://www.github.com/NimaLTD
  Instagram:  http://instagram.com/github.NimaLTD
  Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw
  Version:    1.1.4
 */


#include "W25Qxx_Flash.h"


w25qxx_t w25qxx;


void W25QXX_SPI_Initialization(void)
{
	LL_SPI_Enable(W25QXX_SPI_CHANNEL);
	CHIP_DESELECT(W25QXX);

	W25qxx_Init();

}






unsigned char W25qxx_Spi(unsigned char data)
{
	while(LL_SPI_IsActiveFlag_TXE(W25QXX_SPI_CHANNEL)==RESET);
	LL_SPI_TransmitData8(W25QXX_SPI_CHANNEL, data);

	while(LL_SPI_IsActiveFlag_RXNE(W25QXX_SPI_CHANNEL)==RESET);
	return LL_SPI_ReceiveData8(W25QXX_SPI_CHANNEL);
}


void W25qxx_Spi_Receive(unsigned char* data, unsigned int len)
{
	unsigned int i = 0;
	while(i<len) {

		while(LL_SPI_IsActiveFlag_TXE(W25QXX_SPI_CHANNEL)==RESET);
		LL_SPI_TransmitData8(W25QXX_SPI_CHANNEL, W25QXX_DUMMY_BYTE);

		while(LL_SPI_IsActiveFlag_RXNE(W25QXX_SPI_CHANNEL)==RESET);
		data[i++] = LL_SPI_ReceiveData8(W25QXX_SPI_CHANNEL);

	}
}


void W25qxx_Spi_Transmit(unsigned char* data, unsigned int len)
{
	unsigned int i = 0;
	while(i<len) {

		while(LL_SPI_IsActiveFlag_TXE(W25QXX_SPI_CHANNEL)==RESET);
		LL_SPI_TransmitData8(W25QXX_SPI_CHANNEL, data[i++]);

		while(LL_SPI_IsActiveFlag_RXNE(W25QXX_SPI_CHANNEL)==RESET);
		LL_SPI_ReceiveData8(W25QXX_SPI_CHANNEL);

	}
}

//////////////////////////////////////////////////////////////


uint32_t W25qxx_ReadID(void)
{
	uint32_t Temp = 0;
	uint8_t pBuffer[3];

	CHIP_SELECT(W25QXX);
	W25qxx_Spi(REG_ADDR_JEDEC_ID);
	W25qxx_Spi_Receive(pBuffer,sizeof(pBuffer));
	CHIP_DESELECT(W25QXX);
	Temp = (pBuffer[0] << 16) | (pBuffer[1] << 8) | pBuffer[2];
	return Temp;
}

void W25qxx_ReadUniqID(void)
{
	CHIP_SELECT(W25QXX);
	W25qxx_Spi(REG_ADDR_UNIQUE_ID);
	for (uint8_t i = 0; i < 4; i++)
		W25qxx_Spi(W25QXX_DUMMY_BYTE);
	for (uint8_t i = 0; i < 8; i++)
		w25qxx.UniqID[i] = W25qxx_Spi(W25QXX_DUMMY_BYTE);
	CHIP_DESELECT(W25QXX);

#if (_W25QXX_DEBUG == 1)
	for (uint8_t i = 0; i < 8; i++)
		printf("w25qxx UniqID:0x%X\r\n", w25qxx.UniqID[i]);
#endif


}


void W25qxx_WriteEnable(void)
{
	CHIP_SELECT(W25QXX);
	W25qxx_Spi(REG_WRITE_ENBALE);
	CHIP_DESELECT(W25QXX);
	usDelay(1000);
}




void W25qxx_WriteDisable(void)
{
	CHIP_SELECT(W25QXX);
	W25qxx_Spi(REG_WRITE_DISABLE);
	CHIP_DESELECT(W25QXX);
	usDelay(1000);
}




uint8_t W25qxx_ReadStatusRegister(uint8_t SelectStatusRegister_1_2_3)
{
	uint8_t status = 0;
	CHIP_SELECT(W25QXX);
	if (SelectStatusRegister_1_2_3 == 1)
	{
		W25qxx_Spi(REG_READ_STATUS_1);
		status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
		w25qxx.StatusRegister1 = status;
	}
	else if (SelectStatusRegister_1_2_3 == 2)
	{
		W25qxx_Spi(REG_READ_STATUS_2);
		status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
		w25qxx.StatusRegister2 = status;
	}
	else
	{
		W25qxx_Spi(REG_READ_STATUS_3);
		status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
		w25qxx.StatusRegister3 = status;
	}
	CHIP_DESELECT(W25QXX);

#if (_W25QXX_DEBUG == 1)
	printf("w25qxx StatusRegister1:0x%X\r\n", w25qxx.StatusRegister1);
	printf("w25qxx StatusRegister2:0x%X\r\n", w25qxx.StatusRegister2);
	printf("w25qxx StatusRegister3:0x%X\r\n", w25qxx.StatusRegister3);
#endif
	return status;
}


void W25qxx_WriteStatusRegister(uint8_t SelectStatusRegister_1_2_3, uint8_t Data)
{
	CHIP_SELECT(W25QXX);
	if (SelectStatusRegister_1_2_3 == 1)
	{
		W25qxx_Spi(REG_WRITE_STATUS_1);
		w25qxx.StatusRegister1 = Data;
	}
	else if (SelectStatusRegister_1_2_3 == 2)
	{
		W25qxx_Spi(REG_WRITE_STATUS_2);
		w25qxx.StatusRegister2 = Data;
	}
	else
	{
		W25qxx_Spi(REG_WRITE_STATUS_3);
		w25qxx.StatusRegister3 = Data;
	}
	W25qxx_Spi(Data);
	CHIP_DESELECT(W25QXX);
}


void W25qxx_WaitForWriteEnd(void)
{
	usDelay(1000);
	CHIP_SELECT(W25QXX);
	W25qxx_Spi(0x05);
	do
	{
		w25qxx.StatusRegister1 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
		usDelay(1000);
	} while ((w25qxx.StatusRegister1 & REG_WRITE_STATUS_1) == REG_WRITE_STATUS_1);
	CHIP_DESELECT(W25QXX);
}


bool W25qxx_Init(void)
{
	w25qxx.Lock = 1;
	uint32_t id;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx Init Begin...\r\n");
#endif
	id = W25qxx_ReadID();

#if (_W25QXX_DEBUG == 1)
	printf("w25qxx ID:0x%X\r\n", id);
#endif
	switch (id & 0x000000FF)
	{
	case 0x20: // 	w25q512
		w25qxx.ID = W25Q512;
		w25qxx.BlockCount = 1024;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q512\r\n");
#endif
		break;
	case 0x19: // 	w25q256
		w25qxx.ID = W25Q256;
		w25qxx.BlockCount = 512;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q256\r\n");
#endif
		break;
	case 0x18: // 	w25q128
		w25qxx.ID = W25Q128;
		w25qxx.BlockCount = 256;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q128\r\n");
#endif
		break;
	case 0x17: //	w25q64
		w25qxx.ID = W25Q64;
		w25qxx.BlockCount = 128;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q64\r\n");
#endif
		break;
	case 0x16: //	w25q32
		w25qxx.ID = W25Q32;
		w25qxx.BlockCount = 64;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q32\r\n");
#endif
		break;
	case 0x15: //	w25q16
		w25qxx.ID = W25Q16;
		w25qxx.BlockCount = 32;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q16\r\n");
#endif
		break;
	case 0x14: //	w25q80
		w25qxx.ID = W25Q80;
		w25qxx.BlockCount = 16;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q80\r\n");
#endif
		break;
	case 0x13: //	w25q40
		w25qxx.ID = W25Q40;
		w25qxx.BlockCount = 8;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q40\r\n");
#endif
		break;
	case 0x12: //	w25q20
		w25qxx.ID = W25Q20;
		w25qxx.BlockCount = 4;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q20\r\n");
#endif
		break;
	case 0x11: //	w25q10
		w25qxx.ID = W25Q10;
		w25qxx.BlockCount = 2;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q10\r\n");
#endif
		break;
	default:
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Unknown ID\r\n");
#endif
		w25qxx.Lock = 0;
		return false;
	}
	w25qxx.PageSize = 256;
	w25qxx.SectorSize = 0x1000;
	w25qxx.SectorCount = w25qxx.BlockCount * 16;
	w25qxx.PageCount = (w25qxx.SectorCount * w25qxx.SectorSize) / w25qxx.PageSize;
	w25qxx.BlockSize = w25qxx.SectorSize * 16;
	w25qxx.CapacityInKiloByte = (w25qxx.SectorCount * w25qxx.SectorSize) / 1024;
	W25qxx_ReadUniqID();
	W25qxx_ReadStatusRegister(1);
	W25qxx_ReadStatusRegister(2);
	W25qxx_ReadStatusRegister(3);
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx Page Size: %d Bytes\r\n", w25qxx.PageSize);
	printf("w25qxx Page Count: %d\r\n", w25qxx.PageCount);
	printf("w25qxx Sector Size: %d Bytes\r\n", w25qxx.SectorSize);
	printf("w25qxx Sector Count: %d\r\n", w25qxx.SectorCount);
	printf("w25qxx Block Size: %d Bytes\r\n", w25qxx.BlockSize);
	printf("w25qxx Block Count: %d\r\n", w25qxx.BlockCount);
	printf("w25qxx Capacity: %d KiloBytes\r\n", w25qxx.CapacityInKiloByte);
	printf("w25qxx Init Done\r\n");
#endif
	w25qxx.Lock = 0;
	return true;
}


void W25qxx_EraseChip(void)
{
	while (w25qxx.Lock == 1)
		usDelay(1000);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = DWT->CYCCNT;
	printf("w25qxx EraseChip Begin...\r\n");
#endif
	W25qxx_WriteEnable();
	CHIP_SELECT(W25QXX);
	W25qxx_Spi(REG_CHIP_ERASE);
	CHIP_DESELECT(W25QXX);
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseBlock done after %d ms!\r\n", (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC);
#endif
	usDelay(10000);
	w25qxx.Lock = 0;
}

void W25qxx_EraseSector(uint32_t SectorAddr)
{
	while (w25qxx.Lock == 1)
		usDelay(1000);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = DWT->CYCCNT;
	printf("w25qxx EraseSector %d Begin...\r\n", SectorAddr);
#endif
	W25qxx_WaitForWriteEnd();
	SectorAddr = SectorAddr * w25qxx.SectorSize;
	W25qxx_WriteEnable();
	CHIP_SELECT(W25QXX);
	if (w25qxx.ID >= W25Q256)
	{
		W25qxx_Spi(0x21);
		W25qxx_Spi((SectorAddr & 0xFF000000) >> 24);
	}
	else
	{
		W25qxx_Spi(REG_SECTOR_ERASE);
	}
	W25qxx_Spi((SectorAddr & 0xFF0000) >> 16);
	W25qxx_Spi((SectorAddr & 0xFF00) >> 8);
	W25qxx_Spi(SectorAddr & 0xFF);
	CHIP_DESELECT(W25QXX);
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseSector done after %d ms\r\n", (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC);
#endif
	usDelay(1000);
	w25qxx.Lock = 0;
}


void W25qxx_EraseBlock(uint32_t BlockAddr)
{
	while (w25qxx.Lock == 1)
		usDelay(1000);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseBlock %d Begin...\r\n", BlockAddr);
	usDelay(100000);
	uint32_t StartTime = DWT->CYCCNT;
#endif
	W25qxx_WaitForWriteEnd();
	BlockAddr = BlockAddr * w25qxx.SectorSize * 16;
	W25qxx_WriteEnable();
	CHIP_SELECT(W25QXX);
	if (w25qxx.ID >= W25Q256)
	{
		W25qxx_Spi(0xDC);
		W25qxx_Spi((BlockAddr & 0xFF000000) >> 24);
	}
	else
	{
		W25qxx_Spi(REG_BLOCK_ERASE);
	}
	W25qxx_Spi((BlockAddr & 0xFF0000) >> 16);
	W25qxx_Spi((BlockAddr & 0xFF00) >> 8);
	W25qxx_Spi(BlockAddr & 0xFF);
	CHIP_DESELECT(W25QXX);
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseBlock done after %d ms\r\n", (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC);
	usDelay(100000);
#endif
	usDelay(1000);
	w25qxx.Lock = 0;
}

uint32_t W25qxx_PageToSector(uint32_t PageAddress)
{
	return ((PageAddress * w25qxx.PageSize) / w25qxx.SectorSize);
}
//###################################################################################################################
uint32_t W25qxx_PageToBlock(uint32_t PageAddress)
{
	return ((PageAddress * w25qxx.PageSize) / w25qxx.BlockSize);
}
//###################################################################################################################
uint32_t W25qxx_SectorToBlock(uint32_t SectorAddress)
{
	return ((SectorAddress * w25qxx.SectorSize) / w25qxx.BlockSize);
}
//###################################################################################################################
uint32_t W25qxx_SectorToPage(uint32_t SectorAddress)
{
	return (SectorAddress * w25qxx.SectorSize) / w25qxx.PageSize;
}
//###################################################################################################################
uint32_t W25qxx_BlockToPage(uint32_t BlockAddress)
{
	return (BlockAddress * w25qxx.BlockSize) / w25qxx.PageSize;
}



bool W25qxx_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize)
{
	while (w25qxx.Lock == 1)
		usDelay(1000);
	w25qxx.Lock = 1;
	if (((NumByteToCheck_up_to_PageSize + OffsetInByte) > w25qxx.PageSize) || (NumByteToCheck_up_to_PageSize == 0))
		NumByteToCheck_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckPage:%d, Offset:%d, Bytes:%d begin...\r\n", Page_Address, OffsetInByte, NumByteToCheck_up_to_PageSize);
	usDelay(100000);
	uint32_t StartTime = DWT->CYCCNT;
#endif
	uint8_t pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;
	for (i = OffsetInByte; i < w25qxx.PageSize; i += sizeof(pBuffer))
	{
		CHIP_SELECT(W25QXX);
		WorkAddress = (i + Page_Address * w25qxx.PageSize);
		if (w25qxx.ID >= W25Q256)
		{
			W25qxx_Spi(0x0C);
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		}
		else
		{
			W25qxx_Spi(REG_FAST_READ);
		}
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		W25qxx_Spi_Receive(pBuffer, sizeof(pBuffer));
		CHIP_DESELECT(W25QXX);
		for (uint8_t x = 0; x < sizeof(pBuffer); x++)
		{
			if (pBuffer[x] != 0xFF)
				goto NOT_EMPTY;
		}
	}
	if ((w25qxx.PageSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.PageSize; i++)
		{
			CHIP_SELECT(W25QXX);
			WorkAddress = (i + Page_Address * w25qxx.PageSize);
			W25qxx_Spi(REG_FAST_READ);
			if (w25qxx.ID >= W25Q256)
			{
				W25qxx_Spi(0x0C);
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			}
			else
			{
				W25qxx_Spi(REG_FAST_READ);
			}
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			W25qxx_Spi_Receive(pBuffer, sizeof(pBuffer));
			CHIP_DESELECT(W25QXX);
			if (pBuffer[0] != 0xFF)
				goto NOT_EMPTY;
		}
	}
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckPage is Empty in %d ms\r\n", (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC);
	usDelay(100000);
#endif
	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckPage is Not Empty in %d ms\r\n", (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC);
	usDelay(100000);
#endif
	w25qxx.Lock = 0;
	return false;
}



bool W25qxx_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize)
{
	while (w25qxx.Lock == 1)
		usDelay(1000);
	w25qxx.Lock = 1;
	if ((NumByteToCheck_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToCheck_up_to_SectorSize == 0))
		NumByteToCheck_up_to_SectorSize = w25qxx.SectorSize;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckSector:%d, Offset:%d, Bytes:%d begin...\r\n", Sector_Address, OffsetInByte, NumByteToCheck_up_to_SectorSize);
	usDelay(100000);
	uint32_t StartTime = DWT->CYCCNT;
#endif
	uint8_t pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;
	for (i = OffsetInByte; i < w25qxx.SectorSize; i += sizeof(pBuffer))
	{
		CHIP_SELECT(W25QXX);
		WorkAddress = (i + Sector_Address * w25qxx.SectorSize);
		if (w25qxx.ID >= W25Q256)
		{
			W25qxx_Spi(0x0C);
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		}
		else
		{
			W25qxx_Spi(REG_FAST_READ);
		}
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		W25qxx_Spi_Receive(pBuffer, sizeof(pBuffer));
		CHIP_DESELECT(W25QXX);
		for (uint8_t x = 0; x < sizeof(pBuffer); x++)
		{
			if (pBuffer[x] != 0xFF)
				goto NOT_EMPTY;
		}
	}
	if ((w25qxx.SectorSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.SectorSize; i++)
		{
			CHIP_SELECT(W25QXX);
			WorkAddress = (i + Sector_Address * w25qxx.SectorSize);
			if (w25qxx.ID >= W25Q256)
			{
				W25qxx_Spi(0x0C);
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			}
			else
			{
				W25qxx_Spi(REG_FAST_READ);
			}
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			W25qxx_Spi_Receive(pBuffer, sizeof(pBuffer));
			CHIP_DESELECT(W25QXX);
			if (pBuffer[0] != 0xFF)
				goto NOT_EMPTY;
		}
	}
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckSector is Empty in %d ms\r\n", (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC);
	usDelay(100000);
#endif
	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckSector is Not Empty in %d ms\r\n", (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC);
	usDelay(100000);
#endif
	w25qxx.Lock = 0;
	return false;
}



bool W25qxx_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize)
{
	while (w25qxx.Lock == 1)
		usDelay(1000);
	w25qxx.Lock = 1;
	if ((NumByteToCheck_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToCheck_up_to_BlockSize == 0))
		NumByteToCheck_up_to_BlockSize = w25qxx.BlockSize;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckBlock:%d, Offset:%d, Bytes:%d begin...\r\n", Block_Address, OffsetInByte, NumByteToCheck_up_to_BlockSize);
	usDelay(100000);
	uint32_t StartTime = DWT->CYCCNT;
#endif
	uint8_t pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;
	for (i = OffsetInByte; i < w25qxx.BlockSize; i += sizeof(pBuffer))
	{
		CHIP_SELECT(W25QXX);
		WorkAddress = (i + Block_Address * w25qxx.BlockSize);

		if (w25qxx.ID >= W25Q256)
		{
			W25qxx_Spi(0x0C);
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		}
		else
		{
			W25qxx_Spi(REG_FAST_READ);
		}
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		W25qxx_Spi_Receive(pBuffer, sizeof(pBuffer));
		CHIP_DESELECT(W25QXX);
		for (uint8_t x = 0; x < sizeof(pBuffer); x++)
		{
			if (pBuffer[x] != 0xFF)
				goto NOT_EMPTY;
		}
	}
	if ((w25qxx.BlockSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.BlockSize; i++)
		{
			CHIP_SELECT(W25QXX);
			WorkAddress = (i + Block_Address * w25qxx.BlockSize);

			if (w25qxx.ID >= W25Q256)
			{
				W25qxx_Spi(0x0C);
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			}
			else
			{
				W25qxx_Spi(REG_FAST_READ);
			}
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			W25qxx_Spi_Receive(pBuffer, sizeof(pBuffer));
			CHIP_DESELECT(W25QXX);
			if (pBuffer[0] != 0xFF)
				goto NOT_EMPTY;
		}
	}
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckBlock is Empty in %d ms\r\n", (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC);
	usDelay(100000);
#endif
	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckBlock is Not Empty in %d ms\r\n", (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC);
	usDelay(100000);
#endif
	w25qxx.Lock = 0;
	return false;
}



void W25qxx_WriteByte(uint8_t pBuffer, uint32_t WriteAddr_inBytes)
{
	while (w25qxx.Lock == 1)
		usDelay(1000);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = DWT->CYCCNT;
	printf("w25qxx WriteByte 0x%02X at address %d begin...", pBuffer, WriteAddr_inBytes);
#endif
	W25qxx_WaitForWriteEnd();
	W25qxx_WriteEnable();
	CHIP_SELECT(W25QXX);

	if (w25qxx.ID >= W25Q256)
	{
		W25qxx_Spi(0x12);
		W25qxx_Spi((WriteAddr_inBytes & 0xFF000000) >> 24);
	}
	else
	{
		W25qxx_Spi(REG_PAGE_PROGRAM);
	}
	W25qxx_Spi((WriteAddr_inBytes & 0xFF0000) >> 16);
	W25qxx_Spi((WriteAddr_inBytes & 0xFF00) >> 8);
	W25qxx_Spi(WriteAddr_inBytes & 0xFF);
	W25qxx_Spi(pBuffer);
	CHIP_DESELECT(W25QXX);
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx WriteByte done after %d ms\r\n", (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC);
#endif
	w25qxx.Lock = 0;
}



void W25qxx_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize)
{
	while (w25qxx.Lock == 1)
		usDelay(1000);
	w25qxx.Lock = 1;
	if (((NumByteToWrite_up_to_PageSize + OffsetInByte) > w25qxx.PageSize) || (NumByteToWrite_up_to_PageSize == 0))
		NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
	if ((OffsetInByte + NumByteToWrite_up_to_PageSize) > w25qxx.PageSize)
		NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx WritePage:%d, Offset:%d ,Writes %d Bytes, begin...\r\n", Page_Address, OffsetInByte, NumByteToWrite_up_to_PageSize);
	usDelay(100000);
	uint32_t StartTime = DWT->CYCCNT;
#endif
	W25qxx_WaitForWriteEnd();
	W25qxx_WriteEnable();
	CHIP_SELECT(W25QXX);
	Page_Address = (Page_Address * w25qxx.PageSize) + OffsetInByte;
	if (w25qxx.ID >= W25Q256)
	{
		W25qxx_Spi(0x12);
		W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
	}
	else
	{
		W25qxx_Spi(REG_PAGE_PROGRAM);
	}
	W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
	W25qxx_Spi((Page_Address & 0xFF00) >> 8);
	W25qxx_Spi(Page_Address & 0xFF);
	W25qxx_Spi_Transmit(pBuffer, NumByteToWrite_up_to_PageSize);
	CHIP_DESELECT(W25QXX);
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	StartTime = (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC;
	for (uint32_t i = 0; i < NumByteToWrite_up_to_PageSize; i++)
	{
		if ((i % 8 == 0) && (i > 2))
		{
			printf("\r\n");
			usDelay(10000);
		}
		printf("0x%02X,", pBuffer[i]);
	}
	printf("\r\n");
	printf("w25qxx WritePage done after %d ms\r\n", StartTime);
	usDelay(100000);
#endif
	usDelay(1000);
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_WriteSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize)
{
	if ((NumByteToWrite_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToWrite_up_to_SectorSize == 0))
		NumByteToWrite_up_to_SectorSize = w25qxx.SectorSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx WriteSector:%d, Offset:%d ,Write %d Bytes, begin...\r\n", Sector_Address, OffsetInByte, NumByteToWrite_up_to_SectorSize);
	usDelay(100000);
#endif
	if (OffsetInByte >= w25qxx.SectorSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("---w25qxx WriteSector Faild!\r\n");
		usDelay(100000);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToWrite;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToWrite_up_to_SectorSize) > w25qxx.SectorSize)
		BytesToWrite = w25qxx.SectorSize - OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_SectorSize;
	StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
		StartPage++;
		BytesToWrite -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx WriteSector Done\r\n");
	usDelay(100000);
#endif
}
//###################################################################################################################
void W25qxx_WriteBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize)
{
	if ((NumByteToWrite_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToWrite_up_to_BlockSize == 0))
		NumByteToWrite_up_to_BlockSize = w25qxx.BlockSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx WriteBlock:%d, Offset:%d ,Write %d Bytes, begin...\r\n", Block_Address, OffsetInByte, NumByteToWrite_up_to_BlockSize);
	usDelay(100000);
#endif
	if (OffsetInByte >= w25qxx.BlockSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("---w25qxx WriteBlock Faild!\r\n");
		usDelay(100000);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToWrite;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToWrite_up_to_BlockSize) > w25qxx.BlockSize)
		BytesToWrite = w25qxx.BlockSize - OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_BlockSize;
	StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
		StartPage++;
		BytesToWrite -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx WriteBlock Done\r\n");
	usDelay(100000);
#endif
}

//###################################################################################################################
void W25qxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	while (w25qxx.Lock == 1)
		usDelay(1000);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = DWT->CYCCNT;
	printf("w25qxx ReadBytes at Address:%d, %d Bytes  begin...\r\n", ReadAddr, NumByteToRead);
#endif
	CHIP_SELECT(W25QXX);

	if (w25qxx.ID >= W25Q256)
	{
		W25qxx_Spi(0x0C);
		W25qxx_Spi((ReadAddr & 0xFF000000) >> 24);
	}
	else
	{
		W25qxx_Spi(REG_FAST_READ);
	}
	W25qxx_Spi((ReadAddr & 0xFF0000) >> 16);
	W25qxx_Spi((ReadAddr & 0xFF00) >> 8);
	W25qxx_Spi(ReadAddr & 0xFF);
	W25qxx_Spi(0);
	W25qxx_Spi_Receive(pBuffer, NumByteToRead);
	CHIP_DESELECT(W25QXX);
#if (_W25QXX_DEBUG == 1)
	StartTime = (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC;
	for (uint32_t i = 0; i < NumByteToRead; i++)
	{
		if ((i % 8 == 0) && (i > 2))
		{
			printf("\r\n");
			usDelay(10000);
		}
		printf("0x%02X,", pBuffer[i]);
	}
	printf("\r\n");
	printf("w25qxx ReadBytes done after %d ms\r\n", StartTime);
	usDelay(100000);
#endif
	usDelay(1000);
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize)
{
	while (w25qxx.Lock == 1)
		usDelay(1000);
	w25qxx.Lock = 1;
	if ((NumByteToRead_up_to_PageSize > w25qxx.PageSize) || (NumByteToRead_up_to_PageSize == 0))
		NumByteToRead_up_to_PageSize = w25qxx.PageSize;
	if ((OffsetInByte + NumByteToRead_up_to_PageSize) > w25qxx.PageSize)
		NumByteToRead_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx ReadPage:%d, Offset:%d ,Read %d Bytes, begin...\r\n", Page_Address, OffsetInByte, NumByteToRead_up_to_PageSize);
	usDelay(100000);
	uint32_t StartTime = DWT->CYCCNT;
#endif
	Page_Address = Page_Address * w25qxx.PageSize + OffsetInByte;
	CHIP_SELECT(W25QXX);
	if (w25qxx.ID >= W25Q256)
	{
		W25qxx_Spi(0x0C);
		W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
	}
	else
	{
		W25qxx_Spi(REG_FAST_READ);
	}
	W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
	W25qxx_Spi((Page_Address & 0xFF00) >> 8);
	W25qxx_Spi(Page_Address & 0xFF);
	W25qxx_Spi(0);
	W25qxx_Spi_Receive(pBuffer, NumByteToRead_up_to_PageSize);
	CHIP_DESELECT(W25QXX);
#if (_W25QXX_DEBUG == 1)
	StartTime = (int32_t)(DWT->CYCCNT - StartTime)/CLOCK_PER_MSEC;
	for (uint32_t i = 0; i < NumByteToRead_up_to_PageSize; i++)
	{
		if ((i % 8 == 0) && (i > 2))
		{
			printf("\r\n");
			usDelay(10000);
		}
		printf("0x%02X,", pBuffer[i]);
	}
	printf("\r\n");
	printf("w25qxx ReadPage done after %d ms\r\n", StartTime);
	usDelay(100000);
#endif
	usDelay(1000);
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize)
{
	if ((NumByteToRead_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToRead_up_to_SectorSize == 0))
		NumByteToRead_up_to_SectorSize = w25qxx.SectorSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx ReadSector:%d, Offset:%d ,Read %d Bytes, begin...\r\n", Sector_Address, OffsetInByte, NumByteToRead_up_to_SectorSize);
	usDelay(100000);
#endif
	if (OffsetInByte >= w25qxx.SectorSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("---w25qxx ReadSector Faild!\r\n");
		usDelay(100000);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToRead;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToRead_up_to_SectorSize) > w25qxx.SectorSize)
		BytesToRead = w25qxx.SectorSize - OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_SectorSize;
	StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
		StartPage++;
		BytesToRead -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToRead > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx ReadSector Done\r\n");
	usDelay(100000);
#endif
}
//###################################################################################################################
void W25qxx_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize)
{
	if ((NumByteToRead_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToRead_up_to_BlockSize == 0))
		NumByteToRead_up_to_BlockSize = w25qxx.BlockSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx ReadBlock:%d, Offset:%d ,Read %d Bytes, begin...\r\n", Block_Address, OffsetInByte, NumByteToRead_up_to_BlockSize);
	usDelay(100000);
#endif
	if (OffsetInByte >= w25qxx.BlockSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx ReadBlock Faild!\r\n");
		usDelay(100000);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToRead;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToRead_up_to_BlockSize) > w25qxx.BlockSize)
		BytesToRead = w25qxx.BlockSize - OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_BlockSize;
	StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
		StartPage++;
		BytesToRead -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToRead > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx ReadBlock Done\r\n");
	usDelay(100000);
#endif
}





