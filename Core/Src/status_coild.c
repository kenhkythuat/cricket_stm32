/*
 * status_coild.c
 *
 *  Created on: Aug 31, 2024
 *      Author: admin
 */

#include "main.h"
#include "stdio.h"

#define Address 0x807D000

int val;
int Read;
uint32_t status;
uint32_t value_page0;
uint32_t value_page1;
uint32_t value_page2;
uint32_t value_page3;
uint32_t value_Relay;
uint32_t status_load[4];
void Flash_Erase(uint32_t numberpages) {
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef pEraseInit;
  pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  pEraseInit.PageAddress = Address;
  pEraseInit.NbPages = numberpages;
  uint32_t PageError = 0;
  HAL_FLASHEx_Erase(&pEraseInit, &PageError);
  HAL_FLASH_Lock();
}

void read_flash_payload(void)
{
	for(int i=0;i<=3;i++)
	{
		static int temp;
		temp=Read_Page(Address+(i*16));
		HAL_GPIO_WritePin(GPIO_LOAD_PORT[payLoadPin+i], GPIO_LOAD_PIN[payLoadPin+i],temp);
	}
	onReay = *(uint32_t *)(Address + 64);
}


uint32_t Read_Page(uint32_t Address_ex) {
  value_page0 = *(uint32_t *)(Address_ex);
  return value_page0;
}
void Flash_write(int move, uint32_t Data) {
  HAL_FLASH_Unlock();
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address + move, Data);

  HAL_FLASH_Lock();
}

void read_statusload() {
  Flash_Erase(1);
  for (int i = 0; i < 4; i++) {
    Read = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i], GPIO_LOAD_PIN[i]);
    status_load[val] = Read;
    val++;
  }
  val = 0;
  Flash_write(0, status_load[0]);
  printf("Write ok at 0x%08X\r\n", Address + 0);
  Flash_write(16, status_load[1]);
  printf("Write ok at 0x%08X\r\n", Address + 16);
  Flash_write(32, status_load[2]);
  printf("Write ok at 0x%08X\r\n", Address + 32);
  Flash_write(48, status_load[3]);
  printf("Write ok at 0x%08X\r\n\n", Address + 48);
  Flash_write(64, onReay);
  printf("Write ok at 0x%08X\r\n", Address + 64);
}
