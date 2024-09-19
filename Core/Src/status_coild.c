/*
 * status_coild.c
 *
 *  Created on: Aug 31, 2024
 *      Author: admin
 */

#include "config.h"
#include "main.h"
#include "stdio.h"

#define Address 0x807D000

#define ADDRESS_INIT_FLASH 0x807E000
uint8_t is_init_flash = 0;
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

void read_flash_payload(void) {
  for (int i = 0; i < NUMBER_LOADS; i++) {
    static int temp;
    temp = Read_Page(Address + (i * 16));
    HAL_GPIO_WritePin(GPIO_LOAD_PORT[payLoadPin + i], GPIO_LOAD_PIN[payLoadPin + i], temp);
  }
  onReay = *(uint32_t *)(Address + 64);
  if (onReay > 0) {
    HAL_GPIO_WritePin(ON_OFF_PWM_GPIO_Port, ON_OFF_PWM_Pin, 0);
  }
  if (onReay <= 0) {
    onReay = 0;
    HAL_GPIO_WritePin(ON_OFF_PWM_GPIO_Port, ON_OFF_PWM_Pin, 1);
  }
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
  for (int i = 0; i < NUMBER_LOADS; i++) {
    Read = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i], GPIO_LOAD_PIN[i]);
    status_load[val] = Read;
    val++;
    Flash_write(i * 16, status_load[i]);
  }
  val = 0;
}
void init_flash(void) {
  is_init_flash = *(uint32_t *)(ADDRESS_INIT_FLASH + 80);
  printf("-----init flash %d-----\n", is_init_flash);
  if (is_init_flash != 1) {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef pEraseInit;
    pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    pEraseInit.PageAddress = ADDRESS_INIT_FLASH;
    pEraseInit.NbPages = 1;
    uint32_t PageError = 0;
    HAL_FLASHEx_Erase(&pEraseInit, &PageError);
    HAL_FLASH_Lock();
    for (int i = 0; i < 4; i++) {
      status_load[val] = 0;
      val++;
      Flash_write(i * 16, status_load[i]);
    }
    val = 0;
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDRESS_INIT_FLASH + 80, 1);
    HAL_FLASH_Lock();
    printf("-----init flash done-----\n");
  }
}
