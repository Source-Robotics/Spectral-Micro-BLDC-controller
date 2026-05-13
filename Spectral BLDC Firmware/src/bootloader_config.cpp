#include "bootloader_config.h"
#include "stm32f1xx_hal.h"
#include <string.h>

static volatile const bl_vars_t * const BL_FLASH_VARS =
    reinterpret_cast<volatile const bl_vars_t *>(BOOTLOADER_CONFIG_ADDRESS);

void Bootloader_SetVectorTable(void)
{
    SCB->VTOR = APP_BASE_ADDRESS;
}

void Bootloader_ResetIntoBootloader(void)
{
    volatile uint32_t *magic = reinterpret_cast<volatile uint32_t *>(BOOTLOADER_MAGIC_ADDRESS);

    __disable_irq();
    *magic = 0;
    __DSB();
    __ISB();
    NVIC_SystemReset();
}

bool Bootloader_SyncBoardId(uint8_t board_id)
{
    if (board_id > BOOTLOADER_BOARD_ID_MAX)
        return false;

    if (BL_FLASH_VARS->board.id == board_id)
        return true;

    bl_vars_t vars;
    memcpy(&vars, reinterpret_cast<const void *>(BOOTLOADER_CONFIG_ADDRESS), sizeof(vars));
    vars.board.id = board_id;

    __disable_irq();
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

    FLASH_EraseInitTypeDef erase = {};
    uint32_t page_error = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = BOOTLOADER_CONFIG_ADDRESS;
    erase.NbPages = 1;

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_error);
    if (status == HAL_OK) {
        const uint32_t *src = reinterpret_cast<const uint32_t *>(&vars);
        for (uint32_t i = 0; i < (sizeof(vars) / sizeof(uint32_t)); i++) {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       BOOTLOADER_CONFIG_ADDRESS + (i * sizeof(uint32_t)),
                                       src[i]);
            if (status != HAL_OK)
                break;
        }
    }

    HAL_FLASH_Lock();
    __enable_irq();

    if (status != HAL_OK)
        return false;

    return BL_FLASH_VARS->board.id == board_id;
}
