#ifndef BOOTLOADER_CONFIG_H
#define BOOTLOADER_CONFIG_H

#include <stdint.h>

#define APP_BASE_ADDRESS 0x08003000UL
#define BOOTLOADER_CONFIG_ADDRESS 0x08002C00UL
#define BOOTLOADER_CONFIG_PAGE_SIZE 0x400UL
#define BOOTLOADER_MAGIC_ADDRESS 0x20001000UL
#define SPECTRAL_CAN_ID_MAX 15U
#define BOOTLOADER_BOARD_ID_MAX 13U

struct bl_app_vars_t {
    uint32_t page_count;
    uint32_t crc;
};

struct bl_board_vars_t {
    uint64_t bl_build_version;
    uint8_t id;
    uint8_t padding[3];
};

struct bl_vars_t {
    bl_app_vars_t app;
    bl_board_vars_t board;
};

static_assert((sizeof(bl_vars_t) % sizeof(uint32_t)) == 0,
              "Bootloader config size must be word-aligned");


static inline bool spectral_can_id_is_valid(uint8_t id)
{
    return id <= SPECTRAL_CAN_ID_MAX;
}

static inline bool spectral_can_id_int_is_valid(int32_t id)
{
    return id >= 0 && id <= SPECTRAL_CAN_ID_MAX;
}

void Bootloader_SetVectorTable(void);
void Bootloader_ResetIntoBootloader(void);
bool Bootloader_SyncBoardId(uint8_t board_id);

#endif
