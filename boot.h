#define BOOTLOADER_API_TABLE_SIZE  32
#define BOOTLOADER_API_TABLE_START ((FLASHEND + 1UL) - BOOTLOADER_API_TABLE_SIZE)
#define BOOTLOADER_API_CALL(Index) (void *)((BOOTLOADER_API_TABLE_START + (Index * 2)) / 2)

void (*BootloaderAPI_ErasePage)(uint32_t Address) = BOOTLOADER_API_CALL(0);
void (*BootloaderAPI_WritePage)(uint32_t Address) = BOOTLOADER_API_CALL(1);
void (*BootloaderAPI_FillWord)(uint32_t Address, uint16_t Word) = BOOTLOADER_API_CALL(2);
uint8_t (*BootloaderAPI_ReadSignature)(uint16_t Address) = BOOTLOADER_API_CALL(3);
uint8_t (*BootloaderAPI_ReadFuse)(uint16_t Address) = BOOTLOADER_API_CALL(4);
uint8_t (*BootloaderAPI_ReadLock)(void) = BOOTLOADER_API_CALL(5);
void (*BootloaderAPI_WriteLock)(uint8_t LockBits) = BOOTLOADER_API_CALL(6);
void (*BootloaderAPI_GoDFU)(void) = BOOTLOADER_API_CALL(7);

#define BOOTLOADER_MAGIC_SIGNATURE_START (BOOTLOADER_API_TABLE_START + (BOOTLOADER_API_TABLE_SIZE - 2))
#define BOOTLOADER_MAGIC_SIGNATURE       0xDCFB

#define BOOTLOADER_CLASS_SIGNATURE_START (BOOTLOADER_API_TABLE_START + (BOOTLOADER_API_TABLE_SIZE - 4))
#define BOOTLOADER_DFU_SIGNATURE         0xDF10

#define BOOTLOADER_ADDRESS_START         (BOOTLOADER_API_TABLE_START + (BOOTLOADER_API_TABLE_SIZE - 8))
#define BOOTLOADER_ADDRESS_LENGTH        4

/*
 *  From the application the API support of the bootloader can be detected by reading the FLASH memory bytes located at address
 *  \c BOOTLOADER_MAGIC_SIGNATURE_START and comparing them to the value \c BOOTLOADER_MAGIC_SIGNATURE. The class of bootloader
 *  can be determined by reading the FLASH memory bytes located at address \c BOOTLOADER_CLASS_SIGNATURE_START and comparing them
 *  to the value \c BOOTLOADER_DFU_SIGNATURE. The start address of the bootloader can be retrieved by reading the bytes of FLASH
 *  memory starting from address \c BOOTLOADER_ADDRESS_START.
 */
