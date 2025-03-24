#ifndef __EFM8_MIO_H__
#define __EFM8_MIO_H__

#define EFM8_MIO_FW_VERSION_MAJ 0

#define EFM8_UPDFILE_MAXSIZE	16384
#define EFM8_BOOTLOADER_I2CADDR 0x78

int efm8_init();
int efm8_get_rotary(void);

#endif /* __EFM8_MIO_H__ */