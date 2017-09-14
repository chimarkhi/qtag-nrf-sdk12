@echo off

set soft_device=..\..\..\..\components\softdevice\s132\hex\s132_nrf52_3.0.0_softdevice.hex
set bootloader=.\bootloader_hex\nrf52832_xxaa_s132.hex
set app=..\pca10040\s132\armgcc\_build\nrf52832_xxaa.hex
set out_zip=temp_queuing_dfu_app.zip
set priv_key=priv.pem
set public_key=public_key.c

:: If keys need to be generated 
if not $%1$ == $GEN_KEYS$ goto create_zip
	nrfutil keys generate priv.pem
	nrfutil keys display --key pk --format code %priv_key% --out_file %public_key%
	echo Generated private and public keys
	
:create_zip	

if not $%1$ == $PUSH_TO_PHONE$ goto flash_device 
:: Generate zip of only application
nrfutil pkg generate --application  %app% --application-version 0xff --application-version-string 0.0.1 --hw-version 52 --sd-req 0x8C,0x91 --key-file %priv_key% %out_zip%
adb wait-for-device push %out_zip% /sdcard/Download/

:flash_device
if not $1$ == $FLASH_DEVICE$ goto EOF
:: start by erasing all
nrfjprog --eraseall -f nrf52

:: program softdevice
nrfjprog --program  %soft_device% -f nrf52 --sectorerase 

:: program bootloader
nrfjprog --reset --program %bootloader% -f nrf52

:: build and program application 
cd ..\pca10040\s132\armgcc\
make flash
cd ..\..\..\DFU\

:EOF
