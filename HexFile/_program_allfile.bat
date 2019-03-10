@echo off

rem 设置对应文件的名称
set newapplication_name=..\SourceCode\_build\GreenLung_s130_app.hex
rem set newapplication_name=D:\Nordic\SDK\nRF5_SDK_12.3.0_d7731ad\examples\ble_peripheral\experimental_ble_app_buttonless_dfu\pca10028\s130\arm5_no_packs\_build\App_nrf51422_xxac_S130.hex
set application_name=GreenLung_S130_App.hex
set bootloader_name=..\dfu\dfu\bootloader_secure\pca10028\arm5_no_packs\_build\GreenLung_s130_BL.hex
set merge_file_name=GreenLung_s130_merge_file.hex
set merge_file_All_name=GreenLung_s130_merge_file_All.hex
set setting_file_name=GreenLung_S130_setting.hex
set softdevice_namemerge=..\s130_nrf51_2.0.1_softdevice.hex


rem 擦除 chip 烧录chip
cls
echo ****************program file****************
echo %merge_file_All_name%
nrfjprog.exe --program %merge_file_All_name% --chiperase
IF NOT %ERRORLEVEL% == 0 (
	echo *****************program file Failed****************
	goto end
)
cls
echo *****************program file succeed****************
ping /n 3 127.0.0.1
rem 验证 chip
cls
echo ****************verify file****************
echo %merge_file_All_name%
nrfjprog.exe --verify %merge_file_All_name%
IF NOT %ERRORLEVEL% == 0 (
	echo *****************verify file Failed****************
	goto end
)
echo *****************verify file succeed****************

rem nrfjprog.exe --reset
rem cls

writeISN.bat

echo *****************PASS*****************
:end
ping /n 10 127.0.0.1