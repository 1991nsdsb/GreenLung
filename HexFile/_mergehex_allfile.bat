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


rem 删除旧档案
echo ****************delete old files****************
del %application_name%
del %merge_file_name%
del %merge_file_All_name%
del %setting_file_name%

ping /n 3 127.0.0.1 

rem 获取最新app
cls
echo ****************get app****************
echo %newapplication_name%
copy %newapplication_name% %application_name%
IF NOT %ERRORLEVEL% == 0 (
	echo ****************geting app Failed****************
	goto end
)
echo ****************geting app succeed****************
echo new file %application_name%
ping /n 5 127.0.0.1 
rem 产生setting file
cls
echo ****************generate setting file****************
nrfutil settings generate --family NRF51 --application %application_name% --application-version 1 --bootloader-version 1 --bl-settings-version 1 %setting_file_name%
IF NOT %ERRORLEVEL% == 0 (
	echo *****************generate setting file Failed****************
	goto end
)
echo *****************generate setting file succeed****************
echo setting file %setting_file_name%
ping /n 3 127.0.0.1 

rem 合并 softdevice bootloader application file
cls
echo ****************merge file****************
echo bootloader file %bootloader_name%
echo application file %application_name%
echo softdevice file %softdevice_namemerge%
mergehex --merge %softdevice_namemerge% %bootloader_name% %application_name% --output %merge_file_name%
IF NOT %ERRORLEVEL% == 0 (
	echo *****************merge file Failed****************
	goto end
)
echo *****************merge file succeed****************
echo merge file %merge_file_name%
ping /n 3 127.0.0.1 

rem 合并setting file
cls
echo ****************merge setting file****************
echo setting file %setting_file_name%
mergehex --merge %setting_file_name% %merge_file_name% --output %merge_file_All_name%
IF NOT %ERRORLEVEL% == 0 (
	echo *****************merge setting file Failed****************
	goto end
)
echo *****************merge setting file succeed****************
echo all file %merge_file_All_name%
ping /n 3 127.0.0.1 

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

nrfjprog.exe --reset
cls
echo *****************PASS*****************
:end
pause