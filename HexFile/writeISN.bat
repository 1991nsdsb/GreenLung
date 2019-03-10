
rem ex. E0100001
rem 第一码 E 代表elecomed
rem 第二三码
rem 	01 GreenLung
rem 	02 Handihaler
rem 	03 Sunnyhaler
rem 	04 
rem 	05 



nrfjprog.exe --memwr 0x10001080 --val 0x30333045
IF NOT %ERRORLEVEL% == 0 (
	echo *****************program file Failed****************
	goto end
)

nrfjprog.exe --memwr 0x10001084 --val 0x39313030
IF NOT %ERRORLEVEL% == 0 (
	echo *****************program file Failed****************
	goto end
)

nrfjprog.exe --reset
IF NOT %ERRORLEVEL% == 0 (
	echo *****************program file Failed****************
	goto end
)

echo *****************PASS*****************
:end
ping /n 5 127.0.0.1