@ECHO OFF
CLS
SET SCRIPT_LOCATION=%~dp0
cd %SCRIPT_LOCATION%
echo Lösche Flashspeicher
esptool.exe -cp COM3 -cd nodemcu -ce
echo Flashe Firmware und LittleFS 
esptool.exe -cp COM3 -cd nodemcu -ca 0x000000 -cf BrewDevice.ino.bin -ca 0x200000 -cf BrewDevice.mklittlefs.bin
echo Enter zum Beenden
pause
exit
