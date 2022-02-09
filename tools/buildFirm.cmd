copy build\BrewDevice.ino.bin tools\* /Y
copy build\BrewDevice.mklittlefs.bin tools\* /Y
cd tools
del BrewDevice.zip
"C:\Program Files\7-Zip\7z.exe" a BrewDevice.zip BrewDevice.ino.bin BrewDevice.mklittlefs.bin Flashen.cmd esptool.exe
