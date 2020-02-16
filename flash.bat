set ARDUINO_HOME="c:\_Tomovo\_common\arduino-1.8.10"
set AVRDUDE=%ARDUINO_HOME%"\hardware\tools\avr\bin\avrdude"

%AVRDUDE% -C%ARDUINO_HOME%\hardware\tools\avr/etc/avrdude.conf -v -patmega328p -cbuspirate -PCOM6 -Uflash:w:sources\GccApplication1\Debug\GccApplication1.hex:i  -xascii
