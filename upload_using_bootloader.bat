set AVRDUDE_HOME=c:\_Tomovo\_common\avrdude

%AVRDUDE_HOME%\avrdude.exe -C%AVRDUDE_HOME%\avrdude.conf -v -patmega328p -carduino -PCOM5 -b115200 -D -v -Uflash:w:sources\GccApplication1\Debug\GccApplication1.hex:i
