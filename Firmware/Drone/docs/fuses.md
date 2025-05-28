*** Mega2560 Fuses ***

Set MEGA to internal oscillator 8MHz
```
avrdude -c usbasp -p m2560 -U lfuse:w:0xE2:m
```

Set MEGA to internal oscillator 1MHz
Use JP3 on the programmer to program on this frequency
```
avrdude -c usbasp -p m2560 -U lfuse:w:0x62:m
```

Workable fuses
https://eleccelerator.com/fusecalc/fusecalc.php?chip=atmega2560

```
avrdude -c usbasp -p m2560 -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h

avrdude: AVR device initialized and ready to accept instructions
avrdude: device signature = 0x1e9801 (probably m2560)
avrdude: reading lfuse memory ...
avrdude: writing output file <stdout>
0xe2
avrdude: reading hfuse memory ...
avrdude: writing output file <stdout>
0xd9
avrdude: reading efuse memory ...
avrdude: writing output file <stdout>
0xfd

avrdude done.  Thank you.
```