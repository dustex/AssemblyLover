@ECHO OFF
"avrasm2.exe" -S "assl-labels.tmp" -fI -W+ie -C V2E -o "assl.hex" -d "assl.obj" -e "assl.eep" -m "assl.map" -l "assl.lst" "assl.asm"
