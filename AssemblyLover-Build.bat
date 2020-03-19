@ECHO OFF
"avrasm2.exe" -S "assemblyLover-Labes.tmp" -fI -W+ie -C V2E -o "assemblyLover.hex" -d "assemblyLover.obj" -e "assemblyLover.eep" -m "assemblyLover.map" -l "assemblyLover.lst" "assemblyLover.asm"
