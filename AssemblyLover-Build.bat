@ECHO OFF
"avrasm2.exe" -S "labels.tmp" -fI -W+ie -C V2E -o "ass_seminar.hex" -d "ass_seminar.obj" -e "ass_seminar.eep" -m "ass_seminar.map" -l "ass_seminar.lst" "ass_seminar.asm"