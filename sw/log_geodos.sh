echo "GEODOS02 data logger"
echo "To exit, pres ctrl-a, ctrl-x"
echo " "

mkdir ~/GEODOS02/ >/dev/null 2>&1
picocom -b 115200 -q /dev/ttyUSB0 | awk '{ print strftime("%s,")$0; fflush(); }' | tee ~/GEODOS02/GEODOS_$(date +"%Y%m%d_%H%M%S").log
