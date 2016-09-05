#! /bin/bash
rm ./main
g++ -std=c++11 -g ./future_net/route.cpp ./future_net/future_net.cpp ./future_net/io.cpp -O3 -o main
#g++ -std=c++11 -g ./future_net/route.cpp ./future_net/future_net.cpp ./future_net/io.cpp -o main
opcontrol --deinit
opcontrol --init
opcontrol --vmlinux=/usr/lib/debug/boot/vmlinux-3.13.0-24-generic
opcontrol --reset
opcontrol --buffer-size=0
opcontrol --buffer-watershed=0
opcontrol --cpu-buffer-size=0
#opcontrol --event=CPU_CLK_UNHALTED:60000
opcontrol --event=CPU_CLK_UNHALTED:600000
#opcontrol --event=LLC_MISSES:60000 --event=CPU_CLK_UNHALTED:600000
#opcontrol --event=LLC_REFS:6000 --event=LLC_MISSES:6000
opcontrol --status
opcontrol --start

TOPO=./test-case/mycase2000/topo.csv
DEMAND=./test-case/mycase2000/demand.csv
RESULT=./result.txt
time ./main $TOPO $DEMAND $RESULT

opcontrol --dump
opcontrol --stop
opcontrol -h
#opreport -l
#opreport 

opannotate --source ./main | less -N

