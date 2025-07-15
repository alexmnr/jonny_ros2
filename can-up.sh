sudo ip link set can0 type can loopback on bitrate 500000 restart-ms 1000 
sudo ip link set can0 txqueuelen 1000 
sudo ip link set can0 up
