python3 /crazyflie-clients-python/bin/cfclient &> /dev/null &

MicroXRCEAgent serial --dev $(cat used_serialport.txt) & 