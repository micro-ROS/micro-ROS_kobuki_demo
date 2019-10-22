source /opt/ros/dashing/setup.bash
rm /used_serialport.txt
python3 /crazyflie-clients-python/bin/cfclient &> /dev/null &
while [ ! -f /used_serialport.txt ] ;
do
      sleep 0.1
done

MicroXRCEAgent serial --dev $(cat used_serialport.txt) 