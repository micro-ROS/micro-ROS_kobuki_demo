cd /

git clone --single-branch --branch cf_micro-xrce-dds https://github.com/eProsima/crazyflie-firmware.git

cd crazyflie-firmware
git submodule init
git submodule update
make PLATFORM=cf2

sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D cf2.bin