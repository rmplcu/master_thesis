sudo chmod 777 /dev/ttyUSB*
sudo chmod 777 /dev/input/js0
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
sudo udevadm control --reload-rules && sudo udevadm trigger