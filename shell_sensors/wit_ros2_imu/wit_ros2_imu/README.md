1️⃣ Create the rule file
sudo nano /etc/udev/rules.d/99-imu.rules

Paste:
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", SYMLINK+="imu_usb"

2️⃣ Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger


3️⃣ check
ls -l /dev/imu_usb
