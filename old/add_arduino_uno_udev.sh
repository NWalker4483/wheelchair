echo  'SUBSYSTEM=="tty", ATTRS{idVendor}=="2a03", ATTRS{idProduct}=="0043", MODE:="0777", GROUP:="dialout", SYMLINK+="arduino_uno"' >/etc/udev/rules.d/arduino_uno.rules

sudo adduser $USER dialout

service udev reload
sleep 2
service udev restart