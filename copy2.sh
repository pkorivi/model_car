scp -r catkin_ws root@10.42.0.10:./
scp autostart.sh root@10.42.0.10:./
scp .bashrc root@10.42.0.10:./
scp 72-serial.rules root@10.42.0.10:/etc/udev/rules.d/
scp 71-usb-cam.rules root@10.42.0.10:/etc/udev/rules.d/
echo "copy finished!"


