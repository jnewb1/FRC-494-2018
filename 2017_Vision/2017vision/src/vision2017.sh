v4l2-ctl -d /dev/v4l/by-id/usb-KYE_Systems_Corp._USB_Camera_200901010001-video-index0 --set-ctrl exposure_auto=1 --set-ctrl exposure_absolute=166

while [ 1 ];
 do /home/pi/projects/2017vision/bin/ARM/Debug/2017vision.out && break;
 done