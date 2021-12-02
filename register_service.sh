#! /usr/bin/sh

echo -n "Checking if $USER belongs to group dialout"
if id -nG "$USER" | grep -qw "dialout"; then
    echo ...success
else
    echo " ... ERROR
$USER does not belong to group dialout
please add user and reboot
    sudo groupadd dialout
    usermod -a -G dialout
    shutdown -r now
    "
    exit
fi

echo "Copying rtkstreamer.service to /etc/systemd/system"
sudo cp rtkstreamer.service /etc/systemd/system
sudo systemctl enable rtkstreamer.service