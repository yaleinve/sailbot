#!/bin/bash

#switchNetworks.sh      Eric Anderson
#Switches the wpa config file to prioritize yale or ratchet router


echo "Welcome to switch networks.  This command must be run as super user"
echo "Would you like to switch to Ratchet Router (R) or Yale Wireless (Y)?"

read NET

if [ "$NET" == "Y" ]  ; then
    echo "You have selected to switch to Yale Wireless"
elif [ "$NET" == "R" ] ; then
    echo "You have selected to switch to Ratchet Router"
else
    echo "Invalid input, exiting: " 
    exit
fi

echo "Proceed? Please note that WIRELESS CONNECTIONS will be disconnected during the network switch!!!"
echo "[Y/n]: "

read CONFIRM

if [ "$CONFIRM" == "Y" ] ; then
    echo "Command confirmed, reconnecting now.  Expect to be disconnected..."
    if [ "$NET" == "Y" ] ; then
        sudo cp /etc/wpa_supplicant/yale_wireless_priority /etc/wpa_supplicant/wpa_supplicant.conf
    else
        sudo cp /etc/wpa_supplicant/ratchet_router_priority /etc/wpa_supplicant/wpa_supplicant.conf
    fi
#    echo "dummy command for now"
    sudo ifdown wlan0 && sudo ifup wlan0   #This is the command that matters!!!
else
    echo "Command not confirmed, exiting"
    exit
fi
