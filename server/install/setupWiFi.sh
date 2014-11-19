#!/bin/bash

#sudo nano /etc/network/interfaces
#sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
echo -e "***** Setting up WiFi *****"
sudo cp interfaces /etc/network/interfaces
sudo chmod 0777 /etc/network/interfaces

sudo cp wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf
sudo chmod 0777 /etc/wpa_supplicant/wpa_supplicant.conf
