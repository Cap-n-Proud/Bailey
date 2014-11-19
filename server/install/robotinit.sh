#!/bin/bash

#sudo bash /home/pi/Documents/Sketches/Bailey/server/install/robotinit.sh

asd() {
cat <<"EOT"



 		         _.---""""""""""--.._
 		      .="                    "=,
 		   .-'                          ``-.
 		  :                                 :
                 :  RRVIttIti+==iiii++iii++=;:,       :
                 : IBMMMMWWWWMMMMMBXXVVYYIi=;:,        :
                 : tBBMMMWWWMMMMMMBXXXVYIti;;;:,,      :
                 t YXIXBMMWMMBMBBRXVIi+==;::;::::       ,
                ;t IVYt+=+iIIVMBYi=:,,,=i+=;:::::,      ;;
                YX=YVIt+=,,:=VWBt;::::=,,:::;;;:;:     ;;;
                VMiXRttItIVRBBWRi:.tXXVVYItiIi==;:   ;;;;
                =XIBWMMMBBBMRMBXi;,tXXRRXXXVYYt+;;: ;;;;;
                 =iBWWMMBBMBBWBY;;;,YXRRRRXXVIi;;;:;,;;;=
                  iXMMMMMWWBMWMY+;=+IXRRXXVYIi;:;;:,,;;=
                  iBRBBMMMMYYXV+:,:;+XRXXVIt+;;:;++::;;;
                  =MRRRBMMBBYtt;::::;+VXVIi=;;;:;=+;;;;=
                   XBRBBBBBMMBRRVItttYYYYt=;;;;;;==:;=
                    VRRRRRBRRRRXRVYYIttiti=::;:::=;=
                     YRRRRXXVIIYIiitt+++ii=:;:::;==
                     +XRRXIIIIYVVI;i+=;=tt=;::::;:;
                      tRRXXVYti++==;;;=iYt;:::::,;;
                       IXRRXVVVVYYItiitIIi=:::;,::;
                        tVXRRRBBRXVYYYIti;::::,::::
                         YVYVYYYYYItti+=:,,,,,:::::;
                         YRVI+==;;;;;:,,,,,,,:::::::
 
      L I V E    L O N G    A N D     P R O S P E R


        +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+-+
        |B|a|i|l|e|y| |i|n|s|t|a|l|l|e|r|
        +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+-+


EOT
}

asd

sudo apt-get -y autoremove midori

sudo apt-get -y autoremove lxde-icon-theme
sudo apt-get -y autoremove omxplayer

sudo rm -rv /usr/share/icons/*
sudo rm -rv /opt/vc/src/*


echo -e "***** Replace syslog *****"
#Replace rsyslogd with inetutils-syslogd and remove useless logs
#Reduce memory and cpu usage. We just need a simple vanilla syslogd. Also there is no need to log so many files. Just dump them into /var/log/(cron/mail/messages)
sudo apt-get -y remove --purge rsyslog
sudo apt-get -y install inetutils-syslogd
sudo service inetutils-syslogd stop

for file in /var/log/*.log /var/log/mail.* /var/log/debug /var/log/syslog; do [ -f "$file" ] && rm -f "$file"; done
for dir in fsck news; do [ -d "/var/log/$dir" ] && rm -rf "/var/log/$dir"; done
sudo mkdir -p /etc/logrotate.d
echo -e "/var/log/cron\n/var/log/mail\n/var/log/messages {\n\trotate 4\n\tweekly\n\tmissingok\n\tnotifempty\n\tcompress\n\tsharedscripts\n\tpostrotate\n\t/etc/init.d/inetutils-syslogd reload >/dev/null\n\tendscript\n}" > /etc/logrotate.d/inetutils-syslogd
sudo service inetutils-syslogd start


sudo apt-get autoremove


echo -e "***** Updating system packages *****"
sudo apt-get update
sudo apt-get upgrade

#echo -e "***** Updating firmware *****"
#sudo rpi-update

echo -e "***** Enabling Turbo *****"
#700Mhz-1000Mhz dynamic: Scales the cpu frequency according to the load
echo -e "force_turbo=0" >> /boot/config.txt

echo -e "***** Installing mjpg-streamer *****"
#https://miguelmota.com/blog/raspberry-pi-camera-board-video-streaming/

# Install dev version of libjpeg
sudo apt-get install libjpeg62-dev

# Install cmake
sudo apt-get install cmake

# Download mjpg-streamer with raspicam plugin
git clone https://github.com/jacksonliam/mjpg-streamer.git ~/mjpg-streamer

# Change directory
cd ~/mjpg-streamer/mjpg-streamer-experimental

# Compile
make clean all

# Replace old mjpg-streamer
sudo rm -rf /opt/mjpg-streamer
sudo mv ~/mjpg-streamer/mjpg-streamer-experimental /opt/mjpg-streamer
sudo rm -rf ~/mjpg-streamer

# Begin streaming
#LD_LIBRARY_PATH=/opt/mjpg-streamer/ /opt/mjpg-streamer/mjpg_streamer -i "input_raspicam.so -fps 15 -q 50 -x 640 -y 480" -o "output_http.so -p 9000 -w /opt/mjpg-streamer/www" &

#This is supposed to enable SSH, untested...
#for i in 2 3 4 5; do sudo ln -s /etc/init.d/ssh etc/rc$i.d/S02ssh; done

echo -e "***** Installing node *****"
wget http://node-arm.herokuapp.com/node_latest_armhf.deb 
sudo dpkg -i node_latest_armhf.deb

echo -e "Test"
node -v

echo -e "***** Installing socket.io *****"

sudo npm install -g socket.io
#/usr/local/lib/node_modules/socket.io

echo -e "***** Installing express *****"
sudo npm install -g express
sudo npm install -g express-generator
sudo npm install -g safefs
sudo npm install -g serialport

echo -e "***** Setting up robotserver *****"
sudo cp robotserver /etc/init.d/robotserver
sudo chmod 0755 /etc/init.d/robotserver
sudo update-rc.d robotserver defaults

exit 0
