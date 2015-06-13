#!/bin/bash

#sudo bash /home/pi/Documents/Sketches/Bailey/server/install/robotinit.sh
#If errors in updating libraries try:
#sudo rm /var/lib/dpkg/status
#sudo touch /var/lib/dpkg/status
asd() {
cat <<"EOT"
                     ____
                   ,'_   |
 __________________|__|__|__
<_____                      )                _.------._
      `-----------,------.-'              ,-'          `-.
                 |    |  |              ,'                `.
                ,'    |  |            ,'                    `.
                |  _,-'  |__         /                        \
              _,'-'    `.   `---.___|_____________             \
          .--'  -----.  | _____________________   `-. -----     |
          |    ___|  |  |                      \  ,- \          |
          |    ___|  |===========================((|) |         |
          |       |  |  | _____________________/  `- /          |
          `--._ -----'  |        _________________,-' -----     |
               `.-._   ,' __.---'   |                          /
                |   `-.  |           \                        /
                `.    |  |            `.                    ,'
                 |    |  |              `.                ,'
 _____,----------`-------`-.              `-._        _,-'
<___________________________)                 `------'
                   | _|  |
                   `.____|

        +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+-+
        |B|a|i|l|e|y| |i|n|s|t|a|l|l|e|r|
        +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+-+


EOT
}

asd
    echo -e "***** Setting up robotserver *****"
    cd /home/pi/Bailey
    sudo cp /home/pi/Bailey/install/robotserver /etc/init.d/robotserver
    sudo chmod 0755 /etc/init.d/robotserver
    sudo update-rc.d robotserver defaults
exit 0 
