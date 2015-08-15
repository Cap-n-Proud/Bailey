// sudo udevadm control --reload-rules
// to refresh the port allocation
// cd /home/pi/Bailey/server/app

// include custom functions ======================================================================
var ssh = require('/home/pi/sftptest/ssh');
var source = "/home/pi/Documents/log/Baileylog.log";
var target = "/share/Public/Baileylog.log";

//ssh.uploadFile(source,target);




source = "/home/pi/Documents/telemetry/Telemetry_15-08-05_16-00-15.csv";
target = "/share/Public/Telemetry_15-08-05_16-00-15.csv";
ssh.uploadFile(source,target);