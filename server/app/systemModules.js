

/*
var mkdirSync = function(path) {
    try {
        fs.mkdirSync(path);
    } catch (e) {
        if (e.code != 'EEXIST') throw e;
    }
}

*/

//Get IP address http://stackoverflow.com/questions/3653065/get-local-ip-address-in-node-js
//Check also this http://stackoverflow.com/questions/24378272/node-js-how-to-get-the-ip-address-of-http-server-listening-on-a-specific-port
 function timeStamp(){
      
     var MyDate = new Date();
        var MyDateString;
        var MyTimeStamp;
        MyDateString = ('0' + MyDate.getFullYear()).slice(-2) + '-' + ('0' + (MyDate.getMonth() + 1)).slice(-2) + '-' + ('0' + (MyDate.getUTCDate())).slice(-2);
        MyTimeStamp = ('0' + MyDate.getHours()).slice(-2) + '-' + ('0' + (MyDate.getMinutes())).slice(-2) + '-' + ('0' + (MyDate.getSeconds())).slice(-2);

      return MyDateString + '_' + MyTimeStamp      
    }
    
    
function getServerIP(){
var os = require('os');
var ifaces = os.networkInterfaces();

Object.keys(ifaces).forEach(function (ifname) {
  var alias = 0
    ;

  ifaces[ifname].forEach(function (iface) {
    if ('IPv4' !== iface.family || iface.internal !== false) {
      // skip over internal (i.e. 127.0.0.1) and non-ipv4 addresses
      return iface.address;
    }

    if (alias >= 1) {
      // this single interface has multiple ipv4 addresses
      console.log(ifname + ':' + alias, iface.address);
    } else {
      // this interface has only one ipv4 adress
      return iface.address;
    }
  });
});

}

   // exports ======================================================================
  exports.timeStamp = timeStamp;
  exports.getServerIP = getServerIP;