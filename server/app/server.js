// sudo udevadm control --reload-rules
// to refresh the port allocation

var nconf = require('/usr/local/lib/node_modules/nconf');
nconf.argv()
       .env()
       .file({ file: '/home/pi/Bailey/server/app/config.json' });

var events = require('events');
var eventEmitter = new events.EventEmitter();
var nodeLib = nconf.get('server:nodeLib');

var fs = require(nodeLib + 'safefs');
var PathTelFile=nconf.get('telemetry:PathTelFile');
var SEPARATOR = nconf.get('telemetry:SEPARATOR');
var installPath = nconf.get('server:installPath');
var com = require(nodeLib + 'serialport');
var express = require(nodeLib + 'express');
var app = express();
var http = require('http').Server(app);
var io = require(nodeLib + 'socket.io')(http);

var sys = require('sys');
var exec = require('child_process').exec;

var serPort = nconf.get('server:serPort');
var serBaud = nconf.get('server:serBaud');
var serverPort = nconf.get('server:serverPort');
var version = nconf.get('server:version');
var videoFeedPort = nconf.get('MJPG:MJPGPort');

// include custom functions ======================================================================
//var systemModules = require(installPath + 'app/systemModules');
var functions = require(installPath + 'server/app/functions');
var camera = require(installPath + 'server/app/camera');
//var robot = require(installPath + 'server/app/robot');

// load the routes
require('./routes')(app);

app.use(express.static(installPath + 'server/wwwroot'));
//Not nice, implement asciimo: https://github.com/Marak/asciimo
function greetings() {

 
}

var serverADDR = 'N/A';
var LogR = 0;
var TelemetryFN = "";
var prevTel="";
var prevPitch="";
var THReceived=0;

var TelemetryHeader, PIDHeader, ArduSysHeader;
var Telemetry ={}
var PID ={}
var ArduSys = {};

var serialPort = new com.SerialPort(serPort, {
  baudrate: serBaud,
  parser: com.parsers.readline('\n')
  });
  
serialPort.on('open',function() {
  console.log('Arduino connected on '+ serPort + ' @' + serBaud);
  
    
  
});


//Get IP address http://stackoverflow.com/questions/3653065/get-local-ip-address-in-node-js

var os = require('os');
var ifaces = os.networkInterfaces();

Object.keys(ifaces).forEach(function (ifname) {
  var alias = 0
    ;

  ifaces[ifname].forEach(function (iface) {
    if ('IPv4' !== iface.family || iface.internal !== false) {
      // skip over internal (i.e. 127.0.0.1) and non-ipv4 addresses
      return;
    }

    if (alias >= 1) {
      // this single interface has multiple ipv4 addresses
      console.log(ifname + ':' + alias, iface.address);
    } else {
      // this interface has only one ipv4 adress
      serverADDR = iface.address;
    }
  });
});


//---------------

io.on('connection', function(socket){
  //socket.emit('connected', version, Telemetry);  
   
    var myDate = new Date();
   
   var startMessage = 'Connected ' + myDate.getHours() + ':' + myDate.getMinutes() + ':' + myDate.getSeconds()+ ' v' + version + ' @' + serverADDR;
//Init the heades for telemtry data
   serialPort.write('READ RemoteInit\n\r');
    //Trasmit system and PID parameters
   
    socket.emit('serverADDR', serverADDR);
    socket.emit('connected', startMessage, serverADDR, serverPort, videoFeedPort, PID);
    console.log('New socket.io connection - id: %s', socket.id);

    
    
  setInterval(function(){
  if(THReceived==1)socket.emit('status', Telemetry['yaw'], Telemetry['pitch'], Telemetry['roll'], Telemetry['bal']);
  //console.log(Telemetry['yaw'] + Telemetry['Event']);
  }, 250);
//serialPort.write('READ PIDInit\n\r');
     
  setInterval(function(){

  var usage = "N/A";
  var temperature = fs.readFileSync("/sys/class/thermal/thermal_zone0/temp");
temperature = ((temperature/1000).toPrecision(3)) + "°C";

  socket.emit("CPUInfo", temperature, usage);
  }, 3 * 1000);
 
  socket.on('Video', function(Video){
   socket.emit('CMD', Video);
    function puts(error, stdout, stderr) { sys.puts(stdout) }
     exec('sudo bash ' + installPath + 'server/app/bin/' + Video, puts);
        
    });

  //Set commands goes to Arduino directly
  socket.on('SCMD', function(CMD){
    //console.log(CMD);
    serialPort.write('SCMD ' + CMD + '\n');
    //Commands are echoed back to the remote
    socket.emit('CMD', 'SCMD ' + CMD);    
      });
  
    socket.on('move', function(dX, dY){
	//console.log('event: ', dX, dY);
	serialPort.write('SCMD Steer ' + Math.round(dX) + '\n');
	serialPort.write('SCMD Throttle ' + Math.round(dY) + '\n');	
	});
    
  //Server Commands
  socket.on('SerCMD', function(CMD){  
    socket.emit('CMD', CMD);    
    if ( CMD == "LOG_ON" ) {
      console.log("Log started");
      if (TelemetryFN == "") {
	  var myDate = new Date();
	  TelemetryFN = 'Telemetry_' + myDate.getFullYear() + myDate.getMonth() + myDate.getDate() +  myDate.getHours() +  myDate.getMinutes() + myDate.getSeconds()+ '.csv';
	  
	  fs.appendFile(PathTelFile+TelemetryFN, 'time' + ',');
	  //write headers
	  for(var prop in Telemetry) {
	    if(Telemetry.hasOwnProperty(prop)){
		//console.log(prop);
		fs.appendFile(PathTelFile+TelemetryFN, prop + ',', function (err) {
		  if (err) throw err;          
		});
	    }
	  }
	  fs.appendFile(PathTelFile+TelemetryFN, '\n');
	  //fs.close(PathTelFile+TelemetryFN);
	}
	socket.emit('Info', PathTelFile+TelemetryFN)
	//console.log('Start logging to file ');
	//console.log(PathTelFile+TelemetryFN);
	
        LogR = 1;
      }
    else if ( CMD == "LOG_OFF" ){
	//console.log("Log Stopped");
	socket.emit('Info', "logging stoped");     
	LogR = 0;
    }
      });

  socket.on('REBOOT', function(){
    function puts(error, stdout, stderr) { sys.puts(stdout) }
    exec('sudo reboot now');
    sockets.emit('Info', "Rebooting")
  });

  socket.on('SHUTDOWN', function(){
    socket.emit('Info', "Bailey going down for maintenance now");
    function puts(error, stdout, stderr) { sys.puts(stdout) }
    exec('sudo shutdown');    
  });
  
  socket.on('disconnect', function(){
    console.log('Disconnected id: %s', socket.id);

  });  
 
 
});
io.on('disconnect', function () {
        console.log('A socket with sessionID ' + hs.sessionID 
            + ' disconnected!');

    });

http.listen(serverPort, function(){
console.log('listening on *: ', serverPort);//
greetings();  
  
//Read input from Arduino and stores it into a dictionary
serialPort.on('data', function(data, socket) {	 	
	//var TelemetryHeader, PIDHeader, ArduSysHeader;
        //var Telemetry, PID, ArduSys = {};
        //console.log(data);
	//"T" means we are receiving Telemetry data
        //console.log(data);	
        if (data.indexOf('T') !== -1)
	{
	  var tokenData = data.split(SEPARATOR);
	  var j = 0;
	  
	  for (var i in Telemetry) {
	    Telemetry[i] = tokenData[j];
	    j++;
	    //console.log(i + ' ' + Telemetry[i]);
	  }
	  j = 0;
	  eventEmitter.emit('log', data);
	}
	
	//"TH" means we are receiving Telemetry Headers
        if (data.indexOf('TH') !== -1)
	{
	   TelemetryHeader = data.split(SEPARATOR);
	  var arrayLength = TelemetryHeader.length;
	  for (var i = 0; i < arrayLength; i++) {
	    Telemetry[TelemetryHeader[i]] = "N/A";
	    //console.log(TelemetryHeader[i]);
          }
        
          THReceived=1;
	  eventEmitter.emit('log', data);
	}
	
	if (data.indexOf('SYSH') !== -1)
	{
	   ArduSysHeader = data.split(SEPARATOR);
	  var arrayLength = ArduSysHeader.length;
	  for (var i = 0; i < arrayLength; i++) {
	    ArduSys[ArduSysHeader[i]] = "N/A";
	    //console.log(TelemetryHeader[i]);
	}
            setTimeout(function () {
         	   serialPort.write('READ SYSParamTX\n\r');
        
            }, 100)
	 
        }
	
	if (data.indexOf('SYS') !== -1)
	{
	  var tokenData = data.split(SEPARATOR);
	  var j = 0;
	  
	  for (var i in Telemetry) {
	    ArduSys[i] = tokenData[j];
	    j++;
	    //console.log(i + ' ' + Telemetry[i]);
	  }
	  j = 0;
	  eventEmitter.emit('log', data);
	}
	
        if (data.indexOf('PID') !== -1)
	{
	  var tokenData = data.split(SEPARATOR);
	  var j = 0;
	 //  console.log('------------- PID settings -------------');
	 
	  for (var i in PID) {
	    PID[i] = tokenData[j];
	   
            //console.log(i + ' ' + PID[i]);
	 
            j++;
	     }
	  j = 0;
	  eventEmitter.emit('log', data);
	}
	
        if (data.indexOf('PIDH') !== -1)
	{  // console.log(data);
            //console.log('------------- PID Header -------------');
	
	   PIDHeader = data.split(SEPARATOR);
	  var arrayLength = PIDHeader.length;
	  for (var i = 0; i < arrayLength; i++) {
	    PID[PIDHeader[i]] = "N/A";
	    //console.log(PIDHeader[i]);
	  }
	     setTimeout(function () {
                serialPort.write('READ PIDParamTX\n\r');
                 
            }, 100)
	  
	     
          eventEmitter.emit('log', data);
          
         
	}
	

	
	
	//If the first word is '***' prints in the server console. Used to debug the config from Arduino
	if (data.indexOf('***') !== -1)
	{
	  console.log(data);	  
	}
	
	//Get the header for the object that stores telemetry data
	if (data.indexOf('HEADER') !== -1)
	{
	  TelemetryHeader = data.split(SEPARATOR);
	  var arrayLength = TelemetryHeader.length;
	  for (var i = 0; i < arrayLength; i++) {
	    Telemetry[TelemetryHeader[i]] = "N/A";
	    //console.log(TelemetryHeader[i]);
	  }
	}
	
	if (LogR == 1){
	  LogRow = new Date().getTime() + SEPARATOR;
      LogRow = LogRow + data;
  fs.appendFile(PathTelFile+TelemetryFN, LogRow, function (err) {
		  if (err) {
		  console.log('ERROR: ' + err);
		  console.log(LogRow + '\n' )
		  LogR=0;
		  }
		});
  fs.appendFile(PathTelFile+TelemetryFN, '\r\n');
	}
});
 
});