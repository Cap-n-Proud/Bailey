// sudo udevadm control --reload-rules
// to refresh the port allocation

var events = require('events');
var eventEmitter = new events.EventEmitter();
var fs = require('safefs');
var PathTelFile="/home/pi/Documents/Sketches/Bailey/server/log/"

var com = require("/usr/local/lib/node_modules/serialport");
var serPort = "/dev/ttyACM0";
var serBaud = 38400;
var serverPort = 54321;

var LogR = 0;
var TelemetryFN = "";
var prevTel="";
var prevPitch="";

var version = "0.1";

var ArduRead = new Object();
{
ArduRead['READ Read_LMO'] = 0;
ArduRead['READ Read_RMO'] = 0;
ArduRead['READ Read_anglePIDOutput']=0;
ArduRead['READ Read_APIDKp'] = 0;
ArduRead['READ Read_APIDKi'] = 0;
ArduRead['READ Read_APIDKd'] = 0;
ArduRead['READ Read_APIDAggKp']=0;
ArduRead['READ Read_APIDAggKi']=0;
ArduRead['READ Read_APIDAggKd']=0;
ArduRead['READ Read_SPIDKp']=0;
ArduRead['READ Read_SPIDKi']=0;
ArduRead['READ Read_SPIDKd']=0;
ArduRead['READ Read_Yaw']=0;
ArduRead['READ Read_Roll']=0;
ArduRead['READ Read_Pitch']=0;
ArduRead['READ Read_MotorsON']='0';
ArduRead['READ Read_LoopT']=0;
ArduRead['READ Read_SetsteerGain']=0;
ArduRead['READ Read_SetthrottleGain']=0;
ArduRead['READ Read_TriggerAngleAggressive']=0;
ArduRead['READ Read_Info']='Waiting for socket';
ArduRead['READ Read_ISTE']=0;
ArduRead['READ Read_BAL']=0;
ArduRead['READ FirmwareVersion']='N/A';
}

var serialPort = new com.SerialPort(serPort, {
  baudrate: serBaud,
  parser: com.parsers.readline('\n')
  });
  
serialPort.on('open',function() {
  console.log('Arduino connected on '+ serPort + ' @' + serBaud);
});




//--------------------------------------

var express = require('express');
var app = express();
var http = require('http').Server(app);
var io = require('socket.io')(http);

var sys = require('sys');
var exec = require('child_process').exec;

app.use(express.static(__dirname + '/public'));
// Routers
{
app.get('/', function(req, res){
  res.sendFile(__dirname + '/public/index.html');
  res.end;
});

app.get('/d3', function(req, res) {
  res.sendFile(__dirname + '/public/D3.html');
  res.end;
});

app.get('/d3t', function(req, res) {
  res.sendFile(__dirname + '/public/d3test.html');
  res.end;
});

app.get('/lv2', function(req, res) {
  res.sendFile(__dirname + '/public/livedata2.htm');
  res.end;
});
app.get('/ypr', function(req, res) {
  res.sendFile(__dirname + '/public/ypr2.htm');
  res.end;
});

app.get('/test', function(req, res) {
  res.sendFile(__dirname + '/public/test.htm');
  res.end;
});

app.get('/livedata', function(req, res) {
  res.sendFile(__dirname + '/public/livedata.htm');
  res.end;
});

app.get('/found', function(req, res) {
  res.sendFile(__dirname + '/public/index.html');
  res.end;
});

app.get('/tb', function(req, res) {
  res.sendFile(__dirname + '/public/tempbatt.htm');
  res.end;
});

app.get('/vj', function(req, res) {
  res.sendFile(__dirname + '/public/robotj.html');
  res.end;
});

//Virtul joystick for mobile
app.get('/vjm', function(req, res) {
  res.sendFile(__dirname + '/public/robotjm.html');
  res.end;
});

app.get('/REBOOT', function(req, res) {
  var postData = req.url;  
  function puts(error, stdout, stderr) { sys.puts(stdout) }
  exec('sudo reboot now');
  sockets.emit('Info', "Rebooting")
  //console.log(postData);  
  res.end;
});


}

// Logging middleware


var logF = function(){  
 
     if (LogR == 1 && prevPitch != ArduRead['READ Read_Pitch'])
     {
      prevPitch = ArduRead['READ Read_Pitch']
      LogRow = new Date().getTime() + ',';
		
        for(var prop in ArduRead)
	{
	  if (ArduRead[prop] != undefined)
	  { 
	    if(ArduRead.hasOwnProperty(prop)){
		LogRow = LogRow + ArduRead[prop].toString().trim() + ','
	    }
  	  }
  	}
  fs.appendFile(PathTelFile+TelemetryFN, LogRow, function (err) {
		  if (err) {
		  console.log('ERROR: ' + err);
		  console.log(LogRow + '\n' )
		  LogR=0;
		  }
		});
  fs.appendFile(PathTelFile+TelemetryFN, '\r\n');
    }
  };
//Triggered when new data cames from the serial port
eventEmitter.on('log', logF);
 
io.on('connection', function(socket){
  socket.emit('connected', version, ArduRead);  
  console.log('New socket.io connection - id: %s', socket.id);
  //Add here trigger for remote update event
  
  //var LogRow;  
  setInterval(function(){
   socket.emit('status', ArduRead['READ Read_Yaw'], ArduRead['READ Read_Pitch'], ArduRead['READ Read_Roll'], ArduRead['READ Read_BAL']);
  }, 300);
 
  //This needs to be changed to SCMD command
//  socket.on('move', function(dX, dY){
//	
//	//console.log('event: ', dX, dY);
//	serialPort.write('SCMD Steer ' + dX + '\n');
//	serialPort.write('SCMD Throttle ' + -dY + '\n');	
//	});
  
  socket.on('Video', function(Video){
   socket.emit('CMD', Video); 		
    function puts(error, stdout, stderr) { sys.puts(stdout) }
    exec('sudo bash /home/pi/Documents/Sketches/Bailey/server/scripts/' + Video, puts);
      });
  
  //Set commands goes to arduino directly
  socket.on('SCMD', function(CMD){
    console.log(CMD);
    serialPort.write('SCMD ' + CMD + '\n');
    //Commands are echoed back to the remote
    socket.emit('CMD', 'SCMD ' + CMD);    
      });
  
  //Server Commands
  socket.on('SerCMD', function(CMD){  
    //console.log(CMD);
    socket.emit('CMD', CMD);    
    if ( CMD == "LOG_ON" ) {
      console.log("Log started");
      if (TelemetryFN == "") {
	  var myDate = new Date();
	  TelemetryFN = 'Telemetry_' + myDate.getFullYear() + myDate.getMonth() + myDate.getDate() +  myDate.getHours() +  myDate.getMinutes() + myDate.getSeconds()+ '.csv';
	  
	  fs.appendFile(PathTelFile+TelemetryFN, 'time' + ',');
	  for(var prop in ArduRead) {
	    if(ArduRead.hasOwnProperty(prop)){
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
	console.log('Start logging to file ');
	console.log(PathTelFile+TelemetryFN);
	
        LogR = 1;
      }
    else if ( CMD == "LOG_OFF" ){
       console.log("Log Stopped");
	socket.emit('Info', "stop")
       
       
   LogR = 0;
    }
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
  
//Read input from Arduino and stores it into a dictionary
serialPort.on('data', function(data, socket) {
	//console.log(data);
	 	
	if (data.match(/ /g) && data.match(/READ/g))
	{
	  var tokenData = data.split(" ");
	  ArduRead[tokenData[0] + ' ' + tokenData[1]] = tokenData[2];                                      
	  eventEmitter.emit('log');
	  //eventEmitter.emit('BalancingIndex');
	  
	}
	if (data.indexOf('***') !== -1)
	{
	  console.log(data);
	  
	}

});
 
});