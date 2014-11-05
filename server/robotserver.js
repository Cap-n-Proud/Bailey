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
function greetings() {
  console.log("__        __   _                            _              ");
  console.log("\ \      / /__| | ___ ___  _ __ ___   ___  | |_ ___        ");
  console.log("\ \ /\ / / _ \ |/ __/ _ \| '_ ` _ \ / _ \ | __/ _ \       ");
  console.log("\ V  V /  __/ | (_| (_) | | | | | |  __/ | || (_) |      ");
  console.log("\_/\_/ \___|_|\___\___/|_| |_| |_|\___|  \__\___/       ");
  console.log("____        _ _                                         ");  
  console.log("| __ )  __ _(_) | ___ _   _   ___  ___ _ ____   _____ _ __ ");
  console.log(" |  _ \ / _` | | |/ _ \ | | | / __|/ _ \ '__\ \ / / _ \ '__|");
  console.log("| |_) | (_| | | |  __/ |_| | \__ \  __/ |   \ V /  __/ | ");  
  console.log(" |____/ \__,_|_|_|\___|\__, | |___/\___|_|    \_/ \___|_|");   
  console.log("                      |___/                               ");
}
var LogR = 0;
var TelemetryFN = "";
var prevTel="";
var prevPitch="";

var SEPARATOR = ","
var version = "0.2";

var ArduHeader;
var ArduRead = {};

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

app.get('/found', function(req, res){
  res.sendFile(__dirname + '/public/index.html');
  res.end;
});
app.get('/d3test', function(req, res) {
  res.sendFile(__dirname + '/public/d3test.html');
  res.end;
});


app.get('/livedata', function(req, res) {
  res.sendFile(__dirname + '/public/livedata.htm');
  res.end;
});
app.get('/livedatav2', function(req, res) {
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


var logF = function(data){  
 
     if (LogR == 1 && prevPitch != ArduRead['pitch'])
     {
      prevPitch = ArduRead['pitch']
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
  };

//Triggered when new data cames from the serial port
//eventEmitter.on('log', logF(data));
 
io.on('connection', function(socket){
  socket.emit('connected', version, ArduRead);  
  console.log('New socket.io connection - id: %s', socket.id);
  //Add here trigger for remote update event
  
  //var LogRow;  
  setInterval(function(){
   socket.emit('status', ArduRead['yaw'], ArduRead['pitch'], ArduRead['roll'], ArduRead['bal']);
  }, 250);
 
  socket.on('Video', function(Video){
   socket.emit('CMD', Video); 		
    function puts(error, stdout, stderr) { sys.puts(stdout) }
    exec('sudo bash /home/pi/Documents/Sketches/Bailey/server/scripts/' + Video, puts);
      });
  
  //Set commands goes to Arduino directly
  socket.on('SCMD', function(CMD){
    console.log(CMD);
    serialPort.write('SCMD ' + CMD + '\n');
    //Commands are echoed back to the remote
    socket.emit('CMD', 'SCMD ' + CMD);    
      });
  
    socket.on('move', function(dX, dY){
	//console.log('event: ', dX, dY);
	serialPort.write('SCMD Steer ' + dX + '\n');
	serialPort.write('SCMD Throttle ' + -dY + '\n');	
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
	socket.emit('Info', "stop");     
	LogR = 0;
    }
      });

  socket.on('REBOOT', function(){
    function puts(error, stdout, stderr) { sys.puts(stdout) }
    exec('sudo reboot now');
    sockets.emit('Info', "Rebooting")
  });

  socket.on('SHUTDOWN', function(){
    function puts(error, stdout, stderr) { sys.puts(stdout) }
    exec('sudo shutdown');
    sockets.emit('Info', "Bailey going down for maintenance now")
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
	//console.log(data);
	if (data.indexOf('T') !== -1)
	{
	  var tokenData = data.split(SEPARATOR);
	  var j = 0;
	  
	  for (var i in ArduRead) {
	    ArduRead[i] = tokenData[j];
	    j++;
	    //console.log(i + ' ' + ArduRead[i]);
	  }
	  j = 0;
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
	  ArduHeader = data.split(SEPARATOR);
	  var arrayLength = ArduHeader.length;
	  for (var i = 0; i < arrayLength; i++) {
	    ArduRead[ArduHeader[i]] = "N/A";
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