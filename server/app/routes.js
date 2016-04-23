
// Routers
// expose the routes to our app with module.exports

var nconf = require('nconf');
nconf.argv()
       .env()
       .file({ file: __dirname + '/config.json' });

var installPath = nconf.get('server:installPath');


module.exports = function(app) {

var path = require('path'); 

// app.use(express.static(path.resolve(__dirname + '/../wwwroot')));
    // Routers
  

//
//app.use(express.static('/home/pi/Bailey/server/wwwroot/'));
// Routers
{
app.get('/', function(req, res){
  res.sendFile(path.resolve(__dirname + '/../wwwroot/index.html'));
  res.end;
});


app.get('/d3test', function(req, res) {
  res.sendFile(path.resolve(__dirname + '/../wwwroot/d3test.html'));
  res.end;
});

app.get('/showConfig', function(req, res) {
  res.sendFile(path.resolve(__dirname + '/../wwwroot/showConfig.html'));
  res.end;
});

app.get('/d3', function(req, res) {
  res.sendFile(path.resolve(__dirname + '/../wwwroot/D3.html'));
  res.end;
});

app.get('/video', function(req, res) {
  res.sendFile(path.resolve(__dirname + '/../wwwroot/video.html'));
  res.end;
});

app.get('/livedata', function(req, res) {
  res.sendFile(path.resolve(__dirname + '/../wwwroot/livedata.html'));
  res.end;
});

app.get('/serialMonitor', function(req, res) {
  res.sendFile(path.resolve(__dirname + '/../wwwroot/serialMonitor.html'));
  res.end;
});

app.get('/test', function(req, res) {
  res.sendFile(path.resolve(__dirname + '/../wwwroot/test.html'));
  res.end;
});

app.get('/test2', function(req, res) {
  res.sendFile(path.resolve(__dirname + '/../wwwroot/test2.html'));
  res.end;
});

app.get('/vj', function(req, res) {
  res.sendFile(path.resolve(__dirname + '/../wwwroot/robotj.html'));
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

    }
