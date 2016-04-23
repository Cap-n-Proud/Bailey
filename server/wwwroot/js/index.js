 var serverADDR = "N/A";
    var serverPort = "N/A";
    var MJPGPort = "N/A";
    
    var socket = io();
    //Slide ranges are automatically calculated as percentage of the default config from Arduino
    var PID_SLIDE_RANGE = 1.50; 

    window.onload=function()
    {
    var classname = document.getElementsByClassName("PIDControlSlider");

    var myFunction = function() {
        var attribute = this.getAttribute("id");
        socket.emit('SCMD', this.getAttribute("id")+' '+ this.value);
        //alert(this.getAttribute("id")+' '+ this.value);
    };

    for(var i=0;i<classname.length;i++){
        classname[i].addEventListener('change', myFunction, false);
    }

  
    
    }

   {
    var classname = document.getElementsByClassName("AngleControlSlider");

    var myFunction = function() {
        var attribute = this.getAttribute("id");
        socket.emit('SCMD', this.getAttribute("id")+' '+ this.value);
        //alert(this.getAttribute("id")+' '+ this.value);
    };

    for(var i=0;i<classname.length;i++){
        classname[i].addEventListener('change', myFunction, false);
    }
    }
    
	
    function MotorF(element){

    if (element.checked == true) {
    socket.emit('SCMD', "Motors 1");    
    }
    else
    {
    socket.emit('SCMD', "Motors 0");
    }
    }

    function AutoTune(element){

    if (element.checked == true) {
    socket.emit('SCMD', "AUTOTUNE 1");    
    }
    else
    {
    socket.emit('SCMD', "AUTOTUNE 0");
    }
    }

    
    
		

    function LogF(element){
    if (element.checked == true) {
    socket.emit('SerCMD', "LOG_ON");
    }
    else
    {
    socket.emit('SerCMD', "LOG_OFF");   
    }
    }

    function ThrottleSet(val){
    socket.emit('SCMD', 'Throttle ' + val);
    document.getElementById("ThrottleVal").value = val;

    }

    function SteerSet(val){
    socket.emit('SCMD', 'Steer ' + val);
    document.getElementById("SteerVal").value = val;

    }

    function prntCfg(){
    socket.emit('SCMD', "printConfig");

    }

    function shutdown(){
    socket.emit('SHUTDOWN', "now"); 
    }

    function sendCMD() {
     //document.getElementById("CMD").innerHTML = document.getElementById("right-label").value;
      socket.emit('SCMD', document.getElementById("SCMDCommand").value);    
    }

    socket.on('Info', function(CMD){
    document.getElementById("Info").innerHTML = CMD.replace("\n", "<br />");    
    });

    socket.on('CMD', function(CMD){
    document.getElementById("CMD").innerHTML = CMD;    
    });

    socket.on('status', function (y, p, r, b, dISTE) {
        document.getElementById("BAL").innerHTML = r;
        document.getElementById("Pitch").innerHTML = p;
        document.getElementById("Compass").innerHTML = y;
    
    }); 

    socket.on('CPUInfo', function (temp, usage) {
    document.getElementById("CPUInfo").innerHTML = temp;

    });

    socket.on('cpuUsageUpdate', function (CPU) {
    document.getElementById("CPU").innerHTML = CPU; 
    });

    //socket.on('connected', function(V, PID){
    //   started=0;
    //   document.getElementById("Info").innerHTML = 'Server: ' + V + ' Firmware: ' + PID['FirmwareVersion'];
      //Set defalut values for the PID sliders
 
    //     });

    
    function setSliders(PID){
         var PIDSlider = document.getElementsByClassName("PIDControlSlider");
      var arrayLength = PIDSlider.length;
              
      for (var i = 0; i < arrayLength; i++) {
      //console.log(PIDSlider[i].id + ' ' + parseInt(PID[PIDSlider[i].id]));
          document.getElementById(PIDSlider[i].id).value = parseInt(PID[PIDSlider[i].id]);
          document.getElementById(PIDSlider[i].id).max = (1 + PID_SLIDE_RANGE)*parseInt(PID[PIDSlider[i].id]);
          document.getElementById(PIDSlider[i].id).min = (1 - PID_SLIDE_RANGE)*parseInt(PID[PIDSlider[i].id]);
          document.getElementById(PIDSlider[i].id + 'Val').value = parseInt(PID[PIDSlider[i].id]);
      }
      
        //console.log(' ' + parseInt(PID['calibratedZeroAngle']));
          document.getElementById('calibratedZeroAngle').value = parseInt(PID['calibratedZeroAngle']);
    document.getElementById('calibratedZeroAngle').max = (1 + PID_SLIDE_RANGE)*parseInt(PID['calibratedZeroAngle']);
    document.getElementById('calibratedZeroAngle').min = (1 - PID_SLIDE_RANGE)*parseInt(PID['calibratedZeroAngle']);
    document.getElementById('calibratedZeroAngleVal').value = parseInt(PID['calibratedZeroAngle']);
    
    }

   
    socket.on('connected', function(info, serverADDR, sPort, MJPEGPort, PIDH, PID){
    started=0;

    serverADDR = serverADDR.toString();
    serverPort = sPort.toString(); 
    //MJPEGPort = MJPEGPort.toString();
    
    document.getElementById("botControl").src = 'http://' + serverADDR + ':' + serverPort + '/vj';
    document.getElementById('mainContent').src = 'http://' + serverADDR + ':' + serverPort + '/video';
    //document.getElementById("telemetryGPH").src = '';
    ////console.log('http://' + serverADDR + ':' + MJPEGPort);
    document.getElementById("Info").innerHTML = info;
   //document.getElementById("CMD").innerHTML = 'http://' + serverADDR + ':' + serverPort + '/vj';
    //setSliders(PID);
   // document.getElementById("Info").innerHTML = server;
   //createSliders(PIDH, PID);
	
    
    });


//    function createSliders(PIDH, PID) {
//      for(i = 1; i < PIDH.length; i++) {
//  
//      createVSlider(PIDH[i],0.8* PID[PIDH[i]], 1.2*PID[PIDH[i]],PID[PIDH[i]] )
//   
//    } 
//         
// }
//
// 
////Main ideas for rotation from:
////http://www.benknowscode.com/2014/01/css-rotated-text-parent-dimensions-and-spacing-issues.html
//function createVSlider(id, min, max, val) {
//    var sliderContainer = document.createElement('span');
//	sliderContainer.className="vSliderWraper";
//	var slider = document.createElement('input');
//    slider.id = id;
//    slider.type = 'range';
//    slider.min = min;
//    slider.max = max;
//    slider.value = val;
//    slider.step = 1;
//    slider.className="vSlider";
//	sliderContainer.appendChild(slider);
//    document.body.appendChild(sliderContainer);
//    //getElementById(PIDSliders).appendChild(sliderContainer);
//    
//}

   function VideoF(element) {
	var status = 0;
      if (element.checked == true) {
	    socket.emit('Video', "cameraON.sh");
	    status = 1;
	    
	 
	    
      } else {
	    socket.emit('Video', "cameraOFF.sh");
	    status = 0;
	    }
	
	if (status == 1){
	  setTimeout(camFeedRefresh, 1000);
	  }
	  else
	  {
	  setTimeout(camOFF, 1000);
    
	  }
    }
	
	function camFeedRefresh(){
	   //  document.getElementById('videoFeed').src = document.getElementById('videoFeed').src = document.getElementById("videoFeed").src = 'http://' + serverADDR + ':' + serverPort + '/video';

		
	}
	
	function camOFF(){
	   document.getElementById('videoFeed').src = "default.html";
	 
	}
	
    function changeMainContent(place){
	   document.getElementById('mainContent').src = place;
	 
	}
  
    function resizeIframe(obj) {
    obj.style.height = obj.contentWindow.document.body.scrollHeight + 'px';
  }
  
	function takeSnapshot(element) {
	  socket.emit('takeSnapShot');	
	}
	
function toggleTelemetryGPH(element){

    if (element.checked == true) {
      document.getElementById('telemetryGPH').src = document.getElementById('telemetryGPH').src = document.getElementById("telemetryGPH").src = 'http://' + serverADDR + ':' + serverPort + '/livedata.html';
      document.getElementById("Info").innerHTML = 'http://' + serverADDR + ':' + serverPort + '/livedata.html';

    }
    else
    {
      document.getElementById("telemetryGPH").src = '';//'default.html';
  
    }
    }
