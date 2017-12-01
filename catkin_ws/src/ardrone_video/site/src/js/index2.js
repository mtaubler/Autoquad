
//creates array of objects with x and y coordinates
var dirVector = [];
var mission = [];
var pt="noisy\n";


var computeDistance = function(point1, point2) {
  var distBetweenPoints;
  var position1 = point1;
  var position2 = point2;

  distBetweenPoints = ((position2.x - position1.x)*(position2.x - position1.x) + (position2.y - position1.y)*(position2.y - position1.y)); 
  return distBetweenPoints
}

var breaksToVectors = function() {
  var canvas = document.getElementById('imageView');

  var smallestDistance = canvas.height/20;
  var distance = 0;
  var lastPoint = dirVector[0];

  for(var i = 0; i < dirVector.length-1; i++){
    var d = computeDistance(dirVector[i],  dirVector[i+1]);
    distance += d;

    if(distance > (smallestDistance * smallestDistance)) {
      var xPos = (dirVector[i+1].x - lastPoint.x)/20;
      var yPos = (dirVector[i+1].y - lastPoint.y)/20;
      var nextMission = { x: xPos, y: yPos };

      mission.push(nextMission);
      lastPoint = dirVector[i+1];

      distance=0;
    }
  }

  var sum = { x: 0, y: 0 };
  //console.log("x:" + sum.x + "  y: " + sum.y+"\n");
  for(var i=0; i<mission.length; i++) {
    sum.x += mission[i].x;
    sum.y += mission[i].y;
  //console.log("x:" + sum.x + "  y:" + sum.y +"\n");
  }

  return mission;
}

var clearCanvas = function(canvas,context) {   
  //does the erasing of the canvas
  context.clearRect(0, 0, canvas.width, canvas.height);

  //clears mission array
  mission=[];
};

var load = function () {

  //what selects the draw canvas and allows me to set percent hieght and width for it
  var canvas = document.getElementById('imageView');
  canvas.width = canvas.scrollWidth;
  canvas.height = canvas.scrollHeight;
  var context = canvas.getContext('2d');

  //how thick line is when made by mouse
  var radius = 5;

  //hope users arent holding down mouse as they enter
  var dragging = false; 

  //this is what connects two points: gives width to drawn line 
  context.lineWidth = radius * 2;




  //used for finding the displacement vector
  var x1, x2, y1, y2;
  var flag = false;

  //makes the center of the canvas (0,0)
  var cx = canvas.width*.5;
  var cy = canvas.height*(.5);

  //to do: scale pixels to meters --> idk how to do that yet
  //mousemoving event
  var putPoint = function(e){
    if(dragging) {

      //Draws line from old point to new point = new point = new mouse position
      context.lineTo(e.offsetX, e.offsetY);
      context.stroke();



      //Draws circular points - cur mouse pos, radius size, circumference of point in radians
      context.beginPath();
      context.arc(e.offsetX, e.offsetY, radius, 0, Math.PI * 2);
      context.fill();

      //Creates path after dot = old point = current mouse position
      context.beginPath();
      context.moveTo(e.offsetX, e.offsetY);

      //adds the onscreen mouse coordinates when mousedown on canvas
      //document.getElementById('mouseCoors').innerHTML = 'X: ' + (e.offsetX - cx) + ' Y: ' + (cy - e.offsetY);

      var point = {x:(e.offsetX - cx), y:(cy - e.offsetY)};

      dirVector.push(point);

    }
  }



  //mousedown event: starts a new path for drawing
  var engage = function(e) {
    dragging = true;
    putPoint(e);
  }

  //mouseup event: resets flags and ends current line path
  var disengage = function() {
    dragging = false;
    flag = false;
    //pointCount=0;
    //breaksToVectors();

    //starts from new line every mouseup 
    context.beginPath();
  }

  //mouseleave event: clears the canvas of any drawn lines
  var erase = function() {
    clearCanvas(canvas,context);
  }

  function toggleStuff() {
    //Do Something...maybe another function showAlert(), for instance

    if(this.innerHTML=="Noisy") {
      this.innerHTML = "Tum";
      pt ="tum\n";
    } else {
      //do something else
      this.innerHTML =  "Noisy"
      pt ="noisy\n";
    }
  }

  var flag=true;
  function record() {
    var background = document.getElementById('recBtn').style.backgroundColor;
    var textColor = document.getElementById('recBtn').style.color;
    if (flag) {
      flag = false;
      document.getElementById('recBtn').style.background = "red";
      document.getElementById('recBtn').style.color = "#black";
    } else {
      flag = true;
      document.getElementById('recBtn').style.background = "black";
      document.getElementById('recBtn').style.color = "#FFC904";
    }
  }

  var takePic = function() {
    console.log("oh snap");
  }

  canvas.addEventListener('mousedown',engage);
  canvas.addEventListener('mousemove',putPoint);
  canvas.addEventListener('mouseup',disengage);
  //canvas.addEventListener('mouseleave',erase);

  document.getElementById('picBtn').addEventListener('click', takePic);
  document.getElementById('recBtn').addEventListener('click', record);
  document.getElementById('toggleBtn').addEventListener('click', toggleStuff);
  document.getElementById('myBtn').addEventListener('click', publishCommands);
  document.getElementById('landBtn').addEventListener('click',land);
  document.getElementById('takeoffBtn').addEventListener('click',takeoff);
  document.getElementById("clrBtn").addEventListener("click", erase);
}

window.addEventListener('load', load)




var transfer;
// Connecting to ROS - logs successes and errors for websocket
// -----------------

var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

// Mission topic creation
var pathMission = new ROSLIB.Topic({
  ros : ros,
  name : 'ardrone_video/mission',
  messageType: 'std_msgs/String'
});

// Add the strings of the array together for the drone
function publishCommands(){
  var string = pt;
  var array = breaksToVectors();
  for(var i = 0; i < array.length; i++) {
    string += array[i].x + " " + array[i].y + "\n"; 
  }
  var miss = new ROSLIB.Message({
    data : string
  }); 
  pathMission.publish(miss); 
};

function land(){
  var landDrone = new ROSLIB.Topic({
    ros : ros,
    name : 'ardrone/land',
    messageType : 'std_msgs/Empty'
  });
  landDrone.publish();
};

function takeoff(){
  var takeoffDrone = new ROSLIB.Topic({
    ros : ros,
    name : 'ardrone/takeoff',
    messageType : 'std_msgs/Empty'
  });
  takeoffDrone.publish(); 
};


var keyFlag = false; 
// on key down for manual control 
onkeypress = function(e){
  var key = e.keyCode ? e.keyCode : e.which;
  if(key == 37)
  {
    var keyMiss = new ROSLIB.Message({
      data : "manual" + "\n" + "west"
    });
    pathMission.publish(keyMiss);
  }
  else if(key == 38){
    var keyMiss = new ROSLIB.Message({
      data : "manual" + "\n" + "north"
    });
    pathMission.publish(keyMiss);
  }
  else if(key == 39){
    var keyMiss = new ROSLIB.Message({
      data : "manual" + "\n" + "east"
    });
    pathMission.publish(keyMiss);
  }
  else if(key == 40){
    var keyMiss = new ROSLIB.Message({
      data : "manual" + "\n" + "south"
    });
    pathMission.publish(keyMiss);
  }
};


// 
onkeyup = function(e){
  var key = e.keyCode ? e.keyCode : e.which;
  if(key == 37){
    var keyMiss = new ROSLIB.Message({
      data : "manual" + "\n" + "stop"
    });
    pathMission.publish(keyMiss);
  }
  else if(key == 38){
    var keyMiss = new ROSLIB.Message({
      data : "manual" + "\n" + "stop"
    });
    pathMission.publish(keyMiss);
  }
  else if(key == 39){
    var keyMiss = new ROSLIB.Message({
      data : "manual" + "\n" + "stop"
    });
    pathMission.publish(keyMiss);
  }
  else if(key == 40){
    var keyMiss = new ROSLIB.Message({
      data : "manual" + "\n" + "stop"
    });
    pathMission.publish(keyMiss);
  }
}


// Subscribing to a Topic
// ----------------------

// Subscriber to retrieve drone's navdata
var nav = new ROSLIB.Topic({
  ros : ros,
  name : 'ardrone/navdata',
  messageType: 'ardrone_autonomy/Navdata'
}); 


// Subscribe to the navdata and log or assign data here
nav.subscribe(function(message) {
  //console.log('Battery percentage is : ' + message.batteryPercent + '% \n' + 'Estimated Altitude is : ' + message.altd + 'mm \n' + 'Linear velocities are : x: ' + message.vx + ' y: ' + message.vy + ' z :' + message.vz + '\n' + 'Time since drone started : ' + message.tm/1000000);
  document.getElementById('battery').innerHTML = "Battery percentage: " + message.batteryPercent + "%"; 
  document.getElementById('alt').innerHTML = "Estimated Altitude: " + message.altd/100 + "m"; 
  document.getElementById('vel').innerHTML = "Linear Velocities: x: " + message.vx/100 + "m/s  y: " + message.vy/100 + "m/s  z : " + message.vz/100  + "m/s";
  document.getElementById('time').innerHTML = "Time since started: " + message.tm/1000000 + "s"; 
});
