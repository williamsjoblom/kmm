// Boolean that is true if a gamepad is connected.
var connected = false;
// The current gamepad object
var gamepad;
var DEADZONE = 0.05;
/*
var ros = new ROSLIB.Ros({
  url : 'ws://localhost:8080'
});

ros.on('connection', function() {
  console.log('Connected to a websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed. ');
});

var gamepadVel = new ROSLIB.Topic({
  ros : ros,
  name : '/gamepad_vel',
  messageType : 'geometry_msg/Twist'
});

*/


/* Sets upp eventlisteners for when the gamepad gets connected and disconnected.
Starts listening as soon as the page has loaded.
*/
$(document).ready(function() {
  window.addEventListener("gamepadconnected", function(e) {
    gamepadHandler(e, true)
  }, false);
  window.addEventListener("gamepaddisconnected", function(e) {
      gamepadHandler(e, false);
  }, false)
  window.setInterval(checkMovement, 500); //TODO: change back to 100
});

/* Logs to the console if the gamepad gets connected/disconnected
 and sets the connected boolean as approriate.
*/
function gamepadHandler(event, connecting){
  gamepad = event.gamepad;
  if (connecting){
    console.log("Gamepad connected");
    connected = true;
  } else {
    console.log("Gamepad disconnected");
    connected = false;
  }
}

function checkMovement(){
  if (connected) {
    //Motion in x-direction
    var x = -gamepad.axes[0];
    //Motion in y-direction
    var y = -gamepad.axes[1];
    //the rotation, controlled by the right stick
    var theta = -gamepad.axes[3];
    //Checks that an actuall instruction was sent
    (Math.abs(x) < DEADZONE) ? x=0 : x=x;
    (Math.abs(y) < DEADZONE) ? y=0 : y=y;
    (Math.abs(theta) < DEADZONE) ? theta=0 : theta=theta;

    console.log("X: "+x+ ", Y: "+y+" ,Theta: "+ theta);
    //TODO: implement low pass filter

  }
}

//function lowpass(x)
