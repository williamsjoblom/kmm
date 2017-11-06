// Boolean that is true if a gamepad is connected.
var connected = false;
// The current gamepad object
var gamepad;

var ros = new ROSLIB.ros({
  url : 'ws://localhost:8080'
});

ros.on('connection', function() {
  console.los('Connected to a websocket server.');
}

ros.on('error', function(error) {
  console.lof('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed. ')
})

var gamepadVel = new ROSLIB.Topic({
  ros : ros,
  name : '/gamepad_vel',
  messageType : 'geometry_msg/Twist'
});






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
  window.setInterval(checkMovement, 100);
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
    var x = gamepad.axes[1];
    var y = gamepad.axes[0];
    var theta = gamepad.axes[2];
    //TODO: check deadzone
    //TODO: implement low pass filter
    //TODO: Check how x,y, theta corresponds to the controller

  }
}
