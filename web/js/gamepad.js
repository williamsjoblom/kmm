
/* GLOBAL VARIABLES */

// Boolean that is true if a gamepad is connected.
var connected = false;
// The current gamepad object
var gamepad;

/* Creates a Ros-Topic */
var gamepadVel = new ROSLIB.Topic({
  ros : ros,
  name : '/gamepad_vel',
  messageType : 'geometry_msgs/Twist'
});


/* Sets upp eventlisteners for when the gamepad gets connected and disconnected.
Starts listening as soon as the page has loaded.
*/
$(document).ready(function() {
  window.addEventListener("gamepadconnected", function(e) {
    gamepadHandler(e, true);
  }, false);
  window.addEventListener("gamepaddisconnected", function(e) {
      gamepadHandler(e, false);
  }, false);
  window.setInterval(checkMovement, 100);
});


/* Logs to the console if the gamepad gets connected/disconnected
 and sets the connected boolean as approriate.*/
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


/* Takes the input from the gamepad if it's connected and sends it in
  a ROS-Message */
function checkMovement() {
  if (connected && !isInAutoMode) {
    //Motion in x-direction
    var x = gamepad.axes[0];
    //Motion in y-direction
    var y = -gamepad.axes[1];
    //the rotation, controlled by the right stick
    var rotation = gamepad.axes[3];

    //Uncomment the following line to log input from the gamepad
    //console.log("X: "+x+ ", Y: "+y+", Rotation: "+ rotation);
    //Create and sends ROS-message
    var movement = new ROSLIB.Message({
      linear : {
        x : x,
        y : y
      },
      angular : {
        z : rotation
      }
    });
    gamepadVel.publish(movement);
  }
}
