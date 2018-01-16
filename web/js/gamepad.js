
/* Creates a Ros-Topic */
var gamepadVel = new ROSLIB.Topic({
  ros : ros,
  name : '/gamepad',
  messageType : 'geometry_msgs/Twist'
});

setInterval(checkGamepad, 1000/30);

/* Takes the input from the gamepad if it's connected and sends it in
  a ROS-Message */
function checkGamepad(){
  var gamepad = navigator.getGamepads()[0];
  if (gamepad && !isInAutoMode && plannedPath.length == 0) {
    //Motion in x-direction
    var x = gamepad.axes[0];
    //Motion in y-direction
    var y = gamepad.axes[1];
    //the rotation, controlled by the right stick
    var rotation = gamepad.axes[2];

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
