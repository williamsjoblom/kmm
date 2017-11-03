// Boolean that is true if a gamepad is connected.
var connected = false;
// The current gamepad object
var gamepad;

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
