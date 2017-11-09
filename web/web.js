/* Toggle view state of map between global and local. */
var currViewState = "Local";
toggleViewState();
function toggleViewState() {
    var viewStateButton = document.getElementById("viewStateButton");
    if (currViewState === "Global") {
        currViewState = "Local";
        viewStateButton.style.content = "Local";
    } else {
        currViewState = "Global";
        viewStateButton.style.content = "Global";
    }
}

/* Center view of map */
function centerView() {
}

/* Zooms in on the robot in the map*/
function zoomInMap() {

}

/* Zooms out from the robot in the map*/ 
function zoomOutMap() {


}