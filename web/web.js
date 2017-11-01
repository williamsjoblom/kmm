
/* GLOBAL VARIABLES */

var ctx; // The canvas 2d context used to draw stuff.

// An object holding information about the viewport.
var view = {
  zoom: 1,
  pan: {
    x: 0,
    y: 0
  }
};

/* * * * * * * * */

// When the document has finished loading.
$(document).ready(function () {
  // Initiate view state.
  toggleViewState();
  // Save the canvas 2D context for later use.
  ctx = $("#map")[0].getContext("2d");
  // Set initial canvas size.
  resizeCanvas();
  // Render the canvas once.
  render();
  // Set an interval to show random data every second.
  setInterval(randomData, 1000);
});

// If the window is resized the context needs to know in order
// to not squeeze the graphics in weird ways.
$(window).resize(function () {
  resizeCanvas();
  render();
});

// Bind buttons.
$("#zoom-in-button").click(zoomIn);
$("#zoom-out-button").click(zoomOut);
$("#view-state-button").click(toggleViewState);
$("#center-view-button").click(centerView);

function resizeCanvas() {
  var $container = $("#map-container")
  ctx.canvas.width = $container.width();
  ctx.canvas.height = $container.height();
}

function render() {
  clearScreen();

  // Transform coordinate system to something that is practical to work with
  ctx.save();
  ctx.scale(1, -1);
  ctx.rotate(Math.PI / 2); // robotics standard, x is up, y is left.
  ctx.translate(-ctx.canvas.height / 2, -ctx.canvas.width / 2); // Move origo to center.
  ctx.scale(60, 60); // 1 meter is 60 px.

  drawGrid();
  drawGlobalFrame();

  ctx.restore();
}

function clearScreen() {
  ctx.fillStyle = "#fff";
  ctx.fillRect(0, 0, ctx.canvas.width, ctx.canvas.height);
}

function drawGlobalFrame() {
  ctx.lineWidth = 0.05;
  // Draw x axis in red
  ctx.strokeStyle = "#FF0000";
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(1, 0);
  ctx.stroke();
  // Draw y axis in green
  ctx.strokeStyle = "#00FF00";
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(0, 1);
  ctx.stroke();
}

function drawGrid() {
  ctx.lineWidth = 0.01;
  ctx.strokeStyle = "#AAAAAA";
  var n = 27;
  for (var i = 0; i < n; i++) {
    // horizontal
    ctx.beginPath();
    ctx.moveTo(0.4*i - 0.4*(n-1)/2, 0.4*n/-2);
    ctx.lineTo(0.4*i - 0.4*(n-1)/2, 0.4*n/2);
    ctx.stroke();
    // vectical
    ctx.beginPath();
    ctx.moveTo(0.4*n/-2, 0.4*i - 0.4*(n-1)/2);
    ctx.lineTo(0.4*n/2, 0.4*i - 0.4*(n-1)/2);
    ctx.stroke();
  }
}

function randomData() {
  $("#pos-x").html(Math.round(Math.random() * 100));
  $("#pos-y").html(Math.round(Math.random() * 100));
  $("#theta").html(Math.round(Math.random() * 100));
  $("#tar-pos-x").html(Math.round(Math.random() * 100));
  $("#tar-pos-y").html(Math.round(Math.random() * 100));
  $("#tar-theta").html(Math.round(Math.random() * 100));
  $("#w-vel-1").html(Math.round(Math.random() * 100) + " m/s");
  $("#w-vel-2").html(Math.round(Math.random() * 100) + " m/s");
  $("#w-vel-3").html(Math.round(Math.random() * 100) + " m/s");
}

// Toggle view state of map between global and local.
function toggleViewState() {
  var $viewStateElem = $("#view-state-button");
  if ($viewStateElem.html() === "Global") {
    $viewStateElem.html("Local");
  } else {
    $viewStateElem.html("Global");
  }
}

function centerView() {
  alert("center view!");
}

function zoomIn() {
  alert("zoom in!");
}

function zoomOut() {
  alert("zoom out!");
}
