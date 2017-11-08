
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

var pxPerMeter = 120;

var isDragging = false;
var prevDragPos = { x: 0, y: 0 };

/* * * * * * * * */

// When the document has finished loading.
$(document).ready(function () {
  // Bind key, scroll, click events.
  bindEvents();

  // Initiate view state.
  toggleViewState();

  // Save the canvas 2D context for later use.
  ctx = $("#map")[0].getContext("2d");

  // Set initial canvas size.
  resizeCanvas();

  // Set an interval to show random data every second.
  setInterval(randomData, 1000);

  // Render canvas 60 fps.
  setInterval(render, 0.016);
});

// If the window is resized the context needs to know in order
// to not squeeze the graphics in weird ways.
$(window).resize(resizeCanvas);

function bindEvents() {
  // Bind map buttons.
  $("#zoom-in-button").click(zoomIn);
  $("#zoom-out-button").click(zoomOut);
  $("#view-state-button").click(toggleViewState);
  $("#center-view-button").click(centerView);

  // Bind map scoll zoom and mouse pan.
  $("#map")
  .mousedown(function(e) {
    isDragging = true;
    prevDragPos.x = e.screenX;
    prevDragPos.y = e.screenY;
  })
  .mousemove(function(e) {
    if (isDragging) {
      view.pan.y += (prevDragPos.x - e.screenX) / pxPerMeter;
      view.pan.x += (prevDragPos.y - e.screenY) / pxPerMeter;
      prevDragPos.x = e.screenX;
      prevDragPos.y = e.screenY;
    }
  })
  .mouseup(function() {
    isDragging = false;
  })
  .on("wheel", function (e) {
    e.preventDefault();
    view.zoom *= 1 - e.originalEvent.deltaY * 0.001;
  });
}


function resizeCanvas() {
  var $container = $("#map-container");
  ctx.canvas.width = $container.width();
  ctx.canvas.height = $container.height();
}

var laserScan = [];
for (var i = 0; i < 360; i++) {
  laserScan.push(Math.random() * 6)
}

function render() {
  clearScreen();

  // Transform coordinate system to something that is practical to work with
  ctx.save();
  ctx.rotate(Math.PI / 2); // robotics standard, x is up, y is left.
  ctx.translate(ctx.canvas.height / 2, -ctx.canvas.width / 2); // Move origo to center.
  ctx.scale(-pxPerMeter, pxPerMeter);
  ctx.translate(-2, 0);

  // View settings
  ctx.translate(view.pan.x, view.pan.y);
  ctx.scale(view.zoom, view.zoom);

  drawGrid();
  drawGlobalFrame();
  drawLaserScan(laserScan);

  ctx.restore();
}

function clearScreen() {
  ctx.fillStyle = "#fff";
  ctx.fillRect(0, 0, ctx.canvas.width, ctx.canvas.height);
}

function drawGlobalFrame() {
  ctx.lineWidth = 0.03;
  // Draw x axis in red
  ctx.strokeStyle = "#FF0000";
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(0.6, 0);
  ctx.stroke();
  // Draw y axis in green
  ctx.strokeStyle = "#00FF00";
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(0, 0.6);
  ctx.stroke();
}

function drawGrid() {
  ctx.lineWidth = 0.01;
  ctx.strokeStyle = "#AAAAAA";
  var n = 27;
  for (var i = 0; i < n; i++) {
    // horizontal
    ctx.beginPath();
    ctx.moveTo(0.4*i, 0.4*n/-2);
    ctx.lineTo(0.4*i, 0.4*n/2);
    ctx.stroke();
    // vectical
    ctx.beginPath();
    ctx.moveTo(0.4*n/-2 + 0.4*(n-1)/2, 0.4*i - 0.4*(n-1)/2);
    ctx.lineTo(0.4*n/2  + 0.4*(n-1)/2, 0.4*i - 0.4*(n-1)/2);
    ctx.stroke();
  }
}

function drawLaserScan(laserScan) {
  ctx.save();
  var rectHeight = 0.02;
  var rectWidth = 0.02;
  for (var i = 0; i < 360; i++) {
    ctx.rotate(1);
    ctx.fillStyle = "#9C27B0";
    ctx.fillRect(laserScan[i] + rectHeight/2, rectWidth/2, rectWidth, rectHeight);
  }
  ctx.restore();
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
  view = {
    zoom: 1,
    pan: {
      x: 0,
      y: 0
    }
  };
}

function zoomIn() {
  view.zoom *= 1.2;
}

function zoomOut() {
  view.zoom *= 0.8;
}
