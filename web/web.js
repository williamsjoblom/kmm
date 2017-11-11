
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

var currViewState = "Local";

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
    view.zoom *= 1 - e.originalEvent.deltaY * 0.001;
  });
}


function resizeCanvas() {
  var $container = $("#map-container")
  ctx.canvas.width = $container.width();
  ctx.canvas.height = $container.height();
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
  drawAcceleration();

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

//* The Ros object, wrapping a web socket connection to rosbridge.
var ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090' // url to your rosbridge server
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

//* A topic for messaging.
var wallPositionsListener = new ROSLIB.Topic({
  ros: ros,
  name: '/wall_positions',
  messageType: 'kmm_mapping/wall_positions'
});

var horizontalWall;
var horizontalWalls = [];
var verticalWall;
var verticalWalls = [];

/* Listener that listens to the /wall_positions topic. */
wallPositionsListener.subscribe(function(message) {
    console.log('Received message on ' + wallPositionsListener.name);
    horizontalWalls = [];
    verticalWalls = [];
    for (var i = 0; i < message.horizontal_walls.length; i++) {
      horizontalWall = Object.freeze({'row': message.horizontal_walls[i].x,
        'col': message.horizontal_walls[i].y})
      horizontalWalls.push(horizontalWall);
    };
    console.log(horizontalWall);
    for (var i = 0; i < message.vertical_walls.length; i++) {
      verticalWall = Object.freeze({'row': message.vertical_walls[i].x,
        'col': message.vertical_walls[i].y})
      verticalWalls.push(verticalWall);
    };
  });

function setLineStyle(type) {
  if (type == "wall") {
    ctx.lineWidth = 0.03;
    ctx.strokeStyle = "#000000";
  } else {
    ctx.lineWidth = 0.01;
    ctx.strokeStyle = "#AAAAAA";
  };
}

function drawGrid() {
  var rows = 15; // 6 / 0.4
  var cols = 30; // ((6 / 0.4) * 2)
  setLineStyle("normal");
  // horizontal grid lines
  for (var i = 0; i < rows + 1; i++) {
    ctx.beginPath();
    ctx.moveTo(0.4*(rows - i), 0.4*(cols/2));
    ctx.lineTo(0.4*(rows - i), 0.4*(cols/-2));
    ctx.stroke();
  };
  // vertical grid lines
  for (var i = 0; i < cols + 1; i++) {
    ctx.beginPath();
    ctx.moveTo(0.4*(rows), ((0.4*cols)/2) - (0.4 * i));
    ctx.lineTo(0, ((0.4*cols)/2) - (0.4 * i));
    ctx.stroke();
  };
  setLineStyle("wall");
  var currRow;
  var currCol;
  // Horizontal walls
  for (var i = 0; i < horizontalWalls.length; i++) {
    currRow = horizontalWalls[i].row;
    currCol = horizontalWalls[i].col;
    ctx.beginPath();
    ctx.moveTo(0.4*(rows - (currRow - 1)), (0.4*(cols/2)) - (0.4 * (currCol - 1)));
    ctx.lineTo(0.4*(rows - (currRow - 1)), (0.4*(cols/2)) - (0.4 * (currCol)));
    ctx.stroke();
  };
  // Vertical walls
  for (var i = 0; i < verticalWalls.length; i++) {
    currRow = verticalWalls[i].row;
    currCol = verticalWalls[i].col;
    ctx.beginPath();
    ctx.moveTo(0.4*(rows - (currRow - 1)), (0.4*(cols/2)) - (0.4 * (currCol - 1)));
    ctx.lineTo(0.4*(rows - currRow), (0.4*(cols/2)) - (0.4 * (currCol - 1)));
    ctx.stroke();
  };
}

function drawRobot(){

}

function drawAcceleration(){
  var headlen = 0.1;   // length of head in pixels
  var fromx = Number($("#pos-x").html());
  var fromy = Number($("#pos-y").html());
  var tox = fromx + Number($("#acc-x").html());
  var toy = fromy + Number($("#acc-y").html());
  var angle = Math.atan2(toy-fromy,tox-fromx);

  ctx.strokeStyle = "#000000";

  ctx.beginPath();
  ctx.moveTo(fromx, fromy);
  ctx.lineTo(tox, toy);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(tox, toy);
  ctx.lineTo(tox-headlen*Math.cos(angle-Math.PI/6),toy-headlen*Math.sin(angle-Math.PI/6));
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(tox, toy);
  ctx.lineTo(tox-headlen*Math.cos(angle+Math.PI/6),toy-headlen*Math.sin(angle+Math.PI/6));
  ctx.stroke();
}

function randomData() {
  $("#pos-x").html(Math.round(Math.random()*40)/10);
  $("#pos-y").html(Math.round(Math.random()*40 - 20)/10);
  $("#theta").html(Math.round(Math.random() * 360));
  $("#acc-x").html(Math.round(Math.random()*10 - 5)/10);
  $("#acc-y").html(Math.round(Math.random()*10 - 5)/10);
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
