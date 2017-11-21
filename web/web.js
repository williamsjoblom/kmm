
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

var robot = {
  position: {
    x: 0,
    y: 0,
    angle: 0
  },
  target: {
    x: 0,
    y: 0,
    angle: 0
  },
  velocity: {
    x: 0,
    y: 0,
    w: 0
  },
  wheelVelocities: [0, 0, 0],
  acceleration: {
    x: 0,
    y: 0,
    angle: 0
  }
};

setInterval(updateDom, 100);

function updateDom() {
  //Position
  $("#pos-x").html(decimal(robot.position.x, 1));
  $("#pos-y").html(decimal(robot.position.y, 1));
  $("#theta").html(decimal(robot.position.angle, 1) + " rad");
  //Target
  $("#tar-pos-x").html(decimal(robot.target.x, 1));
  $("#tar-pos-y").html(decimal(robot.target.y, 1));
  $("#tar-theta").html(decimal(robot.target.angle, 1) + " rad");
  //Velocity
  $("#vel-x").html(decimal(robot.velocity.x, 1) + " m/s");
  $("#vel-y").html(decimal(robot.velocity.y, 1) + " m/s");
  $("#vel-w").html(decimal(robot.velocity.w, 1) + " rad/s");
  //Wheel velocity
  $("#w-vel-1").html(decimal(robot.wheelVelocities[0], 1) + " rad/s");
  $("#w-vel-2").html(decimal(robot.wheelVelocities[1], 1) + " rad/s");
  $("#w-vel-3").html(decimal(robot.wheelVelocities[2], 1) + " rad/s");
  //Acceleration
  $("#acc-x").html(decimal(robot.acceleration.x, 1) + " m/s");
  $("#acc-y").html(decimal(robot.acceleration.y, 1) + " m/s");
  $("#acc-w").html(decimal(robot.acceleration.angle, 1) + " rad/s");
}

//Rounds to given decimal
function decimal(val, dec){
  return Math.round(val*Math.pow(10, dec))/Math.pow(10, dec);
}

/* * * * * * * * */

/* SETS UP ROS */

var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
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
  drawWalls();
  drawEndPoints();
  drawGlobalFrame();
  drawRobot();
  updateView();
  //drawAcceleration();
  drawVelocity();
  //drawLaserScan(laserScan);
  drawTarget();

  ctx.restore();
}

function clearScreen() {
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, ctx.canvas.width, ctx.canvas.height);
}

function drawTarget() {
  ctx.beginPath();
  ctx.strokeStyle = "#00ff00";
  ctx.arc(robot.target.x, robot.target.y, 0.05, 0, 2*Math.PI);
  ctx.stroke();
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

function drawLaserScan(laserScan) {
  ctx.save();
  ctx.translate(robot.position.x, robot.position.y); //Uncomment to have laser data drawn at robot pos.
  ctx.rotate(robot.position.angle - Math.PI);
  ctx.fillStyle = "#9C27B0";
  var rectHeight = 0.02;
  var rectWidth = 0.02;
  for (var i = 0; i < 360; i++) {
    ctx.rotate(Math.PI/180);
    if (laserScan[i] > 0.1) {
      ctx.fillRect(laserScan[i] + (rectHeight/2), (rectWidth/2), rectWidth, rectHeight);
    };
  }
  ctx.restore();
}

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
  ctx.save();
  ctx.translate(0,0.2);
  var rows = 26; // (10 / 0.4) + 1
  var cols = 51; // ((10 / 0.4) * 2) + 1
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
  ctx.restore();
}

function drawWalls() {
  setLineStyle("wall");
  var currRow;
  var currCol;
  // Horizontal walls
  for (var i = 0; i < horizontalWalls.length; i++) {
    currRow = horizontalWalls[i].row;
    currCol = horizontalWalls[i].col;
    ctx.beginPath();
    ctx.moveTo(0.4*currRow, 0.4*currCol);
    ctx.lineTo(0.4*currRow, 0.4*currCol - 0.4*(currCol/Math.abs(currCol)));
    ctx.stroke();
  };
  // Vertical walls
  for (var i = 0; i < verticalWalls.length; i++) {
    currRow = verticalWalls[i].row;
    currCol = verticalWalls[i].col;
    ctx.beginPath();
    ctx.moveTo(0.4*currRow, 0.4*currCol);
    ctx.lineTo(0.4*currRow - 0.4*(currRow/Math.abs(currRow)), 0.4*currCol);
    ctx.stroke();
  };
}

function drawEndPoints() {
  ctx.fillStyle = "#FF0000";
  var sideLength = 0.1;
  for (var i = 0; i < endPoints.length; i++) {
    ctx.fillRect(endPoints[i].x - sideLength / 2, endPoints[i].y - sideLength / 2, sideLength, sideLength);
  };
}

function drawGlobalFrame() {
  ctx.lineWidth = 0.02;
  var arrowHeadOffset = 0.07;
  var axisLength = 0.8;
  // Draw x axis in red
  ctx.strokeStyle = "#FF0000";
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(axisLength, 0);
  ctx.lineTo(axisLength - arrowHeadOffset, arrowHeadOffset);
  ctx.moveTo(axisLength, 0);
  ctx.lineTo(axisLength - arrowHeadOffset, -arrowHeadOffset);
  ctx.stroke();
  // Draw y axis in green
  ctx.strokeStyle = "#00FF00";
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(0, axisLength);
  ctx.lineTo(arrowHeadOffset, axisLength - arrowHeadOffset);
  ctx.moveTo(0, axisLength);
  ctx.lineTo(-arrowHeadOffset, axisLength - arrowHeadOffset);
  ctx.stroke();
}

function drawRobot() {
  ctx.fillStyle = "#0000FF";
  var sideLength = 0.1;
  ctx.fillRect(robot.position.x - sideLength / 2, robot.position.y - sideLength / 2, sideLength, sideLength);
}

function drawArrow(fromx, fromy, tox, toy){
  var headlen = 0.1;   // length of head in pixels
  var angle = Math.atan2(toy-fromy,tox-fromx);

  ctx.strokeStyle = "#000000";
  ctx.lineWidth = 0.02;

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

function drawAcceleration(){
  var fromx = robot.position.x;
  var fromy = robot.position.y;
  var tox = fromx + robot.acceleration.x;
  var toy = fromy + robot.acceleration.y;

  drawArrow(fromx, fromy, tox, toy);
}

function drawVelocity(){
  var fromx = robot.position.x;
  var fromy = robot.position.y;
  var tox = fromx + robot.velocity.x * 2;
  var toy = fromy + robot.velocity.y * 2;

  drawArrow(fromx, fromy, tox, toy);
}

// Toggle view state of map between global and local.
function toggleViewState() {
  var $viewStateElem = $("#view-state-button");
  if ($viewStateElem.html() === "Global") {
    $viewStateElem.html("Local");
  } else {
    $viewStateElem.html("Global");
    view = {
      zoom: view.zoom,
      pan: {
        x: 0,
        y: 0
      }
    };
  }
}

function updateView() {
  var $viewStateElem = $("#view-state-button");
  if ($viewStateElem.html() === "Local") {
    view = {
      zoom: view.zoom,
      pan: {
        x: -robot.position.x * view.zoom,
        y: -robot.position.y * view.zoom
      }
    };
  };
}

function centerView() {
  var $viewStateElem = $("#view-state-button");
  if ($viewStateElem.html() === "Global") {
    view = {
      zoom: view.zoom,
      pan: {
        x: 0,
        y: 0
      }
    };
  }
}

function zoomIn() {
  view.zoom *= 1.2;
}

function zoomOut() {
  view.zoom *= 0.8;
}

/*
LISTENERS
*/

var targetPositionListener = new ROSLIB.Topic({
  ros: ros,
  name: '/target_position',
  messageType: 'geometry_msgs/Twist'
})

targetPositionListener.subscribe(function(message) {
  robot.target.x = message.linear.x;
  robot.target.y = message.linear.y;
  robot.target.angle = message.angular.z;
});

var robotPositionListener = new ROSLIB.Topic({
  ros: ros,
  name: '/position',
  messageType: 'geometry_msgs/PoseWithCovarianceStamped'
})

robotPositionListener.subscribe(function(message) {
  robot.position.x = message.pose.pose.position.x;
  robot.position.y = message.pose.pose.position.y;
  robot.position.angle = message.pose.pose.orientation.z;
});

var robotVelocityListener = new ROSLIB.Topic({
  ros: ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/Twist'
})

robotVelocityListener.subscribe(function(message) {
  robot.velocity.x = message.linear.x;
  robot.velocity.y = message.linear.y;
  robot.velocity.w = message.angular.z;
});

var wheelVelocityListener = new ROSLIB.Topic({
  ros: ros,
  name: '/wheel_velocities',
  messageType: 'kmm_drivers/WheelVelocities'
});

wheelVelocityListener.subscribe(function(message) {
  robot.wheelVelocities[0] = message.wheel_0;
  robot.wheelVelocities[1] = message.wheel_1;
  robot.wheelVelocities[2] = message.wheel_2;
});

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
  for (var i = 0; i < message.vertical_walls.length; i++) {
    verticalWall = Object.freeze({'row': message.vertical_walls[i].x,
      'col': message.vertical_walls[i].y})
    verticalWalls.push(verticalWall);
  };
});

var endPointListener = new ROSLIB.Topic({
  ros: ros,
  name: '/end_points',
  messageType: 'sensor_msgs/PointCloud'
});

var endPoints = [];

/* Listener that listens to the /end_points topic. */
endPointListener.subscribe(function(message) {
  console.log('Received message on ' + endPointListener.name);
  endPoints = [];
  for (var i = 0; i < message.points.length; i++) {
    endPoints.push(message.points[i]);
  };
});

var laserScan = [];

var laserScanListener = new ROSLIB.Topic({
  ros: ros,
  name: '/scan',
  messageType: 'sensor_msgs/LaserScan'
});

/* Listener that listens to the /wall_positions topic. */
laserScanListener.subscribe(function(message) {
  console.log('Received message on ' + laserScanListener.name);
  laserScan = message.ranges;
});
