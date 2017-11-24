
// Render canvas 30Hz
setInterval(render, 33);

// Update DOM 10Hz
setInterval(updateDOM, 100);

function updateDOM() {
  //Position
  $("#pos-x").html(decimal(robot.position.x, 2));
  $("#pos-y").html(decimal(robot.position.y, 2));
  $("#theta").html(decimal(robot.position.angle/Math.PI, 3) + "\u03C0");
  //Target
  $("#tar-pos-x").html(decimal(robot.target.x, 1));
  $("#tar-pos-y").html(decimal(robot.target.y, 1));
  $("#tar-theta").html(decimal(robot.target.angle, 2) + " rad");
  //Velocity
  $("#vel-x").html(decimal(robot.velocity.x, 2) + " m/s");
  $("#vel-y").html(decimal(robot.velocity.y, 2) + " m/s");
  $("#vel-w").html(decimal(robot.velocity.w, 2) + " rad/s");
  //Wheel velocity
  $("#w-vel-1").html(decimal(robot.wheelVelocities[0], 2) + " rad/s");
  $("#w-vel-2").html(decimal(robot.wheelVelocities[1], 2) + " rad/s");
  $("#w-vel-3").html(decimal(robot.wheelVelocities[2], 2) + " rad/s");
  //Acceleration
  $("#acc-x").html(decimal(robot.acceleration.x, 1) + " m/s");
  $("#acc-y").html(decimal(robot.acceleration.y, 1) + " m/s");
  $("#acc-w").html(decimal(robot.acceleration.angle, 1) + " rad/s");
}

//Rounds to given decimal
function decimal(val, dec){
  return Math.round(val*Math.pow(10, dec))/Math.pow(10, dec);
}

function render() {
  clearCanvas();

  // Transform coordinate system to something that is practical to work with
  ctx.save();
  ctx.rotate(Math.PI / 2); // robotics standard, x is up, y is left.
  ctx.translate(ctx.canvas.height / 2, -ctx.canvas.width / 2); // Move origo to center.
  ctx.scale(-view.PX_PER_METER, view.PX_PER_METER);

  var restore = false;
  // Different view states.
  if (view.state == "global") {
    ctx.translate(-2, 0);
  } else {
    restore = true;
    view.pan.x = -robot.position.x * view.zoom;
    view.pan.y = -robot.position.y * view.zoom;
    view.rotation = -robot.position.angle;
  }

  // View settings
  ctx.rotate(view.rotation);
  ctx.translate(view.pan.x, view.pan.y);
  ctx.scale(view.zoom, view.zoom);

  // Global frame
  drawGrid();
  drawWalls();
  drawGlobalFrame();
  if (debug.aligned) {drawAlignedScan();};
  if (debug.endPoints) {drawEndPoints();};
  if (debug.path) {drawPath();};
  if (debug.target) {drawTarget();};

  { // Robot frame
    ctx.save();
    ctx.translate(robot.position.x, robot.position.y); //Uncomment to have laser data drawn at robot pos.
    ctx.rotate(robot.position.angle);
    if (debug.velocity) {drawVelocity();};
    if (debug.scan) {drawLaserScan();};
    if (debug.acceleration) {drawAcceleration();};
    drawRobot();
    ctx.restore();
  }
  ctx.restore();
}

function clearCanvas() {
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, ctx.canvas.width, ctx.canvas.height);
}

function drawTarget() {
  ctx.beginPath();
  ctx.fillStyle = "#00ff00";
  ctx.arc(robot.target.x, robot.target.y, 0.05, 0, 2*Math.PI);
  ctx.fill();
}

function drawPath() {
  ctx.lineWidth = 0.03;
  ctx.strokeStyle = "#004400";

  ctx.beginPath();
  if (plannedPath.length > 0) {
    ctx.beginPath();
    ctx.moveTo(plannedPath[0].x, plannedPath[0].y);
    for (var i = 1; i < plannedPath.length; i++) {
      ctx.lineTo(plannedPath[i].x, plannedPath[i].y);
    };
    ctx.stroke();
  };
}

/*
Draws real time laser scan.
*/
function drawLaserScan() {
  var rectSize = 0.02;
  ctx.fillStyle = "#0000ff";
  ctx.save();
  ctx.rotate(laserScan.angle_min + Math.PI);
  for (var i = 0; i < laserScan.ranges.length; i++){
    ctx.fillRect(laserScan.ranges[i] + (rectSize/2), (rectSize/2), rectSize, rectSize);
    ctx.rotate(laserScan.angle_increment);
  }
  ctx.restore();
}

function drawAlignedScan() {
  ctx.save();
  ctx.fillStyle = "#9C27B0";
  var rectHeight = 0.02;
  var rectWidth = 0.02;
  for (var i = 0; i < alignedScan.length; i++) {
    if (alignedScan[i].x + alignedScan[i].y > 0.1) {
      ctx.fillRect(alignedScan[i].x + (rectHeight/2), alignedScan[i].y + (rectWidth/2), rectWidth, rectHeight);
    };
  }
  ctx.restore();
}

function drawGrid() {
  ctx.lineWidth = 0.01;
  ctx.strokeStyle = "#AAAAAA";

  ctx.save();
  ctx.translate(0,0.2);
  var rows = 26; // (10 / 0.4) + 1
  var cols = 51; // ((10 / 0.4) * 2) + 1

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
  ctx.lineWidth = 0.03;
  ctx.strokeStyle = "#884422";
  for (var i = 0; i < walls.length; i++) {
    ctx.beginPath();
    ctx.moveTo(walls[i].from.x, walls[i].from.y);
    ctx.lineTo(walls[i].to.x, walls[i].to.y);
    ctx.stroke();
  };
}

function drawEndPoints() {
  var radius = 0.05;
  ctx.fillStyle = "#ffff00";
  for (var i = 0; i < endPoints.length; i++) {
    ctx.beginPath();
    ctx.arc(endPoints[i].x, endPoints[i].y, radius, 0, 2*Math.PI);
    ctx.fill();
  };
}

function drawGlobalFrame() {
  ctx.lineWidth = 0.02;
  var axisLength = 0.8;

  // Draw X axis in red.
  ctx.strokeStyle = "#FF0000";
  drawArrow(
    { x: 0, y: 0 },
    { x: axisLength, y: 0 }
  );

  // Draw Y axis in green.
  ctx.strokeStyle = "#00FF00";
  drawArrow(
    { x: 0, y: 0 },
    { x: 0, y: axisLength }
  );
}

function drawRobot() {
  ctx.save();
  ctx.rotate(Math.PI / 2);
  var scale = 0.001;
  var pixels = 300;
  var picSize = pixels * scale;
  ctx.drawImage(robot.image, picSize / -2, picSize / -2, picSize, picSize);
  ctx.restore();
}

function drawArrow(from, to){
  var headlen = 0.1;
  var angle = Math.atan2(to.y - from.y, to.x - from.x);
  var cos1 = Math.cos(angle - Math.PI / 6);
  var sin1 = Math.sin(angle - Math.PI / 6);
  var cos2 = Math.cos(angle + Math.PI / 6);
  var sin2 = Math.sin(angle + Math.PI / 6);

  ctx.beginPath();
  ctx.moveTo(from.x, from.y);
  ctx.lineTo(to.x, to.y);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(to.x, to.y);
  ctx.lineTo(to.x - headlen * cos1, to.y - headlen * sin1);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(to.x, to.y);
  ctx.lineTo(to.x - headlen * cos2, to.y - headlen * sin2);
  ctx.stroke();
}

function drawAcceleration() {
  ctx.strokeStyle = "#000000";
  ctx.lineWidth = 0.005;
  var from = { x: 0, y: 0 };
  var to = robot.acceleration;
  drawArrow(from, to);
}

function drawVelocity() {
  ctx.strokeStyle = "#000000";
  ctx.lineWidth = 0.02;
  var from = { x: 0, y: 0 };
  var to = {
    x: robot.velocity.x * 4,
    y: robot.velocity.y * 4
  };
  drawArrow(from, to);
}
