
// The canvas 2d context used to draw stuff.
//var ctx = $("#map")[0].getContext("2d");
var ctx = $("#map")[0].getContext("2d");
var matrix = new Matrix(ctx);
var globalFrame = new Matrix();

// Information about the viewport.
var view = {
  zoom: 1,
  pan: {
    x: 0,
    y: 0
  },
  isDragging: false,
  prevDragPos: { x: 0, y: 0 },
  state: "global",
  rotation: 0,
  PX_PER_METER: 120
};

var debug = {
  axes : true,
  scan: true,
  aligned: true,
  endPoints: true,
  path: true,
  velocity: true,
  acceleration: true,
  target: true,
  goToTarget: true
}

var isUsingGoTo = false;

var goToPos = null;

var laserScan = {
  angle_min: 0,
  angle_increment: 0,
  ranges: []
};

// Information about the robot
var robot = {
  position: {
    x: 0.2,
    y: 0.2,
    angle: 0
  },
  target: {
    x: 0.2,
    y: 0.2,
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
  },
  image: new Image()
};

robot.image.src = "img/robot.png";
var targetImage = new Image();
targetImage.src = "img/target.png";

// Setup ROS connection
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

// Setup ROS subscribers
var pathListener = new ROSLIB.Topic({
  ros: ros,
  name: '/path',
  messageType: 'geometry_msgs/PoseArray'
})

var plannedPath = [];
pathListener.subscribe(function(message) {
  plannedPath = message.poses.map(function (pose) {
    return { x: pose.position.x, y: pose.position.y };
  });
});

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
  var q = message.pose.pose.orientation;
  var angle = Math.atan2(2*(q.x*q.y+q.z*q.w), 1-2*(Math.pow(q.y, 2)+Math.pow(q.z, 2)));
  robot.position.angle = angle;
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

var wallsListener = new ROSLIB.Topic({
  ros: ros,
  name: '/walls',
  messageType: 'std_msgs/Int8MultiArray'
});

var walls = [];
/* Listener that listens to the /walls topic. */
wallsListener.subscribe(function(message) {
  walls = [];
  var wall;
  var width = 51;
  var cell_size = 0.4;
  var offset = (width - 1)/2;
  var row = 0;
  var col = 0;
  var is_end_of_vertical;
  var is_end_of_horizontal;
  var horizontal = true;
  for (var i = 0; i < message.data.length; i++) {
    is_end_of_vertical = !horizontal && col == width + 1;
    is_end_of_horizontal = horizontal && col == width;
    if (is_end_of_vertical) {
      horizontal = true;
      row++;
      col = 0;
    } else if (is_end_of_horizontal) {
      horizontal = false;
      col = 0;
    };
    if (message.data[i]) { // Found wall
      from = Object.freeze({'x': row*cell_size, 'y': (col - offset)*cell_size});
      if (horizontal) { // On horizontal row in array
        to = Object.freeze({'x': row*cell_size, 'y': (col - offset)*cell_size + cell_size});
      } else { // On vertical row in array
        to = Object.freeze({'x': row*cell_size + cell_size, 'y': (col - offset)*cell_size});
      };
      wall = Object.freeze({'from': from, 'to': to});
      walls.push(wall);
    };
    col++;
  };
});


/* Listener that listens to the /end_points topic. */
var endPointListener = new ROSLIB.Topic({
  ros: ros,
  name: '/end_points',
  messageType: 'sensor_msgs/PointCloud'
});

var endPoints = [];
endPointListener.subscribe(function(message) {
  endPoints = [];
  for (var i = 0; i < message.points.length; i++) {
    endPoints.push(message.points[i]);
  };
});
/**/

/* Listener that listens to the /scan topic. */
var laserScanListener = new ROSLIB.Topic({
  ros: ros,
  name: '/scan',
  messageType: 'sensor_msgs/LaserScan'
});
laserScanListener.subscribe(function(message) {
  laserScan.angle_min = message.angle_min;
  laserScan.angle_increment = message.angle_increment;
  laserScan.ranges = message.ranges;
});
/**/

/* Listener that listens to the /scan topic. */
var alignedScanListener = new ROSLIB.Topic({
  ros: ros,
  name: '/aligned_scan',
  messageType: 'sensor_msgs/PointCloud'
});
var alignedScan = [];
alignedScanListener.subscribe(function(message) {
  alignedScan = message.points;
});
/**/

/* Subscriber and publisher for button state */
var isInManualMode = true;
var btnStateSub = new ROSLIB.Topic({ // Subscriber
  ros : ros,
  name : '/btn_state',
  messageType : 'std_msgs/Bool'
});

btnStateSub.subscribe(function(message) {
  isInManualMode = message.data;
  if (isInManualMode) {
    $("#mode-slider").prop("checked", false);
  } else {
    $("#mode-slider").prop("checked", true);
    goToPos = null;
  };
});

var btnStatePub = new ROSLIB.Topic({ // Publisher
  ros : ros,
  name : '/btn_state',
  messageType : 'std_msgs/Bool'
});
/**/

/* Action client for target position */
var navigationClient = new ROSLIB.ActionClient({
  ros : ros,
  serverName : '/navigation',
  actionName : 'kmm_navigation/MoveToAction'
});
/**/
