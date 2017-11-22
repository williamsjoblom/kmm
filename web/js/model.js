
// The canvas 2d context used to draw stuff.
var ctx = $("#map")[0].getContext("2d");

// Information about the viewport.
var view = {
  zoom: 1,
  pan: {
    x: 0,
    y: 0
  },
  isDragging: false,
  prevDragPos: { x: 0, y: 0 },
  PX_PER_METER: 120
};

// Information about the robot
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
  laserScan = message.ranges;
});
