
// The canvas 2d context used to draw stuff.
var ctx = $("#map")[0].getContext("2d");
var matrix = new Matrix(ctx);
var globalFrame = new Matrix();

// Information about the viewport.
var view = {
  zoom: 1,
  rotation: 0,
  pan: {
    x: 0,
    y: 0
  },
  isDragging: false,
  prevDragPos: { x: 0, y: 0 },
  state: "global",
  PX_PER_METER: 150
};

// Debug options
var debug = {
  axes : true,
  scan: true,
  positionScan: true,
  mappingScan: true,
  endPoints: true,
  path: true,
  velocity: true,
  acceleration: true,
  walls: true
};

// Go to
var isUsingGoTo = false;
var goToPos = null;

// Laser scan
var laserScan = {
  angle_min: 0,
  angle_increment: 0,
  ranges: []
};

var positionScan = [];
var mappingScan = [];

// Information about the robot
var robot = {
  position: {
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
  },
  image: new Image()
};

robot.image.src = "img/robot.png";

var isInAutoMode = false;

// Planned path and target
var plannedPath = [];
var targetImage = new Image();
targetImage.src = "img/target.png";
var targetPositionGoal = null;

// Mapping
var walls = [];
var endPoints = [];

// Setup ROS connection
var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'

}).on('connection', function() {
  console.log('Connected to a websocket server.');

}).on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);

}).on('close', function() {
  console.log('Connection to websocket server closed. ');
});

// Setup ROS subscribers

// Planned path.
new ROSLIB.Topic({
  ros: ros,
  name: '/path',
  messageType: 'geometry_msgs/PoseArray'

}).subscribe(function(message) {
  plannedPath = message.poses.map(function (pose) {
    return { x: pose.position.x, y: pose.position.y };
  });
});

// Robot position and orientation.
new ROSLIB.Topic({
  ros: ros,
  name: '/position',
  messageType: 'geometry_msgs/PoseWithCovarianceStamped'

}).subscribe(function(message) {
  robot.position.x = message.pose.pose.position.x;
  robot.position.y = message.pose.pose.position.y;
  var q = message.pose.pose.orientation;
  // Quaternion to Euler angle.
  var angle = Math.atan2(2*(q.x*q.y+q.z*q.w), 1-2*(Math.pow(q.y, 2)+Math.pow(q.z, 2)));
  robot.position.angle = angle;
});

// Robot velocity.
new ROSLIB.Topic({
  ros: ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/Twist'

}).subscribe(function(message) {
  robot.velocity.x = message.linear.x;
  robot.velocity.y = message.linear.y;
  robot.velocity.w = message.angular.z;
});

// Robot wheel velocities.
new ROSLIB.Topic({
  ros: ros,
  name: '/wheel_velocities',
  messageType: 'kmm_drivers/WheelVelocities'

}).subscribe(function(message) {
  robot.wheelVelocities[0] = message.wheel_0;
  robot.wheelVelocities[1] = message.wheel_1;
  robot.wheelVelocities[2] = message.wheel_2;
});

// Map walls.
new ROSLIB.Topic({
  ros: ros,
  name: '/walls',
  messageType: 'std_msgs/Int8MultiArray'

}).subscribe(function(message) {
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


// Map endpoints.
new ROSLIB.Topic({
  ros: ros,
  name: '/end_points',
  messageType: 'sensor_msgs/PointCloud'

}).subscribe(function(message) {
  endPoints = [];
  for (var i = 0; i < message.points.length; i++) {
    endPoints.push(message.points[i]);
  };
});

// LIDAR laser scan.
new ROSLIB.Topic({
  ros: ros,
  name: '/scan',
  messageType: 'sensor_msgs/LaserScan'

}).subscribe(function(message) {
  laserScan.angle_min = message.angle_min;
  laserScan.angle_increment = message.angle_increment;
  laserScan.ranges = message.ranges;
});

// Position laser scan.
new ROSLIB.Topic({
  ros: ros,
  name: '/position_scan',
  messageType: 'sensor_msgs/PointCloud'

}).subscribe(function(message) {
  positionScan = message.points;
});

// Mapping laser scan.
new ROSLIB.Topic({
  ros: ros,
  name: '/mapping_scan',
  messageType: 'sensor_msgs/PointCloud'

}).subscribe(function(message) {
  mappingScan = message.points;
});

// Subscriber for auto_mode
new ROSLIB.Topic({
  ros : ros,
  name : '/auto_mode',
  messageType : 'std_msgs/Bool'

}).subscribe(function(message) {
  isInAutoMode = message.data;
});

// Service client for auto_mode
var setAutoModeClient = new ROSLIB.Service({
  ros : ros,
  name : '/set_auto_mode',
  serviceType : 'std_srvs/SetBool'
});

// Action client for move to navigation goal.
var navigationClient = new ROSLIB.ActionClient({
  ros : ros,
  serverName : '/navigation',
  actionName : 'kmm_navigation/MoveToAction'
});
