
// The canvas 2d context used to draw stuff.
var ctx = $("#map")[0].getContext("2d");
var matrix = new Matrix(ctx);
var globalMatrix = new Matrix();

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
  sidebarState: true,
  PX_PER_METER: 150
};

// Debug options
var debug = {
  globalFrame : true,
  scan: true,
  positionScan: true,
  mappingScan: true,
  endPoints: true,
  path: true,
  velocity: true,
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
  image: new Image()
};

robot.image.src = "img/robot.png";

// Planned path and target
var plannedPath = [];
var targetImage = new Image();
targetImage.src = "img/target.png";
var targetPositionGoal = null;

// Mapping
var walls = [];
var endPoints = [];

var url = window.location.href.replace("http://", "").replace("/", "").replace(":8080", "");

// Setup ROS connection
var ros = new ROSLIB.Ros({
  url : 'ws://' + url + ':9090'

}).on('connection', function() {
  console.log('Connected to a websocket server.');

}).on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);

}).on('close', function() {
  console.log('Connection to websocket server closed. ');
});

// Get map parameters from ROS

ros.getParams(function(params) {});

var rows;
var map_rows_param = new ROSLIB.Param({
  ros : ros,
  name : 'map_rows'
}).get(function(value) {
  rows = value;
});

var cols;
var map_cols_param = new ROSLIB.Param({
  ros : ros,
  name : 'map_cols'
}).get(function(value) {
  cols = value;
});

var cell_size;
var cell_size_param = new ROSLIB.Param({
  ros : ros,
  name : 'cell_size'
}).get(function(value) {
  cell_size = value;
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
  var offset = (cols - 1)/2;
  var row = 0;
  var col = 0;
  var is_end_of_vertical; // If at the end of a row of vertical walls
  var is_end_of_horizontal; // If at the end of a row of horizontal walls
  var horizontal = true;
  for (var i = 0; i < message.data.length; i++) {
    is_end_of_vertical = !horizontal && col == cols + 1;
    is_end_of_horizontal = horizontal && col == cols;
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
var isInAutoMode = false;
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

// Service client for resetting position
var resetPositionClient = new ROSLIB.Service({
  ros : ros,
  name : '/reset_position',
  serviceType : 'std_srvs/SetBool'
});

// Service client for resetting map
var resetMapClient = new ROSLIB.Service({
  ros : ros,
  name : '/reset_map',
  serviceType : 'std_srvs/SetBool'
});

// Subscriber for mapping bool
var mapping = true;
new ROSLIB.Topic({
  ros : ros,
  name : '/mapping',
  messageType : 'std_msgs/Bool'

}).subscribe(function(message) {
  mapping = message.data;
  //console.log(mapping);
});

// Service client for enabling/disabling mapping
var setMappingClient = new ROSLIB.Service({
  ros : ros,
  name : '/set_mapping',
  serviceType : 'std_srvs/SetBool'
});

// Action client for move to navigation goal.
var navigationClient = new ROSLIB.ActionClient({
  ros : ros,
  serverName : '/navigation',
  actionName : 'kmm_navigation/MoveToAction'
});

// Subscriber for finished_mapping
var finishedMapping;
new ROSLIB.Topic({
  ros : ros,
  name : '/finished_mapping',
  messageType : 'std_msgs/Bool'

}).subscribe(function(message) {
  wasFinished = finishedMapping;
  finishedMapping = message.data;
  if (!wasFinished && finishedMapping) {
    var fireworks = new Fireworks();
  }
});
