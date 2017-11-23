
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
  state: "global",
  PX_PER_METER: 120
};

var debug = {
  scan: true,
  velocity: true,
  path: true
}

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
  },
  image: new Image()
};

robot.image.src = "img/robot.png";

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
  name: '/aligned_scan',
  messageType: 'sensor_msgs/PointCloud'
});

/* Listener that listens to the /wall_positions topic. */
laserScanListener.subscribe(function(message) {
  laserScan = message.points;
});
