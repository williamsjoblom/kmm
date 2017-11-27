
// When the document has finished loading.
$(document).ready(function () {
  // Bind key, scroll, click events.
  bindEvents();

  // Set initial canvas size.
  resizeCanvas();
  // Bind the window resize event. If the window
  // is resized the context needs to know in order
  // to not squeeze the graphics in weird ways.
  $(window).resize(resizeCanvas);
});

function bindEvents() {
  bindCanvasEvents();
  bindMenuEvents();
  bindSidebarEvents();
  bindButtonEvents();
}

function bindCanvasEvents() {
  // Bind map buttons.
  $("#zoom-in-button").click(zoomIn);
  $("#zoom-out-button").click(zoomOut);
  $("#view-state-button").click(toggleViewState);
  $("#center-view-button").click(centerView);

  // Bind map scoll zoom and mouse pan.
  $("#map")
  .click(function(e) {
    if (!isInAutoMode && isUsingGoTo) {
      isUsingGoTo = false;
      $("#map").css('cursor', 'default');
      $("#go-to").html("Remove goal");

      var offset = $("#map-container").offset();

      goToPos = globalMatrix.applyToPoint(
        e.pageX - offset.left,
        e.pageY - offset.top
      );

      var targetPositionGoal = new ROSLIB.Goal({
        actionClient : navigationClient,
        goalMessage : {
          x : goToPos.x,
          y : goToPos.y,
          angle : 0,
        }
      });
      targetPositionGoal.send();
    };
  })
  .mousedown(function(e) {
    view.isDragging = true;
    view.prevDragPos.x = e.screenX;
    view.prevDragPos.y = e.screenY;
  })
  .mousemove(function(e) {
    if (view.isDragging) {
      view.pan.y += (view.prevDragPos.x - e.screenX) / view.PX_PER_METER;
      view.pan.x += (view.prevDragPos.y - e.screenY) / view.PX_PER_METER;
      view.prevDragPos.x = e.screenX;
      view.prevDragPos.y = e.screenY;
    }
  })
  .mouseup(function() {
    view.isDragging = false;
  })
  .on("wheel", function (e) {
    e.preventDefault();
    view.zoom *= 1 - e.originalEvent.deltaY * 0.001;
  });
}

function bindMenuEvents() {
  $("#debug-axes").click(function () {
    debug.axes = !debug.axes;
  });
  $('#debug-axes').removeAttr('checked');
  debug.axes = false;

  $("#debug-scan").click(function () {
    debug.scan = !debug.scan;
  });

  $("#debug-aligned").click(function () {
    debug.aligned = !debug.aligned;
  });
  $('#debug-aligned').removeAttr('checked');
  debug.aligned = false;

  $("#debug-end-points").click(function () {
    debug.endPoints = !debug.endPoints;
  });

  $("#debug-velocity").click(function () {
    debug.velocity = !debug.velocity;
  });
  $('#debug-velocity').removeAttr('checked');
  debug.velocity = false;

  $("#debug-acceleration").click(function () {
    debug.acceleration = !debug.acceleration;
  });
  $('#debug-acceleration').removeAttr('checked');
  debug.acceleration = false;

  $("#debug-path").click(function () {
    debug.path = !debug.path;
  });

  $("#go-to").click(function () {
    setGoalClick();
  });
}

function bindSidebarEvents() {
  // Bind mode slider.
  $("#mode-slider").click(toggleMode);
}

function bindButtonEvents(){
  document.addEventListener("keyup", setGoalClickKey, false);
  document.addEventListener("keyup", toggleModeKey, false);
}

function toggleModeKey(e){
  if (e.keyCode == 77) { // m
    toggleMode();
  }
}

function setGoalClickKey(e){
  if (e.keyCode == 71 && !isInAutoMode) { // g
    setGoalClick();
  }
}

function setGoalClick(e){
  isUsingGoTo = !isUsingGoTo;
  if (!isInAutoMode && goToPos) {
    $("#go-to").html("Go to");
    isUsingGoTo = false;
    goToPos = null;
    targetPositionGoal.cancel();
  } else if (!isInAutoMode && isUsingGoTo) {
    goToPos = null;
    $("#go-to").html("Set goal");
    $("#map").css('cursor', 'crosshair');
  }
  else {
    $("#map").css('cursor', 'default');
  }
}

function resizeCanvas() {
  var $container = $("#map-container");
  ctx.canvas.width = $container.width();
  ctx.canvas.height = $container.height();
}

// Toggle view state of map between global and local.
function toggleViewState() {
  var $viewStateElem = $("#view-state-button");
  if (view.state == "global") {
    $viewStateElem.html("Local");
    view.state = "local";
  } else {
    $viewStateElem.html("Global");
    view.state = "global";
    centerView();
  };
}

function centerView() {
  view.pan.x = 0;
  view.pan.y = 0;
  view.zoom = 1;
  view.rotation = 0;
}

function zoomIn() {
  view.zoom *= 1.2;
}

function zoomOut() {
  view.zoom *= 0.8;
}

function toggleMode() {
  var setAutoMode = new ROSLIB.ServiceRequest({
    data : !isInAutoMode
  isUsingGoTo = false;

  SetAutoModeClient.callService(setAutoMode, function(result) {});
}
