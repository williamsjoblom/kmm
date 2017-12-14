
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

      targetPositionGoal = new ROSLIB.Goal({
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
  $("#debug-global-frame").click(function () {
    debug.globalFrame = !debug.globalFrame;
  });
  $('#debug-global-frame').removeAttr('checked');
  debug.globalFrame = false;

  $("#debug-scan").click(function () {
    debug.scan = !debug.scan;
  });

  $("#debug-walls").click(function () {
    debug.walls = !debug.walls;
  });

  $("#debug-position-scan").click(function () {
    debug.positionScan = !debug.positionScan;
  });

  $("#debug-mapping-scan").click(function () {
    debug.mappingScan = !debug.mappingScan;
  });

  $("#debug-end-points").click(function () {
    debug.endPoints = !debug.endPoints;
  });

  $("#debug-velocity").click(function () {
    debug.velocity = !debug.velocity;
  });

  $("#debug-path").click(function () {
    debug.path = !debug.path;
  });

  $("#go-to").click(function () {
    setGoalClick();
  });
}

function bindSidebarEvents() {
  $("#mode-slider").click(toggleMode);
  $("#mapping-slider").click(toggleMapping);
  $("#reset-position-button").click(resetPosition);
  $("#reset-map-button").click(resetMap);
  $("#hide-button").click(hideOrShowSidebar);
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
  if (e.keyCode == 71) { // g
    setGoalClick();
  }
}

function setGoalClick(e){
  isUsingGoTo = !isUsingGoTo;
  if (!isInAutoMode && !goToPos && isUsingGoTo) { // Set new
    $("#go-to").html("Set goal");
    $("#map").css('cursor', 'crosshair');

  } else { // Cancel current
    isUsingGoTo = false;
    goToPos = null;

    $("#go-to").html("Go to");
    $("#map").css('cursor', 'default');

    if (targetPositionGoal) {
      targetPositionGoal.cancel();
    }
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
  });
  setAutoModeClient.callService(setAutoMode, function(result) {});
}

function toggleMapping() {
  var setMapping = new ROSLIB.ServiceRequest({
    data : !mapping
  });
  setMappingClient.callService(setMapping, function(result) {});
}

function resetPosition() {
  var resetPosition = new ROSLIB.ServiceRequest({
    data : true
  });
  resetPositionClient.callService(resetPosition, function(result) {});
}

function resetMap() {
  var resetMap = new ROSLIB.ServiceRequest({
    data : true
  });
  resetMapClient.callService(resetMap, function(result) {});
}

function hideOrShowSidebar() {
  var $showHideElem = $("#hide-button");
  if(view.sidebarState) {
    view.sidebarState = false;
    $showHideElem.html("&lt;&lt;");
  } else {
    view.sidebarState = true;
    $showHideElem.html("&gt;&gt;");
  }
}
