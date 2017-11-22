
// When the document has finished loading.
$(document).ready(function () {
  // Bind key, scroll, click events.
  bindEvents();

  // Initiate view state.
  toggleViewState();

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
}

function bindCanvasEvents() {
  // Bind map buttons.
  $("#zoom-in-button").click(zoomIn);
  $("#zoom-out-button").click(zoomOut);
  $("#view-state-button").click(toggleViewState);
  $("#center-view-button").click(centerView);

  // Bind map scoll zoom and mouse pan.
  $("#map")
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
  // TODO
}

function resizeCanvas() {
  var $container = $("#map-container");
  ctx.canvas.width = $container.width();
  ctx.canvas.height = $container.height();
}

// Toggle view state of map between global and local.
function toggleViewState() {
  var $viewStateElem = $("#view-state-button");
  if ($viewStateElem.html() === "Global") {
    $viewStateElem.html("Local");
  } else {
    $viewStateElem.html("Global");
    view.pan.x = 0;
    view.pan.y = 0;
  }
}

function updateView() {
  var $viewStateElem = $("#view-state-button");
  if ($viewStateElem.html() === "Local") {
    view.pan.x = -robot.position.x * view.zoom;
    view.pan.y = -robot.position.y * view.zoom;
  };
}

function centerView() {
  var $viewStateElem = $("#view-state-button");
  if ($viewStateElem.html() === "Global") {
    view.pan.x = 0;
    view.pan.y = 0;
  }
}

function zoomIn() {
  view.zoom *= 1.2;
}

function zoomOut() {
  view.zoom *= 0.8;
}
