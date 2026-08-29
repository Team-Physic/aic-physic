import { createOrbitCanvas } from "./orbit_canvas.js";

const colors = {
  port: "#ff6f91",
  triangulated_port: "#a695ff",
  ee: "#5fa8ff",
  cable: "#f0a35e",
};
const maxTracePoints = 900;
const maxStoredTrials = 24;
const viewerState = {
  trials: new Map(),
  currentTrialIndex: null,
  selectedTrialIndex: null,
  tabsSignature: "",
  orbit: null,
};

function fitCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.floor(rect.width));
  const height = Math.max(1, Math.floor(rect.height));
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  if (canvas.width !== Math.floor(width * ratio) || canvas.height !== Math.floor(height * ratio)) {
    canvas.width = Math.floor(width * ratio);
    canvas.height = Math.floor(height * ratio);
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { context, width, height };
}

function formatPosition(value) {
  if (!value?.xyz) return "—";
  return value.xyz.map((axis) => Number(axis).toFixed(3)).join("  ");
}

function appendTrace(trial, source, value) {
  if (!value?.xyz || value.sequence === trial.sequences[source]) return;
  trial.sequences[source] = value.sequence;
  const trace = trial.traces[source];
  const point = value.xyz.map(Number);
  const previous = trace.at(-1);
  if (!previous || Math.hypot(...point.map((axis, index) => axis - previous[index])) > 1e-5) {
    trace.push(point);
    if (trace.length > maxTracePoints) trace.splice(0, trace.length - maxTracePoints);
  }
}

function selectedTrial() {
  return viewerState.trials.get(viewerState.selectedTrialIndex) || null;
}

function updateReadouts(spatial) {
  ["port", "triangulated_port", "ee", "cable"].forEach((source) => {
    const value = spatial?.[source];
    const readout = document.querySelector(`[data-spatial="${source}"]`);
    readout.dataset.status = value?.status || "waiting";
    readout.querySelector("b").textContent = formatPosition(value);
    readout.querySelector("small").textContent = value?.frame_id || "Waiting";
  });
}

function renderTrialTabs() {
  const signature = [
    viewerState.currentTrialIndex,
    viewerState.selectedTrialIndex,
    ...viewerState.trials.keys(),
  ].join(":");
  if (signature === viewerState.tabsSignature) return;
  viewerState.tabsSignature = signature;
  const tabs = document.querySelector("#trial-tabs");
  tabs.replaceChildren();
  viewerState.trials.forEach((trial, trialIndex) => {
    const button = document.createElement("button");
    const isCurrent = trialIndex === viewerState.currentTrialIndex;
    button.type = "button";
    button.className = "trial-tab";
    button.dataset.active = String(trialIndex === viewerState.selectedTrialIndex);
    button.dataset.current = String(isCurrent);
    button.setAttribute("aria-selected", String(trialIndex === viewerState.selectedTrialIndex));
    button.title = `${trial.taskId}${isCurrent ? " · Current trial" : ""}`;
    button.textContent = `Trial ${trialIndex}`;
    button.addEventListener("click", () => {
      viewerState.selectedTrialIndex = trialIndex;
      viewerState.tabsSignature = "";
      updateReadouts(trial.spatial);
      renderTrialTabs();
    });
    tabs.append(button);
  });
}

function pruneTrials() {
  while (viewerState.trials.size > maxStoredTrials) {
    const oldest = viewerState.trials.keys().next().value;
    viewerState.trials.delete(oldest);
    if (viewerState.selectedTrialIndex === oldest) {
      viewerState.selectedTrialIndex = viewerState.currentTrialIndex;
    }
  }
}

export function updateCoordinateViewer(spatial) {
  const trialIndex = Number(spatial?.task?.trial_index);
  if (!Number.isInteger(trialIndex) || trialIndex < 1) {
    if (!selectedTrial()) updateReadouts(null);
    return;
  }

  const previousCurrent = viewerState.currentTrialIndex;
  let trial = viewerState.trials.get(trialIndex);
  if (!trial) {
    trial = {
      taskId: spatial.task.id || "Unknown task",
      spatial: null,
      traces: { ee: [], cable: [] },
      sequences: { ee: 0, cable: 0 },
    };
    viewerState.trials.set(trialIndex, trial);
    if (
      viewerState.selectedTrialIndex === null
      || viewerState.selectedTrialIndex === previousCurrent
    ) {
      viewerState.selectedTrialIndex = trialIndex;
    }
  }
  viewerState.currentTrialIndex = trialIndex;
  trial.spatial = spatial;
  appendTrace(trial, "ee", spatial.ee);
  appendTrace(trial, "cable", spatial.cable);
  pruneTrials();
  renderTrialTabs();
  updateReadouts(selectedTrial()?.spatial);
}

function drawLine(context, points, color, width = 1, dash = []) {
  if (points.length < 2) return;
  context.beginPath();
  context.moveTo(points[0][0], points[0][1]);
  points.slice(1).forEach((point) => context.lineTo(point[0], point[1]));
  context.strokeStyle = color;
  context.lineWidth = width;
  context.setLineDash(dash);
  context.stroke();
  context.setLineDash([]);
}

function drawMarker(context, projected, color, label, shape = "circle") {
  const [x, y] = projected;
  context.save();
  context.strokeStyle = color;
  context.fillStyle = color;
  context.lineWidth = 2;
  context.beginPath();
  if (shape === "diamond") {
    context.moveTo(x, y - 6);
    context.lineTo(x + 6, y);
    context.lineTo(x, y + 6);
    context.lineTo(x - 6, y);
    context.closePath();
    context.stroke();
  } else if (shape === "ring") {
    context.arc(x, y, 6, 0, Math.PI * 2);
    context.stroke();
    context.beginPath();
    context.arc(x, y, 2, 0, Math.PI * 2);
    context.fill();
  } else {
    context.arc(x, y, 4, 0, Math.PI * 2);
    context.fill();
  }
  context.font = "600 10px Inter, system-ui, sans-serif";
  context.fillText(label, x + 9, y - 8);
  context.restore();
}

export function renderCoordinateViewer() {
  const canvas = document.querySelector("#coordinate-canvas");
  if (!viewerState.orbit) viewerState.orbit = createOrbitCanvas(canvas);
  const { context, width, height } = fitCanvas(canvas);
  context.clearRect(0, 0, width, height);

  const trial = selectedTrial();
  const spatial = trial?.spatial;
  const traces = trial?.traces || { ee: [], cable: [] };
  const current = {
    port: spatial?.port?.xyz,
    triangulated_port: spatial?.triangulated_port?.xyz
      || (spatial?.triangulated_port?.x !== undefined
        ? [spatial.triangulated_port.x, spatial.triangulated_port.y, spatial.triangulated_port.z]
        : null),
    ee: spatial?.ee?.xyz,
    cable: spatial?.cable?.xyz,
  };
  const points = [
    ...Object.values(current).filter(Boolean),
    ...traces.ee,
    ...traces.cable,
  ];
  if (!points.length) {
    context.fillStyle = "#777d8d";
    context.font = "11px Inter, system-ui, sans-serif";
    context.textAlign = "center";
    context.fillText("Waiting for position data", width / 2, height / 2);
    context.textAlign = "start";
    return;
  }

  const minimum = [0, 1, 2].map((axis) => Math.min(...points.map((point) => point[axis])));
  const maximum = [0, 1, 2].map((axis) => Math.max(...points.map((point) => point[axis])));
  const origin = minimum.map((value, axis) => (value + maximum[axis]) / 2);
  const extent = Math.max(0.04, ...maximum.map((value, axis) => value - minimum[axis]));
  const scale = Math.min(width, height) * 0.68 / extent;
  const project = (point) => viewerState.orbit.project(point, width, height, scale, origin);

  const gridRadius = extent * 0.75;
  const gridZ = minimum[2];
  for (let index = -5; index <= 5; index += 1) {
    const offset = gridRadius * index / 5;
    drawLine(
      context,
      [
        project([origin[0] - gridRadius, origin[1] + offset, gridZ]),
        project([origin[0] + gridRadius, origin[1] + offset, gridZ]),
      ],
      "rgba(148, 163, 184, 0.11)",
    );
    drawLine(
      context,
      [
        project([origin[0] + offset, origin[1] - gridRadius, gridZ]),
        project([origin[0] + offset, origin[1] + gridRadius, gridZ]),
      ],
      "rgba(148, 163, 184, 0.11)",
    );
  }

  const axisLength = extent * 0.55;
  const axes = [
    [[origin[0], origin[1], gridZ], [origin[0] + axisLength, origin[1], gridZ], "X", "#ff7085"],
    [[origin[0], origin[1], gridZ], [origin[0], origin[1] + axisLength, gridZ], "Y", "#a695ff"],
    [[origin[0], origin[1], gridZ], [origin[0], origin[1], gridZ + axisLength], "Z", "#5fa8ff"],
  ];
  axes.forEach(([from, to, label, color]) => {
    const line = [project(from), project(to)];
    drawLine(context, line, color, 1.3);
    context.fillStyle = color;
    context.font = "600 9px Inter, system-ui, sans-serif";
    context.fillText(label, line[1][0] + 4, line[1][1] - 3);
  });

  drawLine(context, traces.ee.map(project), `${colors.ee}99`, 2);
  drawLine(context, traces.cable.map(project), `${colors.cable}99`, 2);

  if (current.port) drawMarker(context, project(current.port), colors.port, "Port", "diamond");
  if (current.triangulated_port) {
    drawMarker(context, project(current.triangulated_port), colors.triangulated_port, "Triangulated", "ring");
  }
  if (current.ee) drawMarker(context, project(current.ee), colors.ee, "EE");
  if (current.cable) drawMarker(context, project(current.cable), colors.cable, "Cable");
}
