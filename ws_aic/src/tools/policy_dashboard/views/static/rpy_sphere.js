import { createOrbitCanvas } from "./orbit_canvas.js";

const axisColors = ["#ff7085", "#a695ff", "#5fa8ff"];
const orientationColors = { ee: "#5fa8ff", cable: "#f0a35e" };
let orientationState = {};
let orbit = null;

function statusLabel(status) {
  return status ? status.charAt(0).toUpperCase() + status.slice(1) : "Waiting";
}

function signed(value, digits = 1) {
  return Number(value).toLocaleString(undefined, {
    minimumFractionDigits: digits,
    maximumFractionDigits: digits,
    signDisplay: "always",
  });
}

function fitCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.floor(rect.width));
  const height = Math.max(1, Math.floor(rect.height));
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const pixelWidth = Math.floor(width * ratio);
  const pixelHeight = Math.floor(height * ratio);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { context, width, height };
}

function drawPolyline(context, points, color, width = 1, dash = []) {
  if (!points.length) return;
  context.beginPath();
  context.moveTo(points[0][0], points[0][1]);
  points.slice(1).forEach((point) => context.lineTo(point[0], point[1]));
  context.strokeStyle = color;
  context.lineWidth = width;
  context.setLineDash(dash);
  context.stroke();
  context.setLineDash([]);
}

function circlePoints(plane, offset = 0, radius = 1) {
  const points = [];
  for (let step = 0; step <= 80; step += 1) {
    const angle = (step / 80) * Math.PI * 2;
    const a = Math.cos(angle) * radius;
    const b = Math.sin(angle) * radius;
    if (plane === "xy") points.push([a, b, offset]);
    if (plane === "xz") points.push([a, offset, b]);
    if (plane === "yz") points.push([offset, a, b]);
  }
  return points;
}

function drawArrow(context, start, end, color, label) {
  const angle = Math.atan2(end[1] - start[1], end[0] - start[0]);
  context.save();
  context.beginPath();
  context.moveTo(start[0], start[1]);
  context.lineTo(end[0], end[1]);
  context.strokeStyle = color;
  context.lineWidth = 3;
  context.stroke();
  context.beginPath();
  context.moveTo(end[0], end[1]);
  context.lineTo(
    end[0] - 11 * Math.cos(angle - Math.PI / 6),
    end[1] - 11 * Math.sin(angle - Math.PI / 6),
  );
  context.lineTo(
    end[0] - 11 * Math.cos(angle + Math.PI / 6),
    end[1] - 11 * Math.sin(angle + Math.PI / 6),
  );
  context.closePath();
  context.fillStyle = color;
  context.fill();
  context.font = "600 10px Inter, system-ui, sans-serif";
  context.fillText(label, end[0] + 8, end[1] - 8);
  context.restore();
}

export function updateRpySphere(orientation) {
  orientationState = orientation || {};
  ["ee", "cable"].forEach((source) => {
    const value = orientationState[source] || { status: "waiting" };
    const readout = document.querySelector(`[data-orientation="${source}"]`);
    readout.dataset.status = value.status;
    readout.querySelector("em").textContent = statusLabel(value.status);
    readout.querySelector(".orientation-frame").textContent = value.frame_id || "—";
    const rpy = readout.querySelectorAll(".rpy-values b");
    if (value.rpy_degrees) {
      value.rpy_degrees.forEach((angle, index) => {
        rpy[index].textContent = signed(angle);
      });
    } else {
      rpy.forEach((element) => { element.textContent = "—"; });
    }
  });
}

export function renderRpySphere() {
  const canvas = document.querySelector("#orientation-canvas");
  if (!orbit) orbit = createOrbitCanvas(canvas);
  const { context, width, height } = fitCanvas(canvas);
  context.clearRect(0, 0, width, height);
  const scale = Math.min(width, height) * 0.35;
  const project = (point) => orbit.project(point, width, height, scale);

  ["xy", "xz", "yz"].forEach((plane) => {
    drawPolyline(
      context,
      circlePoints(plane).map(project),
      "rgba(148, 163, 184, 0.24)",
    );
  });
  [-0.5, 0.5].forEach((z) => {
    const radius = Math.sqrt(1 - z * z);
    drawPolyline(
      context,
      circlePoints("xy", z, radius).map(project),
      "rgba(148, 163, 184, 0.11)",
    );
  });

  const axes = [
    { from: [-1.15, 0, 0], to: [1.15, 0, 0], label: "X", color: axisColors[0] },
    { from: [0, -1.15, 0], to: [0, 1.15, 0], label: "Y", color: axisColors[1] },
    { from: [0, 0, -1.15], to: [0, 0, 1.15], label: "Z", color: axisColors[2] },
  ];
  axes.forEach((axis) => {
    const from = project(axis.from);
    const to = project(axis.to);
    drawPolyline(context, [from, to], `${axis.color}88`, 1, [3, 4]);
    context.fillStyle = axis.color;
    context.font = "600 9px Inter, system-ui, sans-serif";
    context.fillText(axis.label, to[0] + 4, to[1] - 3);
  });

  ["ee", "cable"].forEach((source) => {
    const value = orientationState[source];
    if (!value || !value.direction) return;
    const endpoint = value.direction.map((component) => component * 1.05);
    drawArrow(
      context,
      project([0, 0, 0]),
      project(endpoint),
      orientationColors[source],
      source === "ee" ? "EE" : "CABLE",
    );
  });
}
