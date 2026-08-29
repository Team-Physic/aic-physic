const historyWindowMs = 15_000;
const axisColors = ["#ff7085", "#a695ff", "#5fa8ff"];

function statusLabel(status) {
  return status ? status.charAt(0).toUpperCase() + status.slice(1) : "Waiting";
}

const state = {
  history: [],
  lastSequence: 0,
};

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

export function updateHaptic(wrench) {
  const status = document.querySelector("#haptic-status");
  status.dataset.status = wrench.status;
  status.textContent = statusLabel(wrench.status);
  document.querySelector("#wrench-frame").textContent =
    wrench.frame_id ? `${wrench.topic} · ${wrench.frame_id}` : wrench.topic;

  // Keep the payload bias visible in the raw headline values. The force
  // chart stays raw; chart_torque is filtered at the ROS sensor rate.
  if (wrench.sequence > 0) {
    document.querySelector("#force-magnitude").textContent =
      wrench.force_magnitude.toFixed(2);
    document.querySelector("#torque-magnitude").textContent =
      wrench.torque_magnitude.toFixed(3);
  }
  if (wrench.sequence === 0 || wrench.sequence === state.lastSequence) return;

  const now = performance.now();
  state.lastSequence = wrench.sequence;
  state.history.push({
    time: now,
    force: wrench.force.map(Number),
    torque: (wrench.chart_torque || wrench.torque).map(Number),
  });
}

function niceLimit(value) {
  if (!Number.isFinite(value) || value <= 0) return 1;
  const power = 10 ** Math.floor(Math.log10(value));
  return Math.ceil((value / power) * 2) / 2 * power;
}

function drawPlot(context, samples, key, bounds, title, unit, minimumLimit) {
  const { left, top, width, height, now } = bounds;
  const values = samples.flatMap((sample) => sample[key]);
  const limit = niceLimit(Math.max(minimumLimit, ...values.map(Math.abs)) * 1.08);
  context.save();
  context.strokeStyle = "rgba(148, 163, 184, 0.12)";
  context.fillStyle = "#858a99";
  context.font = "9px Inter, system-ui, sans-serif";
  [-1, 0, 1].forEach((fraction) => {
    const y = top + height / 2 - fraction * height * 0.42;
    context.beginPath();
    context.moveTo(left, y);
    context.lineTo(left + width, y);
    context.stroke();
    if (fraction !== 0) {
      context.fillText(`${(fraction * limit).toFixed(limit < 1 ? 2 : 1)}`, 3, y + 3);
    }
  });
  context.fillStyle = "#c7cad4";
  context.font = "600 9px Inter, system-ui, sans-serif";
  context.fillText(`${title} · ${unit}`, left, top + 10);

  axisColors.forEach((color, axis) => {
    context.beginPath();
    samples.forEach((sample, index) => {
      const x = left + ((sample.time - (now - historyWindowMs)) / historyWindowMs) * width;
      const y = top + height / 2 - (sample[key][axis] / limit) * height * 0.42;
      if (index === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.strokeStyle = color;
    context.lineWidth = 1.5;
    context.stroke();
  });
  context.restore();
}

export function renderHapticChart() {
  const canvas = document.querySelector("#haptic-canvas");
  const { context, width, height } = fitCanvas(canvas);
  context.clearRect(0, 0, width, height);
  const now = performance.now();
  state.history = state.history.filter(
    (sample) => sample.time >= now - historyWindowMs,
  );

  const left = 42;
  const right = 12;
  const plotWidth = Math.max(1, width - left - right);
  const gap = 15;
  const bottom = 20;
  const plotHeight = Math.max(55, (height - bottom - gap) / 2);
  const common = { left, width: plotWidth, height: plotHeight, now };
  drawPlot(context, state.history, "force", { ...common, top: 0 }, "Force", "N", 1);
  drawPlot(
    context,
    state.history,
    "torque",
    { ...common, top: plotHeight + gap },
    "Torque",
    "Nm",
    0.1,
  );

  context.fillStyle = "#666b79";
  context.font = "9px Inter, system-ui, sans-serif";
  [15, 10, 5, 0].forEach((seconds) => {
    const x = left + ((15 - seconds) / 15) * plotWidth;
    context.fillText(seconds === 0 ? "Now" : `-${seconds}s`, x - 10, height - 3);
  });
  if (!state.history.length) {
    context.fillStyle = "#858a99";
    context.font = "10px Inter, system-ui, sans-serif";
    context.textAlign = "center";
    context.fillText("Waiting for force and torque data", width / 2, height / 2);
    context.textAlign = "start";
  }
}
