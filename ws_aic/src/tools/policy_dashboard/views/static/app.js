import { renderHapticChart, updateHaptic } from "./haptic.js";
import { renderRpySphere, updateRpySphere } from "./rpy_sphere.js";
import {
  renderCoordinateViewer,
  updateCoordinateViewer,
} from "./coordinate_viewer.js";

const cameras = ["left", "center", "right"];

function updateCamera(name, camera) {
  const card = document.querySelector(`[data-camera="${name}"]`);
  card.dataset.status = camera.status;
  card.querySelector(".fps").textContent = camera.sequence > 0
    ? `${camera.fps.toFixed(1)} FPS`
    : "— FPS";
}

function updateOverall(cameraStates) {
  const overall = document.querySelector("#overall-status");
  const statuses = cameraStates.map((camera) => camera.status);
  const liveCount = statuses.filter((status) => status === "live").length;
  const hasFrames = statuses.some((status) => status !== "waiting");
  if (liveCount > 0) {
    overall.dataset.status = "live";
    overall.querySelector("strong").textContent =
      `${liveCount}/3 camera${liveCount === 1 ? "" : "s"} live`;
    overall.querySelector("small").textContent = "FinalPolicy connected";
  } else if (hasFrames) {
    overall.dataset.status = "stale";
    overall.querySelector("strong").textContent = "Image stream stale";
    overall.querySelector("small").textContent = "Waiting for new policy output";
  } else {
    overall.dataset.status = "waiting";
    overall.querySelector("strong").textContent = "Waiting for FinalPolicy";
    overall.querySelector("small").textContent = "ROS debug topics";
  }
}

function renderLoop() {
  renderRpySphere();
  renderHapticChart();
  renderCoordinateViewer();
  requestAnimationFrame(renderLoop);
}

async function refresh() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const state = await response.json();
    const cameraStates = cameras.map((name) => state.cameras[name]);
    cameras.forEach((name) => updateCamera(name, state.cameras[name]));
    updateOverall(cameraStates);
    updateRpySphere(state.orientation);
    updateHaptic(state.wrench);
    updateCoordinateViewer(state.spatial);
    document.querySelector("#last-update").textContent =
      `Updated ${new Date().toLocaleTimeString()}`;
  } catch (error) {
    const overall = document.querySelector("#overall-status");
    overall.dataset.status = "stale";
    overall.querySelector("strong").textContent = "Dashboard disconnected";
    overall.querySelector("small").textContent = error.message;
  } finally {
    window.setTimeout(refresh, 100);
  }
}

refresh();
requestAnimationFrame(renderLoop);
