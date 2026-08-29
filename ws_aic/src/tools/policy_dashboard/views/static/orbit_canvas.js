const clamp = (value, minimum, maximum) =>
  Math.min(maximum, Math.max(minimum, value));

export function createOrbitCanvas(
  canvas,
  { yaw = -0.75, pitch = 0.42, zoom = 1 } = {},
) {
  const initial = { yaw, pitch, zoom };
  const camera = { ...initial, dragging: false, x: 0, y: 0 };

  canvas.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    camera.dragging = true;
    camera.x = event.clientX;
    camera.y = event.clientY;
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!camera.dragging) return;
    camera.yaw += (event.clientX - camera.x) * 0.008;
    camera.pitch = clamp(
      camera.pitch + (event.clientY - camera.y) * 0.008,
      -1.45,
      1.45,
    );
    camera.x = event.clientX;
    camera.y = event.clientY;
  });
  const stopDragging = (event) => {
    camera.dragging = false;
    if (canvas.hasPointerCapture(event.pointerId)) {
      canvas.releasePointerCapture(event.pointerId);
    }
  };
  canvas.addEventListener("pointerup", stopDragging);
  canvas.addEventListener("pointercancel", stopDragging);
  canvas.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      camera.zoom = clamp(camera.zoom * Math.exp(-event.deltaY * 0.001), 0.35, 5);
    },
    { passive: false },
  );
  canvas.addEventListener("dblclick", () => {
    Object.assign(camera, initial);
  });

  return {
    project(point, width, height, scale, origin = [0, 0, 0]) {
      const x = point[0] - origin[0];
      const y = point[1] - origin[1];
      const z = point[2] - origin[2];
      const cosYaw = Math.cos(camera.yaw);
      const sinYaw = Math.sin(camera.yaw);
      const cosPitch = Math.cos(camera.pitch);
      const sinPitch = Math.sin(camera.pitch);
      const horizontal = x * cosYaw - y * sinYaw;
      const depthPlane = x * sinYaw + y * cosYaw;
      const vertical = z * cosPitch - depthPlane * sinPitch;
      const depth = z * sinPitch + depthPlane * cosPitch;
      const zoomedScale = scale * camera.zoom;
      return [
        width / 2 + horizontal * zoomedScale,
        height / 2 - vertical * zoomedScale,
        depth,
      ];
    },
  };
}
