"use strict"; // Browser-side Model / View / Controller.

const KEYPOINT_COLORS = ["#ec7182", "#e5a665", "#aa9af4", "#62a8f7"];
const CLASS_COLORS = ["#8291ff", "#e5a665", "#ec7182", "#62a8f7", "#aa9af4", "#65c4d7", "#d58bdd", "#8dbd78"];
const MIN_BOX_SIZE = 0.00001;
const clamp = (value, low = 0, high = 1) => Math.min(Math.max(value, low), high);
const deepClone = (value) => JSON.parse(JSON.stringify(value));
const annotationKey = (annotations) => JSON.stringify(annotations);

function classColor(classId) {
  return CLASS_COLORS[Math.abs(Number(classId) || 0) % CLASS_COLORS.length];
}

function bboxEdges(annotation) {
  const [cx, cy, width, height] = annotation.bbox;
  return [cx - width / 2, cy - height / 2, cx + width / 2, cy + height / 2];
}

function moveAnnotation(annotation, dx, dy) {
  const moved = deepClone(annotation);
  const [left, top, right, bottom] = bboxEdges(annotation);
  const xs = [left, right, ...annotation.keypoints.map((point) => point[0])];
  const ys = [top, bottom, ...annotation.keypoints.map((point) => point[1])];
  dx = clamp(dx, -Math.min(...xs), 1 - Math.max(...xs));
  dy = clamp(dy, -Math.min(...ys), 1 - Math.max(...ys));
  moved.bbox[0] += dx;
  moved.bbox[1] += dy;
  moved.keypoints.forEach((point) => { point[0] += dx; point[1] += dy; });
  return moved;
}

function moveSelection(annotations, annotationIndices, keypointKeys, dx, dy) {
  const xs = [];
  const ys = [];
  annotations.forEach((annotation, annotationIndex) => {
    if (annotationIndices.has(annotationIndex)) {
      const [left, top, right, bottom] = bboxEdges(annotation);
      xs.push(left, right, ...annotation.keypoints.map((point) => point[0]));
      ys.push(top, bottom, ...annotation.keypoints.map((point) => point[1]));
    } else {
      annotation.keypoints.forEach((point, keypointIndex) => {
        if (keypointKeys.has(`${annotationIndex}:${keypointIndex}`)) {
          xs.push(point[0]); ys.push(point[1]);
        }
      });
    }
  });
  if (!xs.length) return deepClone(annotations);
  dx = clamp(dx, -Math.min(...xs), 1 - Math.max(...xs));
  dy = clamp(dy, -Math.min(...ys), 1 - Math.max(...ys));
  const result = deepClone(annotations);
  annotations.forEach((annotation, annotationIndex) => {
    if (annotationIndices.has(annotationIndex)) {
      result[annotationIndex] = moveAnnotation(annotation, dx, dy);
    } else {
      result[annotationIndex].keypoints.forEach((point, keypointIndex) => {
        if (keypointKeys.has(`${annotationIndex}:${keypointIndex}`)) {
          point[0] += dx; point[1] += dy;
        }
      });
    }
  });
  return result;
}

function resizeAnnotation(annotation, corner, targetX, targetY) {
  const result = deepClone(annotation);
  const [oldLeft, oldTop, oldRight, oldBottom] = bboxEdges(annotation);
  let [left, top, right, bottom] = [oldLeft, oldTop, oldRight, oldBottom];
  targetX = clamp(targetX); targetY = clamp(targetY);
  if (corner === 0 || corner === 3) left = Math.min(targetX, right - MIN_BOX_SIZE);
  else right = Math.max(targetX, left + MIN_BOX_SIZE);
  if (corner === 0 || corner === 1) top = Math.min(targetY, bottom - MIN_BOX_SIZE);
  else bottom = Math.max(targetY, top + MIN_BOX_SIZE);
  const oldWidth = Math.max(oldRight - oldLeft, MIN_BOX_SIZE);
  const oldHeight = Math.max(oldBottom - oldTop, MIN_BOX_SIZE);
  const width = right - left;
  const height = bottom - top;
  result.bbox = [(left + right) / 2, (top + bottom) / 2, width, height];
  result.keypoints = annotation.keypoints.map(([x, y, visibility]) => [
    clamp(left + ((x - oldLeft) / oldWidth) * width),
    clamp(top + ((y - oldTop) / oldHeight) * height),
    visibility,
  ]);
  return result;
}

class ApiClient {
  async request(url, options = {}) {
    const response = await fetch(url, {
      ...options,
      headers: {"Content-Type": "application/json", ...(options.headers || {})},
    });
    const contentType = response.headers.get("content-type") || "";
    const payload = contentType.includes("application/json") ? await response.json() : null;
    if (!response.ok) {
      const error = new Error(payload?.error || `${response.status} ${response.statusText}`);
      error.status = response.status;
      throw error;
    }
    return payload;
  }
  state() { return this.request("/api/state"); }
  open(path) { return this.request("/api/datasets/open", {method: "POST", body: JSON.stringify({path})}); }
  image(index) { return this.request(`/api/images/${index}`); }
  save(index, revision, annotations) {
    return this.request(`/api/images/${index}/annotations`, {
      method: "PUT", body: JSON.stringify({revision, annotations}),
    });
  }
  autoVisibility(index, annotations) {
    return this.request(`/api/images/${index}/auto-visibility`, {
      method: "POST", body: JSON.stringify({annotations}),
    });
  }
  deleteImage(index) { return this.request(`/api/images/${index}`, {method: "DELETE"}); }
}

// Model: browser-side working copy and undo history.
class AnnotationModel {
  constructor() {
    this.dataset = {open: false, generation: 0, images: [], class_names: []};
    this.currentIndex = -1;
    this.imageMeta = null;
    this.image = null;
    this.annotations = [];
    this.savedAnnotations = [];
    this.revision = "missing";
    this.selectedIndex = -1;
    this.selectedAnnotations = new Set();
    this.selectedKeypoints = new Set();
    this.undoStack = [];
    this.overlayVisible = true;
    this.addMode = false;
    this.listeners = [];
  }
  subscribe(listener) { this.listeners.push(listener); }
  notify(scope = "all") { this.listeners.forEach((listener) => listener(scope)); }
  get dirty() { return annotationKey(this.annotations) !== annotationKey(this.savedAnnotations); }
  get editable() { return Boolean(this.imageMeta?.editable); }
  beforeMutation() {
    return {
      annotations: deepClone(this.annotations),
      selectedIndex: this.selectedIndex,
      selectedAnnotations: [...this.selectedAnnotations],
      selectedKeypoints: [...this.selectedKeypoints],
    };
  }
  commitMutation(before) {
    if (annotationKey(before.annotations) === annotationKey(this.annotations)) return false;
    this.undoStack.push(before);
    if (this.undoStack.length > 100) this.undoStack.shift();
    this.notify();
    return true;
  }
  mutate(callback) {
    const before = this.beforeMutation();
    callback();
    return this.commitMutation(before);
  }
  undo() {
    const snapshot = this.undoStack.pop();
    if (!snapshot) return;
    this.annotations = snapshot.annotations;
    this.selectedIndex = snapshot.selectedIndex;
    this.selectedAnnotations = new Set(snapshot.selectedAnnotations);
    this.selectedKeypoints = new Set(snapshot.selectedKeypoints);
    this.notify();
  }
  setDataset(dataset) {
    this.dataset = {images: [], class_names: [], ...dataset};
    this.clearImage();
    this.notify();
  }
  clearImage() {
    this.currentIndex = -1;
    this.imageMeta = null;
    this.image = null;
    this.annotations = [];
    this.savedAnnotations = [];
    this.selectedIndex = -1;
    this.selectedAnnotations.clear();
    this.selectedKeypoints.clear();
    this.undoStack = [];
    this.addMode = false;
  }
  setImage(index, metadata, image) {
    this.currentIndex = index;
    this.imageMeta = metadata;
    this.image = image;
    this.annotations = deepClone(metadata.annotations);
    this.savedAnnotations = deepClone(metadata.annotations);
    this.revision = metadata.revision;
    this.selectedIndex = -1;
    this.selectedAnnotations.clear();
    this.selectedKeypoints.clear();
    this.undoStack = [];
    this.addMode = false;
    this.notify();
  }
  markSaved(result) {
    this.annotations = deepClone(result.annotations);
    this.savedAnnotations = deepClone(result.annotations);
    this.revision = result.revision;
    this.notify();
  }
  discardChanges() {
    this.annotations = deepClone(this.savedAnnotations);
    this.undoStack = [];
    this.selectedIndex = -1;
    this.selectedAnnotations.clear();
    this.selectedKeypoints.clear();
    this.notify();
  }
  labelFor(classId) {
    return this.dataset.class_names?.find((item) => item.id === classId)?.label || `class_${classId}`;
  }
}

// View: DOM rendering and all canvas coordinate/drawing concerns.
class EditorView {
  constructor(model) {
    this.model = model;
    this.canvas = document.querySelector("#editor-canvas");
    this.canvasWrap = document.querySelector("#canvas-wrap");
    this.ctx = this.canvas.getContext("2d");
    this.zoom = 1;
    this.panX = 0;
    this.panY = 0;
    this.previewBox = null;
    this.selectionRect = null;
    this.lastDatasetGeneration = null;
    this.lastImageQuery = null;
    this.lastActiveIndex = -1;
    this.elements = Object.fromEntries([
      "dataset-path", "save-state", "save-button", "undo-button", "add-button",
      "delete-button", "delete-sample-button", "visibility-button", "previous-button",
      "next-button", "fit-button", "zoom-in-button", "zoom-out-button", "overlay-toggle",
      "image-position", "image-count", "image-search", "image-list", "dataset-empty",
      "canvas-empty", "busy", "status-message", "image-meta", "object-count", "object-list",
      "inspector", "selected-object", "class-select", "bbox-values", "keypoint-controls",
      "warning-count", "warnings", "class-dialog", "dialog-class-select", "custom-class-wrap",
      "custom-class-id", "toast-region",
    ].map((id) => [id, document.getElementById(id)]));
    new ResizeObserver(() => this.renderCanvas()).observe(this.canvasWrap);
  }
  render() {
    this.renderDataset();
    this.renderObjects();
    this.renderInspector();
    this.renderWarnings();
    this.renderActions();
    this.renderCanvas();
  }
  renderDataset(force = false) {
    const dataset = this.model.dataset;
    const query = this.elements["image-search"].value.trim().toLowerCase();
    const rebuild = force || this.lastDatasetGeneration !== dataset.generation || this.lastImageQuery !== query;
    if (this.lastDatasetGeneration !== dataset.generation) {
      this.lastDatasetGeneration = dataset.generation;
      this.elements["dataset-path"].value = dataset.open ? dataset.selected_path : "";
      this.elements["image-count"].textContent = dataset.images?.length || 0;
      this.elements["dataset-empty"].hidden = Boolean(dataset.open);
    }
    if (!rebuild && this.lastActiveIndex === this.model.currentIndex) return;
    if (!rebuild) {
      this.elements["image-list"].querySelector("li.active")?.classList.remove("active");
      const active = this.elements["image-list"].querySelector(`li[data-index="${this.model.currentIndex}"]`);
      active?.classList.add("active");
      active?.scrollIntoView({block: "nearest"});
      this.lastActiveIndex = this.model.currentIndex;
      return;
    }
    this.lastImageQuery = query;
    this.lastActiveIndex = this.model.currentIndex;
    const images = (dataset.images || []).filter((item) => item.display_path.toLowerCase().includes(query));
    this.elements["image-list"].replaceChildren(...images.map((item) => {
      const row = document.createElement("li");
      row.dataset.index = item.index;
      row.className = item.index === this.model.currentIndex ? "active" : "";
      row.title = item.display_path;
      const index = document.createElement("span"); index.className = "index"; index.textContent = String(item.index + 1).padStart(3, "0");
      const name = document.createElement("span"); name.className = "name"; name.textContent = item.display_path;
      row.append(index, name);
      if (!item.editable) { const lock = document.createElement("span"); lock.className = "lock"; lock.textContent = "READ"; row.append(lock); }
      return row;
    }));
  }
  renderObjects() {
    this.elements["object-count"].textContent = this.model.annotations.length;
    const selected = this.model.selectedAnnotations;
    this.elements["object-list"].replaceChildren(...this.model.annotations.map((annotation, index) => {
      const row = document.createElement("li");
      row.dataset.index = index;
      if (index === this.model.selectedIndex || selected.has(index)) row.className = "active";
      const dot = document.createElement("span"); dot.className = "class-dot"; dot.style.background = classColor(annotation.class_id);
      const name = document.createElement("span"); name.className = "name"; name.textContent = annotation.label;
      const meta = document.createElement("span"); meta.className = "object-meta"; meta.textContent = `#${annotation.class_id}`;
      row.append(dot, name, meta); return row;
    }));
  }
  classOptions(select, current) {
    const classes = this.model.dataset.class_names || [];
    select.replaceChildren();
    if (!classes.length) {
      const option = new Option(`class_${current ?? 0}`, String(current ?? 0)); select.add(option); return;
    }
    classes.forEach(({id, label}) => select.add(new Option(`${id}: ${label}`, String(id))));
    if (current !== undefined && !classes.some((item) => item.id === current)) select.add(new Option(`${current}: class_${current}`, String(current)));
    if (current !== undefined) select.value = String(current);
  }
  renderInspector() {
    const index = this.model.selectedIndex;
    const annotation = this.model.annotations[index];
    this.elements.inspector.hidden = !annotation;
    if (!annotation) return;
    this.elements["selected-object"].textContent = `Object ${index + 1}`;
    this.classOptions(this.elements["class-select"], annotation.class_id);
    const [x, y, width, height] = annotation.bbox;
    this.elements["bbox-values"].innerHTML = `<span>X <b>${x.toFixed(4)}</b></span><span>Y <b>${y.toFixed(4)}</b></span><span>W <b>${width.toFixed(4)}</b></span><span>H <b>${height.toFixed(4)}</b></span>`;
    this.elements["keypoint-controls"].replaceChildren(...annotation.keypoints.map((point, pointIndex) => {
      const row = document.createElement("div"); row.className = "keypoint-row";
      const dot = document.createElement("i"); dot.style.background = KEYPOINT_COLORS[pointIndex];
      const coords = document.createElement("span"); coords.textContent = `${pointIndex + 1}  ${point[0].toFixed(3)}, ${point[1].toFixed(3)}`;
      const select = document.createElement("select"); select.dataset.keypoint = pointIndex;
      [[0, "0 hidden"], [1, "1 occluded"], [2, "2 visible"]].forEach(([value, text]) => select.add(new Option(text, value)));
      select.value = String(point[2]); row.append(dot, coords, select); return row;
    }));
  }
  renderWarnings() {
    const warnings = this.model.imageMeta?.warnings || [];
    this.elements["warning-count"].textContent = warnings.length;
    this.elements.warnings.textContent = warnings.length ? warnings.join("\n") : "No warnings";
    this.elements.warnings.classList.toggle("has-warning", Boolean(warnings.length));
  }
  renderActions() {
    const hasImage = Boolean(this.model.image);
    const editable = this.model.editable;
    const hasSelection = this.model.selectedIndex >= 0 || this.model.selectedAnnotations.size > 0;
    this.elements["save-button"].disabled = !editable || !this.model.dirty;
    this.elements["undo-button"].disabled = !editable || !this.model.undoStack.length;
    this.elements["add-button"].disabled = !editable;
    this.elements["add-button"].classList.toggle("primary", this.model.addMode);
    this.elements["delete-button"].disabled = !editable || !hasSelection;
    this.elements["delete-sample-button"].disabled = !hasImage || !this.model.dataset.dataset_root;
    this.elements["visibility-button"].disabled = !editable || !this.model.annotations.length;
    this.elements["previous-button"].disabled = this.model.currentIndex <= 0;
    this.elements["next-button"].disabled = this.model.currentIndex < 0 || this.model.currentIndex >= this.model.dataset.images.length - 1;
    this.elements["fit-button"].disabled = !hasImage;
    this.elements["zoom-in-button"].disabled = !hasImage;
    this.elements["zoom-out-button"].disabled = !hasImage;
    const total = this.model.dataset.images?.length || 0;
    this.elements["image-position"].textContent = `${this.model.currentIndex >= 0 ? this.model.currentIndex + 1 : 0} / ${total}`;
    const saveState = this.elements["save-state"];
    if (!hasImage) { saveState.dataset.state = "idle"; saveState.querySelector("strong").textContent = "No dataset"; }
    else if (this.model.dirty) { saveState.dataset.state = "dirty"; saveState.querySelector("strong").textContent = "Unsaved"; }
    else { saveState.dataset.state = "saved"; saveState.querySelector("strong").textContent = "Saved"; }
    this.elements["canvas-empty"].hidden = hasImage;
    this.elements["image-meta"].textContent = hasImage ? `${this.model.image.naturalWidth}×${this.model.image.naturalHeight} · ${this.model.annotations.length} objects` : "—";
  }
  resizeCanvas() {
    const rect = this.canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const width = Math.max(1, Math.round(rect.width * dpr));
    const height = Math.max(1, Math.round(rect.height * dpr));
    if (this.canvas.width !== width || this.canvas.height !== height) { this.canvas.width = width; this.canvas.height = height; }
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return {width: rect.width, height: rect.height, dpr};
  }
  transform() {
    const rect = this.canvas.getBoundingClientRect();
    const image = this.model.image;
    if (!image) return {scale: 1, x: 0, y: 0, width: rect.width, height: rect.height};
    const base = Math.min(rect.width / image.naturalWidth, rect.height / image.naturalHeight);
    const scale = base * this.zoom;
    return {
      scale,
      x: (rect.width - image.naturalWidth * scale) / 2 + this.panX,
      y: (rect.height - image.naturalHeight * scale) / 2 + this.panY,
      width: rect.width,
      height: rect.height,
    };
  }
  normToCanvas(x, y) {
    const t = this.transform();
    return {x: t.x + x * this.model.image.naturalWidth * t.scale, y: t.y + y * this.model.image.naturalHeight * t.scale};
  }
  canvasToNorm(clientX, clientY, clampPoint = true) {
    const rect = this.canvas.getBoundingClientRect();
    const t = this.transform();
    let x = ((clientX - rect.left - t.x) / t.scale) / this.model.image.naturalWidth;
    let y = ((clientY - rect.top - t.y) / t.scale) / this.model.image.naturalHeight;
    if (clampPoint) { x = clamp(x); y = clamp(y); }
    return {x, y};
  }
  eventPoint(event) { const rect = this.canvas.getBoundingClientRect(); return {x: event.clientX - rect.left, y: event.clientY - rect.top}; }
  fit() { this.zoom = 1; this.panX = 0; this.panY = 0; this.renderCanvas(); }
  zoomBy(factor, clientX = null, clientY = null) {
    if (!this.model.image) return;
    const rect = this.canvas.getBoundingClientRect();
    const sx = clientX === null ? rect.width / 2 : clientX - rect.left;
    const sy = clientY === null ? rect.height / 2 : clientY - rect.top;
    const old = this.transform();
    const imageX = (sx - old.x) / old.scale;
    const imageY = (sy - old.y) / old.scale;
    this.zoom = clamp(this.zoom * factor, 0.25, 20);
    const base = Math.min(rect.width / this.model.image.naturalWidth, rect.height / this.model.image.naturalHeight);
    const scale = base * this.zoom;
    this.panX = sx - (rect.width - this.model.image.naturalWidth * scale) / 2 - imageX * scale;
    this.panY = sy - (rect.height - this.model.image.naturalHeight * scale) / 2 - imageY * scale;
    this.renderCanvas();
  }
  renderCanvas() {
    const {width, height} = this.resizeCanvas();
    this.ctx.clearRect(0, 0, width, height);
    const image = this.model.image;
    if (!image) return;
    const t = this.transform();
    this.ctx.imageSmoothingEnabled = true;
    this.ctx.drawImage(image, t.x, t.y, image.naturalWidth * t.scale, image.naturalHeight * t.scale);
    if (!this.model.overlayVisible) return;
    this.model.annotations.forEach((annotation, index) => this.drawAnnotation(annotation, index));
    if (this.previewBox) this.drawDashedRect(this.previewBox, "#727986");
    if (this.selectionRect) this.drawDashedRect(this.selectionRect, "#8291ff", true);
  }
  drawAnnotation(annotation, index) {
    const selected = index === this.model.selectedIndex || this.model.selectedAnnotations.has(index);
    const color = selected ? "#f1f3f6" : classColor(annotation.class_id);
    const [left, top, right, bottom] = bboxEdges(annotation);
    const start = this.normToCanvas(left, top); const end = this.normToCanvas(right, bottom);
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = selected ? 3 : 2; ctx.strokeRect(start.x, start.y, end.x - start.x, end.y - start.y);
    const visiblePoints = annotation.keypoints.filter((point) => point[2] > 0).map((point) => this.normToCanvas(point[0], point[1]));
    if (visiblePoints.length > 1) {
      ctx.beginPath(); ctx.strokeStyle = "#727986"; ctx.lineWidth = 1.7;
      visiblePoints.forEach((point, pointIndex) => pointIndex ? ctx.lineTo(point.x, point.y) : ctx.moveTo(point.x, point.y));
      ctx.closePath(); ctx.stroke();
    }
    annotation.keypoints.forEach(([x, y, visibility], pointIndex) => {
      if (visibility <= 0) return;
      const point = this.normToCanvas(x, y); const keySelected = this.model.selectedKeypoints.has(`${index}:${pointIndex}`);
      ctx.beginPath(); ctx.arc(point.x, point.y, keySelected ? 7 : 5, 0, Math.PI * 2);
      ctx.strokeStyle = keySelected ? "#f1f3f6" : KEYPOINT_COLORS[pointIndex]; ctx.lineWidth = keySelected ? 3 : 2;
      if (visibility === 1) { ctx.setLineDash([3, 2]); ctx.stroke(); ctx.setLineDash([]); }
      else { ctx.fillStyle = KEYPOINT_COLORS[pointIndex]; ctx.fill(); ctx.stroke(); }
      if (selected || keySelected) {
        ctx.font = "700 10px ui-monospace, monospace"; ctx.lineWidth = 3; ctx.strokeStyle = "#0c0e12"; ctx.strokeText(`${pointIndex + 1}:${visibility}`, point.x + 7, point.y - 8);
        ctx.fillStyle = KEYPOINT_COLORS[pointIndex]; ctx.fillText(`${pointIndex + 1}:${visibility}`, point.x + 7, point.y - 8);
      }
    });
    if (selected) [[start.x, start.y], [end.x, start.y], [end.x, end.y], [start.x, end.y]].forEach(([x, y]) => {
      ctx.fillStyle = "#f1f3f6"; ctx.strokeStyle = "#12151a"; ctx.lineWidth = 1; ctx.fillRect(x - 4, y - 4, 8, 8); ctx.strokeRect(x - 4, y - 4, 8, 8);
    });
    ctx.font = "700 11px ui-sans-serif, sans-serif"; const labelWidth = ctx.measureText(annotation.label).width + 10;
    const labelY = Math.max(0, start.y - 19); ctx.fillStyle = "#11141ae6"; ctx.fillRect(start.x, labelY, labelWidth, 18);
    ctx.fillStyle = color; ctx.fillText(annotation.label, start.x + 5, labelY + 13);
    ctx.restore();
  }
  drawDashedRect(rect, color, fill = false) {
    const ctx = this.ctx; const a = this.normToCanvas(rect.x1, rect.y1); const b = this.normToCanvas(rect.x2, rect.y2);
    const left = Math.min(a.x, b.x), top = Math.min(a.y, b.y), width = Math.abs(b.x - a.x), height = Math.abs(b.y - a.y);
    ctx.save(); ctx.setLineDash([6, 4]); ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.strokeRect(left, top, width, height);
    if (fill) { ctx.fillStyle = "#8291ff18"; ctx.fillRect(left, top, width, height); } ctx.restore();
  }
  hitTest(event) {
    if (!this.model.image || !this.model.overlayVisible) return null;
    const point = this.eventPoint(event); const threshold = 10;
    for (let index = this.model.annotations.length - 1; index >= 0; index -= 1) {
      const annotation = this.model.annotations[index];
      for (let kp = 0; kp < annotation.keypoints.length; kp += 1) {
        const [x, y, visibility] = annotation.keypoints[kp]; if (visibility <= 0) continue;
        const screen = this.normToCanvas(x, y);
        if (Math.hypot(point.x - screen.x, point.y - screen.y) <= threshold) return {type: "keypoint", index, detail: kp};
      }
      const [left, top, right, bottom] = bboxEdges(annotation);
      const a = this.normToCanvas(left, top), b = this.normToCanvas(right, bottom);
      const corners = [[a.x, a.y], [b.x, a.y], [b.x, b.y], [a.x, b.y]];
      for (let corner = 0; corner < corners.length; corner += 1) if (Math.hypot(point.x - corners[corner][0], point.y - corners[corner][1]) <= threshold) return {type: "resize", index, detail: corner};
      if (point.x >= a.x && point.x <= b.x && point.y >= a.y && point.y <= b.y) return {type: "move", index, detail: -1};
    }
    return null;
  }
  async loadImage(url) {
    const image = new Image();
    image.decoding = "async";
    await new Promise((resolve, reject) => { image.onload = resolve; image.onerror = () => reject(new Error("image를 불러올 수 없습니다")); image.src = `${url}?t=${Date.now()}`; });
    return image;
  }
  setBusy(busy) { this.elements.busy.hidden = !busy; }
  status(message) { this.elements["status-message"].textContent = message; }
  toast(message, error = false) {
    const toast = document.createElement("div"); toast.className = `toast${error ? " error" : ""}`; toast.textContent = message;
    this.elements["toast-region"].append(toast); setTimeout(() => toast.remove(), 4200);
  }
  chooseClass(current = null) {
    const dialog = this.elements["class-dialog"];
    const select = this.elements["dialog-class-select"];
    const classes = this.model.dataset.class_names || [];
    this.classOptions(select, current ?? classes[0]?.id ?? 0);
    this.elements["custom-class-wrap"].hidden = classes.length > 0;
    this.elements["custom-class-id"].value = current ?? 0;
    dialog.showModal();
    return new Promise((resolve) => dialog.addEventListener("close", () => {
      if (dialog.returnValue !== "confirm") { resolve(null); return; }
      const value = classes.length ? Number(select.value) : Number(this.elements["custom-class-id"].value);
      resolve(Number.isInteger(value) && value >= 0 && value <= 9999 ? value : null);
    }, {once: true}));
  }
}

// Controller: API orchestration, UI events, and edit gestures.
class EditorController {
  constructor(model, view, api) {
    this.model = model; this.view = view; this.api = api;
    this.interaction = null; this.loadSequence = 0;
    this.model.subscribe((scope) => scope === "canvas" ? this.view.renderCanvas() : this.view.render());
    this.bindEvents();
  }
  bindEvents() {
    const el = this.view.elements;
    document.querySelector("#open-form").addEventListener("submit", (event) => { event.preventDefault(); this.openDataset(el["dataset-path"].value); });
    el["image-search"].addEventListener("input", () => this.view.renderDataset(true));
    el["image-list"].addEventListener("click", (event) => { const row = event.target.closest("li[data-index]"); if (row) this.navigate(Number(row.dataset.index)); });
    el["object-list"].addEventListener("click", (event) => { const row = event.target.closest("li[data-index]"); if (row) this.selectObject(Number(row.dataset.index)); });
    el["save-button"].addEventListener("click", () => this.save());
    el["undo-button"].addEventListener("click", () => this.model.undo());
    el["add-button"].addEventListener("click", () => this.toggleAddMode());
    el["delete-button"].addEventListener("click", () => this.deleteSelected());
    el["delete-sample-button"].addEventListener("click", () => this.deleteCurrentImage());
    el["visibility-button"].addEventListener("click", () => this.autoVisibility());
    el["previous-button"].addEventListener("click", () => this.navigate(this.model.currentIndex - 1));
    el["next-button"].addEventListener("click", () => this.navigate(this.model.currentIndex + 1));
    el["fit-button"].addEventListener("click", () => this.view.fit());
    el["zoom-in-button"].addEventListener("click", () => this.view.zoomBy(1.2));
    el["zoom-out-button"].addEventListener("click", () => this.view.zoomBy(1 / 1.2));
    el["overlay-toggle"].addEventListener("change", () => { this.model.overlayVisible = el["overlay-toggle"].checked; this.model.notify("canvas"); });
    el["class-select"].addEventListener("change", () => this.changeClass(Number(el["class-select"].value)));
    el["keypoint-controls"].addEventListener("change", (event) => { if (event.target.matches("select[data-keypoint]")) this.changeVisibility(Number(event.target.dataset.keypoint), Number(event.target.value)); });
    this.view.canvas.addEventListener("pointerdown", (event) => this.pointerDown(event));
    this.view.canvas.addEventListener("pointermove", (event) => this.pointerMove(event));
    this.view.canvas.addEventListener("pointerup", (event) => this.pointerUp(event));
    this.view.canvas.addEventListener("pointercancel", (event) => this.pointerUp(event));
    this.view.canvas.addEventListener("wheel", (event) => { if (!this.model.image) return; event.preventDefault(); this.view.zoomBy(event.deltaY < 0 ? 1.12 : 1 / 1.12, event.clientX, event.clientY); }, {passive: false});
    window.addEventListener("keydown", (event) => this.keyDown(event));
    window.addEventListener("beforeunload", (event) => { if (this.model.dirty) { event.preventDefault(); event.returnValue = ""; } });
  }
  async init() {
    try {
      const state = await this.api.state(); this.model.setDataset(state);
      if (state.open && state.images.length) await this.loadImage(0);
    } catch (error) { this.fail(error); }
  }
  async openDataset(path) {
    if (!path.trim() || !(await this.confirmLeave())) return;
    this.view.setBusy(true);
    try {
      const dataset = await this.api.open(path.trim()); this.model.setDataset(dataset);
      if (dataset.images.length) await this.loadImage(0); else this.view.status("No images");
    } catch (error) { this.fail(error); }
    finally { this.view.setBusy(false); }
  }
  async navigate(index) {
    if (index === this.model.currentIndex || index < 0 || index >= this.model.dataset.images.length) return;
    if (!(await this.confirmLeave())) return;
    await this.loadImage(index);
  }
  async confirmLeave() {
    if (!this.model.dirty) return true;
    if (window.confirm("현재 annotation을 저장하고 계속하시겠습니까?")) return await this.save();
    if (window.confirm("저장하지 않은 변경을 버리고 계속하시겠습니까?")) { this.model.discardChanges(); return true; }
    return false;
  }
  async loadImage(index) {
    const sequence = ++this.loadSequence; this.view.setBusy(true);
    try {
      const metadata = await this.api.image(index);
      const image = await this.view.loadImage(metadata.image_url);
      if (sequence !== this.loadSequence) return;
      this.model.setImage(index, metadata, image); this.view.fit();
      this.view.status(metadata.editable ? metadata.annotation_path : "Read-only image");
    } catch (error) { this.fail(error); }
    finally { if (sequence === this.loadSequence) this.view.setBusy(false); }
  }
  async save() {
    if (!this.model.editable || !this.model.dirty) return true;
    this.view.setBusy(true);
    try {
      const result = await this.api.save(this.model.currentIndex, this.model.revision, this.model.annotations);
      this.model.markSaved(result); this.view.status("Saved"); this.view.toast("Annotation을 저장했습니다."); return true;
    } catch (error) {
      this.fail(error.status === 409 ? new Error(`${error.message} (다른 탭 또는 프로세스의 변경을 확인하세요.)`) : error); return false;
    } finally { this.view.setBusy(false); }
  }
  selectObject(index) {
    this.model.selectedIndex = index; this.model.selectedAnnotations.clear(); this.model.selectedKeypoints.clear(); this.model.addMode = false; this.model.notify();
  }
  toggleAddMode() {
    if (!this.model.editable) return;
    this.model.addMode = !this.model.addMode; this.model.selectedIndex = -1; this.model.selectedAnnotations.clear(); this.model.selectedKeypoints.clear(); this.model.notify();
  }
  changeClass(classId) {
    const index = this.model.selectedIndex; if (!this.model.editable || !this.model.annotations[index]) return;
    this.model.mutate(() => { this.model.annotations[index].class_id = classId; this.model.annotations[index].label = this.model.labelFor(classId); });
  }
  changeVisibility(keypointIndex, visibility) {
    const index = this.model.selectedIndex; if (!this.model.editable || !this.model.annotations[index]) return;
    this.model.mutate(() => { this.model.annotations[index].keypoints[keypointIndex][2] = visibility; });
  }
  async editClass() {
    const annotation = this.model.annotations[this.model.selectedIndex]; if (!annotation) return;
    const classId = await this.view.chooseClass(annotation.class_id); if (classId !== null) this.changeClass(classId);
  }
  deleteSelected() {
    if (!this.model.editable) return;
    const indices = this.model.selectedAnnotations.size ? [...this.model.selectedAnnotations] : [this.model.selectedIndex];
    const valid = indices.filter((index) => index >= 0 && index < this.model.annotations.length); if (!valid.length) return;
    if (!window.confirm(`선택한 bbox ${valid.length}개를 삭제하시겠습니까?`)) return;
    this.model.mutate(() => {
      valid.sort((a, b) => b - a).forEach((index) => this.model.annotations.splice(index, 1));
      this.model.selectedIndex = -1; this.model.selectedAnnotations.clear(); this.model.selectedKeypoints.clear();
    });
  }
  async deleteCurrentImage() {
    if (!this.model.imageMeta || !this.model.dataset.dataset_root) return;
    const message = `해당 데이터를 영구 삭제하시겠습니까?\n\n이미지: ${this.model.imageMeta.image_path}\n라벨: ${this.model.imageMeta.annotation_path || "없음"}\n\nsamples.jsonl도 갱신됩니다.`;
    if (!window.confirm(message)) return;
    this.view.setBusy(true);
    try {
      const result = await this.api.deleteImage(this.model.currentIndex); this.model.setDataset(result.dataset);
      if (result.warnings.length) this.view.toast(result.warnings.join("\n"), true);
      if (result.next_index !== null) await this.loadImage(result.next_index); else this.view.status("No images remain");
    } catch (error) { this.fail(error); }
    finally { this.view.setBusy(false); }
  }
  async autoVisibility() {
    if (!this.model.editable || !this.model.annotations.length) return;
    const before = this.model.beforeMutation(); this.view.setBusy(true);
    try {
      const result = await this.api.autoVisibility(this.model.currentIndex, this.model.annotations);
      this.model.annotations = result.annotations; this.model.selectedIndex = Math.min(this.model.selectedIndex, result.annotations.length - 1);
      this.model.commitMutation(before);
      const message = `Auto visibility: deleted=${result.deleted_objects}, preserved=${result.preserved_occluded_objects}, occluded keypoints=${result.occluded_keypoints}`;
      this.view.status(message); this.view.toast(message);
    } catch (error) { this.fail(error); }
    finally { this.view.setBusy(false); }
  }
  pointerDown(event) {
    if (event.button !== 0 || !this.model.image) return;
    this.view.canvas.setPointerCapture(event.pointerId);
    const start = this.view.canvasToNorm(event.clientX, event.clientY);
    const screenStart = this.view.eventPoint(event);
    if (this.model.addMode && this.model.editable) {
      this.interaction = {type: "add", start}; this.view.previewBox = {x1: start.x, y1: start.y, x2: start.x, y2: start.y}; this.view.renderCanvas(); return;
    }
    if (event.shiftKey) {
      this.interaction = {type: "range", start}; this.view.selectionRect = {x1: start.x, y1: start.y, x2: start.x, y2: start.y}; this.view.renderCanvas(); return;
    }
    const hit = this.view.hitTest(event);
    if (!hit) {
      this.model.selectedIndex = -1; this.model.selectedAnnotations.clear(); this.model.selectedKeypoints.clear();
      this.interaction = {type: "pan", screenStart, panX: this.view.panX, panY: this.view.panY}; this.model.notify(); return;
    }
    const inGroup = this.model.selectedAnnotations.has(hit.index) || (hit.type === "keypoint" && this.model.selectedKeypoints.has(`${hit.index}:${hit.detail}`));
    if (!inGroup) {
      this.model.selectedIndex = hit.index; this.model.selectedAnnotations.clear(); this.model.selectedKeypoints.clear();
    }
    const type = inGroup && (this.model.selectedAnnotations.size || this.model.selectedKeypoints.size) ? "group" : hit.type;
    this.interaction = {type, hit, start, before: this.model.beforeMutation(), original: deepClone(this.model.annotations), changed: false};
    this.model.notify();
  }
  pointerMove(event) {
    if (!this.interaction || !this.model.image) return;
    const current = this.view.canvasToNorm(event.clientX, event.clientY);
    const action = this.interaction;
    if (action.type === "add") { this.view.previewBox.x2 = current.x; this.view.previewBox.y2 = current.y; this.view.renderCanvas(); return; }
    if (action.type === "range") { this.view.selectionRect.x2 = current.x; this.view.selectionRect.y2 = current.y; this.view.renderCanvas(); return; }
    if (action.type === "pan") { const point = this.view.eventPoint(event); this.view.panX = action.panX + point.x - action.screenStart.x; this.view.panY = action.panY + point.y - action.screenStart.y; this.view.renderCanvas(); return; }
    const dx = current.x - action.start.x, dy = current.y - action.start.y;
    if (action.type === "move") this.model.annotations[action.hit.index] = moveAnnotation(action.original[action.hit.index], dx, dy);
    else if (action.type === "resize") this.model.annotations[action.hit.index] = resizeAnnotation(action.original[action.hit.index], action.hit.detail, current.x, current.y);
    else if (action.type === "keypoint") {
      this.model.annotations = deepClone(action.original); const point = this.model.annotations[action.hit.index].keypoints[action.hit.detail]; point[0] = current.x; point[1] = current.y;
    } else if (action.type === "group") this.model.annotations = moveSelection(action.original, this.model.selectedAnnotations, this.model.selectedKeypoints, dx, dy);
    action.changed = annotationKey(action.before.annotations) !== annotationKey(this.model.annotations); this.view.renderCanvas();
  }
  async pointerUp(event) {
    if (!this.interaction) return;
    const action = this.interaction; this.interaction = null;
    if (action.type === "add") {
      const box = this.view.previewBox; this.view.previewBox = null;
      const left = Math.min(box.x1, box.x2), top = Math.min(box.y1, box.y2), right = Math.max(box.x1, box.x2), bottom = Math.max(box.y1, box.y2);
      const startScreen = this.view.normToCanvas(box.x1, box.y1), endScreen = this.view.normToCanvas(box.x2, box.y2);
      if (Math.hypot(endScreen.x - startScreen.x, endScreen.y - startScreen.y) >= 5) {
        const classId = await this.view.chooseClass();
        if (classId !== null) this.model.mutate(() => {
          this.model.annotations.push({class_id: classId, label: this.model.labelFor(classId), bbox: [(left + right) / 2, (top + bottom) / 2, right - left, bottom - top], keypoints: [[left, top, 2], [right, top, 2], [right, bottom, 2], [left, bottom, 2]]});
          this.model.selectedIndex = this.model.annotations.length - 1; this.model.selectedAnnotations.clear(); this.model.selectedKeypoints.clear();
        });
      }
      this.model.addMode = false; this.model.notify(); return;
    }
    if (action.type === "range") { this.applyRangeSelection(this.view.selectionRect); this.view.selectionRect = null; this.model.notify(); return; }
    if (action.changed) this.model.commitMutation(action.before); else this.model.notify();
  }
  applyRangeSelection(rect) {
    const left = Math.min(rect.x1, rect.x2), top = Math.min(rect.y1, rect.y2), right = Math.max(rect.x1, rect.x2), bottom = Math.max(rect.y1, rect.y2);
    this.model.selectedAnnotations.clear(); this.model.selectedKeypoints.clear(); this.model.selectedIndex = -1;
    this.model.annotations.forEach((annotation, annotationIndex) => {
      const [boxLeft, boxTop, boxRight, boxBottom] = bboxEdges(annotation);
      if (boxLeft >= left && boxTop >= top && boxRight <= right && boxBottom <= bottom) { this.model.selectedAnnotations.add(annotationIndex); return; }
      annotation.keypoints.forEach(([x, y, visibility], keypointIndex) => { if (visibility > 0 && x >= left && x <= right && y >= top && y <= bottom) this.model.selectedKeypoints.add(`${annotationIndex}:${keypointIndex}`); });
    });
  }
  keyDown(event) {
    const tag = event.target.tagName; if (["INPUT", "SELECT", "TEXTAREA"].includes(tag) || event.target.isContentEditable) return;
    const key = event.key.toLowerCase();
    if ((event.ctrlKey || event.metaKey) && key === "s") { event.preventDefault(); this.save(); return; }
    if ((event.ctrlKey || event.metaKey) && key === "z") { event.preventDefault(); this.model.undo(); return; }
    if ((event.ctrlKey || event.metaKey) && key === "d") { event.preventDefault(); this.deleteCurrentImage(); return; }
    if ((event.ctrlKey || event.metaKey) && key === "o") { event.preventDefault(); this.view.elements["dataset-path"].focus(); return; }
    if ((event.ctrlKey || event.metaKey) && ["+", "="].includes(key)) { event.preventDefault(); this.view.zoomBy(1.2); return; }
    if ((event.ctrlKey || event.metaKey) && key === "-") { event.preventDefault(); this.view.zoomBy(1 / 1.2); return; }
    if (key === "delete" || key === "backspace") { event.preventDefault(); this.deleteSelected(); }
    else if (key === "w") this.toggleAddMode();
    else if (key === "e") this.editClass();
    else if (key === "v") this.autoVisibility();
    else if (key === "f") this.view.fit();
    else if (key === "arrowleft" || key === "a") this.navigate(this.model.currentIndex - 1);
    else if (key === "arrowright" || key === "d") this.navigate(this.model.currentIndex + 1);
  }
  fail(error) { console.error(error); this.view.status(error.message); this.view.toast(error.message, true); }
}

const model = new AnnotationModel();
const view = new EditorView(model);
const controller = new EditorController(model, view, new ApiClient());
controller.init();
