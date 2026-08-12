"""LabelImg-style PyQt YOLO-pose annotation editor."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

from bounding_box_tool.dataset import (
    ImageDataset,
    ImageEntry,
    PoseAnnotation,
    load_annotations,
    save_annotations,
    samples_without_image,
    save_samples,
)
from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QKeySequence,
    QPainter,
    QPen,
    QPixmap,
    QPolygonF,
)
from PyQt5.QtWidgets import (
    QAction,
    QFileDialog,
    QGraphicsItem,
    QGraphicsPixmapItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QInputDialog,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QSplitter,
    QStyle,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

KEYPOINT_COLORS = (
    QColor("#ff3b30"),
    QColor("#ff9500"),
    QColor("#ff2dce"),
    QColor("#007aff"),
)
MIN_BOX_SIZE = 1e-5


def class_color(class_id: int) -> QColor:
    """class마다 안정적으로 구분되는 밝은 색을 반환한다."""
    return QColor.fromHsv((class_id * 47 + 110) % 360, 210, 255)


def _clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return min(max(value, low), high)


def _bbox_edges(annotation: PoseAnnotation) -> tuple[float, float, float, float]:
    center_x, center_y, width, height = annotation.bbox
    return (
        center_x - width / 2.0,
        center_y - height / 2.0,
        center_x + width / 2.0,
        center_y + height / 2.0,
    )


def _move_annotation(
    annotation: PoseAnnotation,
    delta_x: float,
    delta_y: float,
) -> PoseAnnotation:
    """bbox와 keypoint를 image 경계 안에서 함께 이동한다."""
    left, top, right, bottom = _bbox_edges(annotation)
    xs = [left, right, *(point[0] for point in annotation.keypoints)]
    ys = [top, bottom, *(point[1] for point in annotation.keypoints)]
    delta_x = _clamp(delta_x, -min(xs), 1.0 - max(xs))
    delta_y = _clamp(delta_y, -min(ys), 1.0 - max(ys))
    center_x, center_y, width, height = annotation.bbox
    keypoints = tuple(
        (x + delta_x, y + delta_y, visibility)
        for x, y, visibility in annotation.keypoints
    )
    return replace(
        annotation,
        bbox=(center_x + delta_x, center_y + delta_y, width, height),
        keypoints=keypoints,
    )


def _move_annotation_selection(
    annotations: tuple[PoseAnnotation, ...],
    annotation_indices: set[int],
    keypoint_indices: set[tuple[int, int]],
    delta_x: float,
    delta_y: float,
) -> tuple[PoseAnnotation, ...]:
    """선택된 bbox와 단독 keypoint를 같은 이동량으로 옮긴다."""
    xs: list[float] = []
    ys: list[float] = []
    for annotation_index, annotation in enumerate(annotations):
        if annotation_index in annotation_indices:
            left, top, right, bottom = _bbox_edges(annotation)
            xs.extend((left, right, *(point[0] for point in annotation.keypoints)))
            ys.extend((top, bottom, *(point[1] for point in annotation.keypoints)))
            continue
        for keypoint_index, (x, y, _visibility) in enumerate(annotation.keypoints):
            if (annotation_index, keypoint_index) in keypoint_indices:
                xs.append(x)
                ys.append(y)
    if not xs:
        return annotations
    delta_x = _clamp(delta_x, -min(xs), 1.0 - max(xs))
    delta_y = _clamp(delta_y, -min(ys), 1.0 - max(ys))
    moved = list(annotations)
    for annotation_index, annotation in enumerate(annotations):
        if annotation_index in annotation_indices:
            moved[annotation_index] = _move_annotation(annotation, delta_x, delta_y)
            continue
        keypoints = list(annotation.keypoints)
        changed = False
        for keypoint_index, (x, y, visibility) in enumerate(keypoints):
            if (annotation_index, keypoint_index) in keypoint_indices:
                keypoints[keypoint_index] = (x + delta_x, y + delta_y, visibility)
                changed = True
        if changed:
            moved[annotation_index] = replace(annotation, keypoints=tuple(keypoints))
    return tuple(moved)


def _resize_annotation(
    annotation: PoseAnnotation,
    corner: int,
    target_x: float,
    target_y: float,
) -> PoseAnnotation:
    """bbox corner를 이동하고 keypoint를 기존 bbox 상대 위치에 맞춰 변환한다."""
    old_left, old_top, old_right, old_bottom = _bbox_edges(annotation)
    left, top, right, bottom = old_left, old_top, old_right, old_bottom
    target_x, target_y = _clamp(target_x), _clamp(target_y)
    if corner in (0, 3):
        left = min(target_x, right - MIN_BOX_SIZE)
    else:
        right = max(target_x, left + MIN_BOX_SIZE)
    if corner in (0, 1):
        top = min(target_y, bottom - MIN_BOX_SIZE)
    else:
        bottom = max(target_y, top + MIN_BOX_SIZE)
    old_width = max(old_right - old_left, MIN_BOX_SIZE)
    old_height = max(old_bottom - old_top, MIN_BOX_SIZE)
    width, height = right - left, bottom - top
    keypoints = tuple(
        (
            _clamp(left + (x - old_left) / old_width * width),
            _clamp(top + (y - old_top) / old_height * height),
            visibility,
        )
        for x, y, visibility in annotation.keypoints
    )
    return replace(
        annotation,
        bbox=((left + right) / 2.0, (top + bottom) / 2.0, width, height),
        keypoints=keypoints,
    )


def _set_keypoint(
    annotation: PoseAnnotation,
    keypoint_index: int,
    x: float,
    y: float,
) -> PoseAnnotation:
    """하나의 keypoint 위치만 image 경계 안에서 변경한다."""
    keypoints = list(annotation.keypoints)
    _, _, visibility = keypoints[keypoint_index]
    keypoints[keypoint_index] = (_clamp(x), _clamp(y), visibility)
    return replace(annotation, keypoints=tuple(keypoints))


class AnnotationCanvas(QGraphicsView):
    """bbox와 keypoint를 선택·생성·이동·크기 조절하는 canvas."""

    selectionChanged = pyqtSignal(int)
    annotationsChanged = pyqtSignal()
    boxDrawn = pyqtSignal(object)
    addModeChanged = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._pixmap_item: QGraphicsPixmapItem | None = None
        self._overlay_items: list[QGraphicsItem] = []
        self._annotations: list[PoseAnnotation] = []
        self._selected_index = -1
        self._fit_mode = True
        self._annotations_visible = True
        self._add_mode = False
        self._add_origin: QPointF | None = None
        self._preview_item: QGraphicsRectItem | None = None
        self._drag: tuple[str, int, int] | None = None
        self._drag_origin: QPointF | None = None
        self._drag_original: PoseAnnotation | None = None
        self._drag_original_annotations: tuple[PoseAnnotation, ...] | None = None
        self._drag_undo_state = None
        self._selection_origin: QPointF | None = None
        self._selection_preview: QGraphicsRectItem | None = None
        self._selected_annotations: set[int] = set()
        self._selected_keypoints: set[tuple[int, int]] = set()
        self._undo_stack: list[tuple] = []
        self.setBackgroundBrush(QColor("#202124"))
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setRenderHints(
            self.renderHints()
            | QPainter.Antialiasing
            | QPainter.SmoothPixmapTransform
        )

    @property
    def image_size(self) -> tuple[int, int]:
        """현재 image의 pixel 크기를 반환한다."""
        if self._pixmap_item is None:
            return 0, 0
        pixmap = self._pixmap_item.pixmap()
        return pixmap.width(), pixmap.height()

    @property
    def annotations(self) -> tuple[PoseAnnotation, ...]:
        return tuple(self._annotations)

    @property
    def selected_index(self) -> int:
        return self._selected_index

    @property
    def can_undo(self) -> bool:
        return bool(self._undo_stack)

    @property
    def has_deletable_selection(self) -> bool:
        return self._selected_index >= 0 or bool(self._selected_annotations)

    def _snapshot(self) -> tuple:
        return (
            tuple(self._annotations),
            self._selected_index,
            frozenset(self._selected_annotations),
            frozenset(self._selected_keypoints),
        )

    def _push_undo(self) -> None:
        self._undo_stack.append(self._snapshot())

    def undo(self) -> None:
        if not self._undo_stack:
            return
        annotations, selected_index, selected_annotations, selected_keypoints = (
            self._undo_stack.pop()
        )
        self._annotations = list(annotations)
        self._selected_index = selected_index
        self._selected_annotations = set(selected_annotations)
        self._selected_keypoints = set(selected_keypoints)
        self._redraw_overlays()
        self.selectionChanged.emit(self._selected_index)
        self.annotationsChanged.emit()

    def set_image(
        self,
        image_path: Path,
        annotations: tuple[PoseAnnotation, ...],
    ) -> bool:
        """image와 편집 가능한 annotation을 교체한다."""
        pixmap = QPixmap(str(image_path))
        if pixmap.isNull():
            return False
        self._scene.clear()
        self._overlay_items.clear()
        self._preview_item = None
        self._selection_preview = None
        self._pixmap_item = self._scene.addPixmap(pixmap)
        self._pixmap_item.setZValue(0)
        self._scene.setSceneRect(QRectF(pixmap.rect()))
        self._annotations = list(annotations)
        self._selected_index = -1
        self._selected_annotations.clear()
        self._selected_keypoints.clear()
        self._undo_stack.clear()
        self._redraw_overlays()
        self.resetTransform()
        self.fit_to_view()
        self.selectionChanged.emit(-1)
        return True

    def clear_image(self) -> None:
        """마지막 image 삭제 후 canvas 상태를 비운다."""
        self.set_add_mode(False)
        self._scene.clear()
        self._overlay_items.clear()
        self._pixmap_item = None
        self._annotations.clear()
        self._selected_index = -1
        self._selected_annotations.clear()
        self._selected_keypoints.clear()
        self._undo_stack.clear()
        self.selectionChanged.emit(-1)

    def _annotation_rect(self, annotation: PoseAnnotation) -> QRectF:
        image_width, image_height = self.image_size
        center_x, center_y, width, height = annotation.bbox
        return QRectF(
            (center_x - width / 2.0) * image_width,
            (center_y - height / 2.0) * image_height,
            width * image_width,
            height * image_height,
        )

    def _clear_overlays(self) -> None:
        for item in self._overlay_items:
            self._scene.removeItem(item)
        self._overlay_items.clear()

    def _redraw_overlays(self) -> None:
        self._clear_overlays()
        image_width, image_height = self.image_size
        if not image_width or not image_height:
            return
        for index, annotation in enumerate(self._annotations):
            self._draw_annotation(
                annotation,
                image_width,
                image_height,
                selected=(
                    index == self._selected_index
                    or index in self._selected_annotations
                ),
                selected_keypoints={
                    keypoint_index
                    for annotation_index, keypoint_index in self._selected_keypoints
                    if annotation_index == index
                },
            )
        for item in self._overlay_items:
            item.setVisible(self._annotations_visible)

    def _add_overlay(self, item: QGraphicsItem, z_value: float) -> None:
        item.setZValue(z_value)
        self._overlay_items.append(item)

    def _draw_annotation(
        self,
        annotation: PoseAnnotation,
        image_width: int,
        image_height: int,
        *,
        selected: bool,
        selected_keypoints: set[int],
    ) -> None:
        """bbox, polygon, visibility별 keypoint와 선택 handle을 그린다."""
        color = QColor("#ffee58") if selected else class_color(annotation.class_id)
        rectangle = self._annotation_rect(annotation)
        box_pen = QPen(color, 3.0 if selected else 2.5)
        box_pen.setCosmetic(True)
        self._add_overlay(self._scene.addRect(rectangle, box_pen), 10)

        points = [
            QPointF(x * image_width, y * image_height)
            for x, y, visibility in annotation.keypoints
            if visibility > 0
        ]
        if len(points) >= 2:
            polygon_pen = QPen(QColor("#00e5ff"), 2.0)
            polygon_pen.setCosmetic(True)
            self._add_overlay(
                self._scene.addPolygon(QPolygonF(points), polygon_pen), 11
            )

        for index, (x, y, visibility) in enumerate(annotation.keypoints, 1):
            if visibility <= 0:
                continue
            point = QPointF(x * image_width, y * image_height)
            keypoint_selected = index - 1 in selected_keypoints
            marker_pen = QPen(
                QColor("#ffee58") if keypoint_selected else KEYPOINT_COLORS[index - 1],
                3.5 if keypoint_selected else 2.0,
            )
            marker_pen.setCosmetic(True)
            brush = QBrush(KEYPOINT_COLORS[index - 1])
            if visibility == 1:
                marker_pen.setStyle(Qt.DashLine)
                brush = QBrush(Qt.NoBrush)
            radius = 7.0 if keypoint_selected else 5.0
            marker = self._scene.addEllipse(
                -radius, -radius, radius * 2.0, radius * 2.0, marker_pen, brush
            )
            marker.setPos(point)
            marker.setFlag(QGraphicsItem.ItemIgnoresTransformations)
            self._add_overlay(marker, 13)

            number = QGraphicsSimpleTextItem(f"{index}:{visibility}")
            number.setBrush(KEYPOINT_COLORS[index - 1])
            number.setPen(QPen(Qt.black, 1.0))
            number.setPos(point + QPointF(6.0, -18.0))
            number.setFlag(QGraphicsItem.ItemIgnoresTransformations)
            self._scene.addItem(number)
            self._add_overlay(number, 14)

        if selected:
            for corner in (
                rectangle.topLeft(),
                rectangle.topRight(),
                rectangle.bottomRight(),
                rectangle.bottomLeft(),
            ):
                handle = self._scene.addRect(
                    -4.0,
                    -4.0,
                    8.0,
                    8.0,
                    QPen(Qt.black, 1.0),
                    QBrush(QColor("#ffee58")),
                )
                handle.setPos(corner)
                handle.setFlag(QGraphicsItem.ItemIgnoresTransformations)
                self._add_overlay(handle, 15)

        label = QGraphicsSimpleTextItem(annotation.label)
        label.setBrush(color)
        label.setPen(QPen(Qt.black, 1.5))
        label.setPos(QPointF(rectangle.left(), max(0.0, rectangle.top() - 20.0)))
        label.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        self._scene.addItem(label)
        self._add_overlay(label, 14)

    def set_selected_index(self, index: int) -> None:
        """객체 selection을 변경하고 handle을 다시 그린다."""
        normalized = index if 0 <= index < len(self._annotations) else -1
        if (
            normalized == self._selected_index
            and not self._selected_annotations
            and not self._selected_keypoints
        ):
            return
        self._selected_index = normalized
        self._selected_annotations.clear()
        self._selected_keypoints.clear()
        self._redraw_overlays()
        self.selectionChanged.emit(normalized)

    def add_annotation(self, annotation: PoseAnnotation) -> None:
        self._push_undo()
        self._annotations.append(annotation)
        self._selected_index = len(self._annotations) - 1
        self._redraw_overlays()
        self.selectionChanged.emit(self._selected_index)
        self.annotationsChanged.emit()

    def replace_selected(self, annotation: PoseAnnotation) -> None:
        if not 0 <= self._selected_index < len(self._annotations):
            return
        if self._annotations[self._selected_index] == annotation:
            return
        self._push_undo()
        self._annotations[self._selected_index] = annotation
        self._redraw_overlays()
        self.annotationsChanged.emit()

    def delete_selected(self) -> None:
        indices = (
            self._selected_annotations
            if self._selected_annotations
            else {self._selected_index}
        )
        indices = {index for index in indices if 0 <= index < len(self._annotations)}
        if not indices:
            return
        self._push_undo()
        for index in sorted(indices, reverse=True):
            del self._annotations[index]
        self._selected_index = -1
        self._selected_annotations.clear()
        self._selected_keypoints.clear()
        self._redraw_overlays()
        self.selectionChanged.emit(-1)
        self.annotationsChanged.emit()

    def set_annotations(self, annotations: tuple[PoseAnnotation, ...]) -> None:
        """후처리된 annotation 전체를 현재 image에 적용한다."""
        if annotations == tuple(self._annotations):
            return
        self._push_undo()
        self._annotations = list(annotations)
        self._selected_annotations.clear()
        self._selected_keypoints.clear()
        self._selected_index = min(self._selected_index, len(self._annotations) - 1)
        self._redraw_overlays()
        self.selectionChanged.emit(self._selected_index)
        self.annotationsChanged.emit()

    def set_annotations_visible(self, visible: bool) -> None:
        self._annotations_visible = visible
        for item in self._overlay_items:
            item.setVisible(visible)

    def set_add_mode(self, enabled: bool) -> None:
        enabled = bool(enabled and self._pixmap_item is not None)
        if self._add_mode == enabled:
            return
        self._add_mode = enabled
        self._add_origin = None
        if self._preview_item is not None:
            self._scene.removeItem(self._preview_item)
            self._preview_item = None
        if self._selection_preview is not None:
            self._scene.removeItem(self._selection_preview)
            self._selection_preview = None
        self._selection_origin = None
        self.setCursor(Qt.CrossCursor if enabled else Qt.ArrowCursor)
        self.addModeChanged.emit(enabled)

    def _clamped_scene_point(self, point: QPointF) -> QPointF:
        width, height = self.image_size
        return QPointF(_clamp(point.x(), 0.0, float(width)), _clamp(point.y(), 0.0, float(height)))

    def _hit_test(self, point: QPointF) -> tuple[str, int, int] | None:
        if not self._annotations_visible:
            return None
        scale = max(abs(self.transform().m11()), 1e-9)
        threshold = 9.0 / scale
        width, height = self.image_size
        for annotation_index in reversed(range(len(self._annotations))):
            annotation = self._annotations[annotation_index]
            for keypoint_index, (x, y, visibility) in enumerate(annotation.keypoints):
                if visibility <= 0:
                    continue
                if math.hypot(point.x() - x * width, point.y() - y * height) <= threshold:
                    return "keypoint", annotation_index, keypoint_index
            rectangle = self._annotation_rect(annotation)
            corners = (
                rectangle.topLeft(),
                rectangle.topRight(),
                rectangle.bottomRight(),
                rectangle.bottomLeft(),
            )
            for corner_index, corner in enumerate(corners):
                if math.hypot(point.x() - corner.x(), point.y() - corner.y()) <= threshold:
                    return "resize", annotation_index, corner_index
            if rectangle.contains(point):
                return "move", annotation_index, -1
        return None

    def _hit_selected_range(self, hit: tuple[str, int, int]) -> bool:
        kind, annotation_index, detail = hit
        return annotation_index in self._selected_annotations or (
            kind == "keypoint"
            and (annotation_index, detail) in self._selected_keypoints
        )

    def _select_range(self, rectangle: QRectF) -> None:
        width, height = self.image_size
        self._selected_annotations = {
            index
            for index, annotation in enumerate(self._annotations)
            if rectangle.contains(self._annotation_rect(annotation))
        }
        self._selected_keypoints = {
            (annotation_index, keypoint_index)
            for annotation_index, annotation in enumerate(self._annotations)
            if annotation_index not in self._selected_annotations
            for keypoint_index, (x, y, visibility) in enumerate(annotation.keypoints)
            if visibility > 0 and rectangle.contains(QPointF(x * width, y * height))
        }
        self._selected_index = -1
        self._redraw_overlays()
        self.selectionChanged.emit(-1)

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.LeftButton or self._pixmap_item is None:
            super().mousePressEvent(event)
            return
        point = self._clamped_scene_point(self.mapToScene(event.pos()))
        if self._add_mode:
            self._add_origin = point
            pen = QPen(QColor("#69f0ae"), 2.0, Qt.DashLine)
            pen.setCosmetic(True)
            self._preview_item = self._scene.addRect(QRectF(point, point), pen)
            self._preview_item.setZValue(20)
            event.accept()
            return
        if event.modifiers() & Qt.ShiftModifier:
            self._selection_origin = point
            pen = QPen(QColor("#ffee58"), 2.0, Qt.DashLine)
            pen.setCosmetic(True)
            self._selection_preview = self._scene.addRect(QRectF(point, point), pen)
            self._selection_preview.setZValue(20)
            event.accept()
            return
        hit = self._hit_test(point)
        if hit is None:
            self.set_selected_index(-1)
            super().mousePressEvent(event)
            return
        if self._hit_selected_range(hit):
            self._drag = ("group", -1, -1)
            self._drag_origin = point
            self._drag_original_annotations = tuple(self._annotations)
            self._drag_undo_state = self._snapshot()
            event.accept()
            return
        _kind, annotation_index, _detail = hit
        self.set_selected_index(annotation_index)
        self._drag = hit
        self._drag_origin = point
        self._drag_original = self._annotations[annotation_index]
        self._drag_undo_state = self._snapshot()
        event.accept()

    def mouseMoveEvent(self, event) -> None:
        point = self._clamped_scene_point(self.mapToScene(event.pos()))
        if self._add_mode and self._add_origin is not None:
            if self._preview_item is not None:
                self._preview_item.setRect(QRectF(self._add_origin, point).normalized())
            event.accept()
            return
        if self._selection_origin is not None:
            if self._selection_preview is not None:
                self._selection_preview.setRect(
                    QRectF(self._selection_origin, point).normalized()
                )
            event.accept()
            return
        if self._drag is None or self._drag_origin is None:
            super().mouseMoveEvent(event)
            return
        kind, annotation_index, detail = self._drag
        width, height = self.image_size
        normalized_x = point.x() / width
        normalized_y = point.y() / height
        if kind == "group" and self._drag_original_annotations is not None:
            self._annotations = list(
                _move_annotation_selection(
                    self._drag_original_annotations,
                    self._selected_annotations,
                    self._selected_keypoints,
                    (point.x() - self._drag_origin.x()) / width,
                    (point.y() - self._drag_origin.y()) / height,
                )
            )
            self._redraw_overlays()
            event.accept()
            return
        elif self._drag_original is None:
            return
        elif kind == "move":
            annotation = _move_annotation(
                self._drag_original,
                (point.x() - self._drag_origin.x()) / width,
                (point.y() - self._drag_origin.y()) / height,
            )
        elif kind == "resize":
            annotation = _resize_annotation(
                self._drag_original,
                detail,
                normalized_x,
                normalized_y,
            )
        else:
            annotation = _set_keypoint(
                self._drag_original,
                detail,
                normalized_x,
                normalized_y,
            )
        self._annotations[annotation_index] = annotation
        self._redraw_overlays()
        event.accept()

    def mouseReleaseEvent(self, event) -> None:
        point = self._clamped_scene_point(self.mapToScene(event.pos()))
        if event.button() == Qt.LeftButton and self._add_mode and self._add_origin is not None:
            rectangle = QRectF(self._add_origin, point).normalized()
            self._add_origin = None
            if self._preview_item is not None:
                self._scene.removeItem(self._preview_item)
                self._preview_item = None
            self.set_add_mode(False)
            width, height = self.image_size
            if rectangle.width() >= 4.0 and rectangle.height() >= 4.0:
                self.boxDrawn.emit(
                    (
                        rectangle.center().x() / width,
                        rectangle.center().y() / height,
                        rectangle.width() / width,
                        rectangle.height() / height,
                    )
                )
            event.accept()
            return
        if event.button() == Qt.LeftButton and self._selection_origin is not None:
            rectangle = QRectF(self._selection_origin, point).normalized()
            self._selection_origin = None
            if self._selection_preview is not None:
                self._scene.removeItem(self._selection_preview)
                self._selection_preview = None
            self._select_range(rectangle)
            event.accept()
            return
        if event.button() == Qt.LeftButton and self._drag is not None:
            changed = (
                self._drag_undo_state is not None
                and tuple(self._annotations) != self._drag_undo_state[0]
            )
            self._drag = None
            self._drag_origin = None
            self._drag_original = None
            self._drag_original_annotations = None
            if changed:
                self._undo_stack.append(self._drag_undo_state)
                self.annotationsChanged.emit()
            self._drag_undo_state = None
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def fit_to_view(self) -> None:
        if self._pixmap_item is None:
            return
        self._fit_mode = True
        self.fitInView(self._pixmap_item, Qt.KeepAspectRatio)

    def zoom_by(self, factor: float) -> None:
        if self._pixmap_item is None:
            return
        current = self.transform().m11()
        if not 0.03 <= current * factor <= 50.0:
            return
        self._fit_mode = False
        self.scale(factor, factor)

    def wheelEvent(self, event) -> None:
        self.zoom_by(1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self._fit_mode:
            self.fit_to_view()


class BoundingBoxViewer(QMainWindow):
    """image/object 목록과 YOLO-pose CRUD canvas를 갖는 editor."""

    def __init__(self, initial_path: str | Path | None = None, parent=None):
        super().__init__(parent)
        self.dataset: ImageDataset | None = None
        self.current_index = -1
        self.dirty = False
        self._saved_annotations: tuple[PoseAnnotation, ...] = ()
        self._changing_image_row = False
        self.setWindowTitle("Bounding Box Tool")
        self.resize(1500, 900)
        self.setAcceptDrops(True)

        self.image_list = QListWidget()
        self.image_list.setMinimumWidth(280)
        self.image_list.currentRowChanged.connect(self._show_image)
        self.object_list = QListWidget()
        self.object_list.setMinimumWidth(260)
        self.object_list.currentRowChanged.connect(self.canvas_selection_from_list)
        self.object_list.itemDoubleClicked.connect(lambda _item: self._edit_class())
        self.annotation_path = QLabel("No annotation")
        self.annotation_path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.annotation_path.setWordWrap(True)
        self.warning_label = QLabel("")
        self.warning_label.setStyleSheet("color: #ffb74d;")
        self.warning_label.setWordWrap(True)
        self.canvas = AnnotationCanvas()
        self.canvas.selectionChanged.connect(self._select_object_row)
        self.canvas.annotationsChanged.connect(self._annotations_changed)
        self.canvas.boxDrawn.connect(self._add_drawn_box)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._panel("Images", self.image_list))
        splitter.addWidget(self.canvas)
        splitter.addWidget(
            self._panel(
                "Objects",
                self.object_list,
                QLabel("Annotation"),
                self.annotation_path,
                self.warning_label,
            )
        )
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        splitter.setSizes([300, 900, 300])
        self.setCentralWidget(splitter)

        self._create_actions()
        self.canvas.addModeChanged.connect(self._sync_add_action)
        self._create_toolbar()
        self._create_menus()
        self._update_actions()
        self.statusBar().showMessage("Open a dataset root or image folder")
        if initial_path is not None:
            self.open_path(initial_path)

    @staticmethod
    def _panel(title: str, *widgets: QWidget) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        heading = QLabel(title)
        heading.setStyleSheet("font-weight: 600; font-size: 14px;")
        layout.addWidget(heading)
        for widget in widgets:
            layout.addWidget(widget)
        return panel

    def _create_actions(self) -> None:
        self.open_action = QAction(
            self.style().standardIcon(QStyle.SP_DialogOpenButton),
            "Open Folder…",
            self,
        )
        self.open_action.setShortcut(QKeySequence.Open)
        self.open_action.triggered.connect(self._choose_folder)

        self.save_action = QAction(
            self.style().standardIcon(QStyle.SP_DialogSaveButton), "Save", self
        )
        self.save_action.setShortcut(QKeySequence.Save)
        self.save_action.triggered.connect(self.save_current)

        self.add_action = QAction("Add Box", self)
        self.add_action.setCheckable(True)
        self.add_action.setShortcut(QKeySequence("W"))
        self.add_action.toggled.connect(self.canvas.set_add_mode)
        self.delete_action = QAction("Delete Object", self)
        self.delete_action.setShortcut(QKeySequence.Delete)
        self.delete_action.triggered.connect(self._delete_selected)
        self.delete_sample_action = QAction("Delete Image + Label", self)
        self.delete_sample_action.setShortcut(QKeySequence("Ctrl+D"))
        self.delete_sample_action.triggered.connect(self._delete_current_sample)
        self.undo_action = QAction("Undo", self)
        self.undo_action.setShortcut(QKeySequence.Undo)
        self.undo_action.triggered.connect(self.canvas.undo)
        self.edit_class_action = QAction("Edit Class…", self)
        self.edit_class_action.setShortcut(QKeySequence("E"))
        self.edit_class_action.triggered.connect(self._edit_class)
        self.auto_visibility_action = QAction("Auto Visibility", self)
        self.auto_visibility_action.setShortcut(QKeySequence("V"))
        self.auto_visibility_action.triggered.connect(self._auto_visibility)

        self.previous_action = QAction("Previous", self)
        self.previous_action.setShortcuts([QKeySequence(Qt.Key_Left), QKeySequence("A")])
        self.previous_action.triggered.connect(lambda: self._move_selection(-1))
        self.next_action = QAction("Next", self)
        self.next_action.setShortcuts([QKeySequence(Qt.Key_Right), QKeySequence("D")])
        self.next_action.triggered.connect(lambda: self._move_selection(1))

        self.fit_action = QAction("Fit", self)
        self.fit_action.setShortcut(QKeySequence("F"))
        self.fit_action.triggered.connect(self.canvas.fit_to_view)
        self.zoom_in_action = QAction("Zoom In", self)
        self.zoom_in_action.setShortcut(QKeySequence.ZoomIn)
        self.zoom_in_action.triggered.connect(lambda: self.canvas.zoom_by(1.2))
        self.zoom_out_action = QAction("Zoom Out", self)
        self.zoom_out_action.setShortcut(QKeySequence.ZoomOut)
        self.zoom_out_action.triggered.connect(lambda: self.canvas.zoom_by(1.0 / 1.2))
        self.overlay_action = QAction("Show Annotations", self)
        self.overlay_action.setCheckable(True)
        self.overlay_action.setChecked(True)
        self.overlay_action.triggered.connect(self.canvas.set_annotations_visible)

    def _create_toolbar(self) -> None:
        toolbar = QToolBar("Annotation Editor", self)
        toolbar.setMovable(False)
        for action in (self.open_action, self.save_action):
            toolbar.addAction(action)
        toolbar.addSeparator()
        for action in (
            self.add_action,
            self.delete_action,
            self.delete_sample_action,
            self.edit_class_action,
            self.auto_visibility_action,
        ):
            toolbar.addAction(action)
        toolbar.addSeparator()
        for action in (
            self.previous_action,
            self.next_action,
            self.fit_action,
            self.zoom_in_action,
            self.zoom_out_action,
            self.overlay_action,
        ):
            toolbar.addAction(action)
        self.addToolBar(toolbar)

    def _create_menus(self) -> None:
        file_menu = self.menuBar().addMenu("File")
        file_menu.addAction(self.open_action)
        file_menu.addAction(self.save_action)
        file_menu.addSeparator()
        file_menu.addAction("Exit", self.close, QKeySequence.Quit)
        edit_menu = self.menuBar().addMenu("Edit")
        edit_menu.addAction(self.undo_action)
        edit_menu.addSeparator()
        edit_menu.addAction(self.add_action)
        edit_menu.addAction(self.delete_action)
        edit_menu.addAction(self.delete_sample_action)
        edit_menu.addAction(self.edit_class_action)
        edit_menu.addSeparator()
        edit_menu.addAction(self.auto_visibility_action)
        view_menu = self.menuBar().addMenu("View")
        for action in (
            self.previous_action,
            self.next_action,
            self.fit_action,
            self.zoom_in_action,
            self.zoom_out_action,
            self.overlay_action,
        ):
            view_menu.addAction(action)

    def _current_entry(self) -> ImageEntry | None:
        if self.dataset is None or not 0 <= self.current_index < len(self.dataset.entries):
            return None
        return self.dataset.entries[self.current_index]

    def _update_actions(self) -> None:
        entry = self._current_entry()
        editable = entry is not None and entry.annotation_path is not None
        selected = self.canvas.selected_index >= 0
        self.save_action.setEnabled(editable and self.dirty)
        self.add_action.setEnabled(editable)
        self.delete_action.setEnabled(
            editable and self.canvas.has_deletable_selection
        )
        self.delete_sample_action.setEnabled(entry is not None)
        self.undo_action.setEnabled(editable and self.canvas.can_undo)
        self.edit_class_action.setEnabled(editable and selected)
        self.auto_visibility_action.setEnabled(editable and bool(self.canvas.annotations))

    def _update_title(self) -> None:
        name = self.dataset.selected_path.name if self.dataset is not None else ""
        suffix = f" — {name}" if name else ""
        dirty = " *" if self.dirty else ""
        self.setWindowTitle(f"Bounding Box Tool{suffix}{dirty}")

    def _set_dirty(self, dirty: bool) -> None:
        self.dirty = dirty
        self._update_title()
        self._update_actions()

    def _choose_folder(self) -> None:
        directory = QFileDialog.getExistingDirectory(self, "Open image or dataset folder")
        if directory:
            self.open_path(directory)

    def open_path(self, path: str | Path) -> bool:
        if not self._confirm_changes():
            return False
        try:
            dataset = ImageDataset.open(path)
        except (FileNotFoundError, OSError, ValueError) as exc:
            QMessageBox.warning(self, "Cannot open folder", str(exc))
            return False
        self.dataset = dataset
        self.current_index = -1
        self.image_list.blockSignals(True)
        self.image_list.clear()
        for entry in dataset.entries:
            item = QListWidgetItem(entry.display_path)
            item.setToolTip(str(entry.image_path))
            self.image_list.addItem(item)
        self.image_list.blockSignals(False)
        self._set_dirty(False)
        self.image_list.setCurrentRow(0)
        return True

    def _restore_image_row(self) -> None:
        self.image_list.blockSignals(True)
        self.image_list.setCurrentRow(self.current_index)
        self.image_list.blockSignals(False)

    def _show_image(self, index: int) -> None:
        if self.dataset is None or not 0 <= index < len(self.dataset.entries):
            return
        if index == self.current_index:
            return
        if not self._confirm_changes():
            self._restore_image_row()
            return
        entry = self.dataset.entries[index]
        annotations, warnings = load_annotations(
            entry.annotation_path, self.dataset.class_names
        )
        if not self.canvas.set_image(entry.image_path, annotations):
            QMessageBox.warning(self, "Cannot load image", str(entry.image_path))
            self._restore_image_row()
            return
        self.current_index = index
        self._saved_annotations = annotations
        self.canvas.set_annotations_visible(self.overlay_action.isChecked())
        self._refresh_object_list()
        self.annotation_path.setText(
            str(entry.annotation_path)
            if entry.annotation_path is not None
            else "No dataset annotation path"
        )
        self.warning_label.setText("\n".join(warnings))
        self._set_dirty(False)
        self._show_status()

    def _show_status(self, message: str | None = None) -> None:
        entry = self._current_entry()
        if entry is None or self.dataset is None:
            return
        width, height = self.canvas.image_size
        prefix = f"{message}  |  " if message else ""
        self.statusBar().showMessage(
            f"{prefix}{self.current_index + 1}/{len(self.dataset.entries)}  |  "
            f"{width}×{height}  |  objects: {len(self.canvas.annotations)}  |  "
            f"{entry.image_path}"
        )

    def _refresh_object_list(self) -> None:
        selected = self.canvas.selected_index
        self.object_list.blockSignals(True)
        self.object_list.clear()
        for annotation in self.canvas.annotations:
            x, y, width, height = annotation.bbox
            visibility = "/".join(str(point[2]) for point in annotation.keypoints)
            item = QListWidgetItem(annotation.label)
            item.setToolTip(
                f"class={annotation.class_id}  x={x:.4f} y={y:.4f} "
                f"w={width:.4f} h={height:.4f}  visibility={visibility}"
            )
            item.setForeground(class_color(annotation.class_id))
            self.object_list.addItem(item)
        if selected >= 0:
            self.object_list.setCurrentRow(selected)
        self.object_list.blockSignals(False)
        self._update_actions()

    def _select_object_row(self, index: int) -> None:
        self.object_list.blockSignals(True)
        self.object_list.setCurrentRow(index)
        self.object_list.blockSignals(False)
        self._update_actions()

    def canvas_selection_from_list(self, index: int) -> None:
        self.canvas.set_selected_index(index)

    def _annotations_changed(self) -> None:
        self._set_dirty(self.canvas.annotations != self._saved_annotations)
        self._refresh_object_list()
        self._show_status("Unsaved changes" if self.dirty else "Restored")

    def save_current(self) -> bool:
        entry = self._current_entry()
        if entry is None or entry.annotation_path is None:
            return False
        try:
            save_annotations(entry.annotation_path, self.canvas.annotations)
        except OSError as exc:
            QMessageBox.critical(self, "Cannot save annotation", str(exc))
            return False
        self._saved_annotations = self.canvas.annotations
        self._set_dirty(False)
        self._show_status("Saved")
        return True

    def _confirm_changes(self) -> bool:
        if not self.dirty:
            return True
        answer = QMessageBox.question(
            self,
            "Unsaved annotations",
            "Save changes to the current annotation?",
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
            QMessageBox.Save,
        )
        if answer == QMessageBox.Save:
            return self.save_current()
        if answer == QMessageBox.Discard:
            self._set_dirty(False)
            return True
        return False

    def _choose_class_id(self, current: int | None = None) -> int | None:
        class_names = self.dataset.class_names if self.dataset is not None else {}
        if class_names:
            classes = sorted(class_names.items())
            labels = [f"{class_id}: {label}" for class_id, label in classes]
            current_index = next(
                (index for index, (class_id, _label) in enumerate(classes) if class_id == current),
                0,
            )
            selected, accepted = QInputDialog.getItem(
                self,
                "Object class",
                "Class:",
                labels,
                current_index,
                False,
            )
            if not accepted:
                return None
            return classes[labels.index(selected)][0]
        class_id, accepted = QInputDialog.getInt(
            self,
            "Object class",
            "Class ID:",
            value=current or 0,
            min=0,
            max=9999,
        )
        return class_id if accepted else None

    def _add_drawn_box(self, bbox: tuple[float, float, float, float]) -> None:
        class_id = self._choose_class_id()
        if class_id is None:
            return
        center_x, center_y, width, height = bbox
        left, top = center_x - width / 2.0, center_y - height / 2.0
        right, bottom = center_x + width / 2.0, center_y + height / 2.0
        label = (
            self.dataset.class_names.get(class_id, f"class_{class_id}")
            if self.dataset is not None
            else f"class_{class_id}"
        )
        self.canvas.add_annotation(
            PoseAnnotation(
                class_id=class_id,
                label=label,
                bbox=bbox,
                keypoints=(
                    (left, top, 2),
                    (right, top, 2),
                    (right, bottom, 2),
                    (left, bottom, 2),
                ),
            )
        )

    def _edit_class(self) -> None:
        index = self.canvas.selected_index
        if not 0 <= index < len(self.canvas.annotations):
            return
        annotation = self.canvas.annotations[index]
        class_id = self._choose_class_id(annotation.class_id)
        if class_id is None or class_id == annotation.class_id:
            return
        label = (
            self.dataset.class_names.get(class_id, f"class_{class_id}")
            if self.dataset is not None
            else f"class_{class_id}"
        )
        self.canvas.replace_selected(
            replace(annotation, class_id=class_id, label=label)
        )

    def _delete_selected(self) -> None:
        if not self.canvas.has_deletable_selection:
            return
        answer = QMessageBox.question(
            self,
            "Delete annotation",
            "선택한 bbox를 삭제하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.canvas.delete_selected()

    def _delete_current_sample(self) -> None:
        entry = self._current_entry()
        if entry is None or self.dataset is None or self.dataset.dataset_root is None:
            return
        root = self.dataset.dataset_root
        samples_path = root / "samples.jsonl"
        answer = QMessageBox.question(
            self,
            "Delete image and label",
            "해당 데이터를 삭제하시겠습니까?\n\n"
            f"이미지: {entry.image_path}\n"
            f"라벨: {entry.annotation_path or '없음'}\n\n"
            "samples.jsonl을 갱신하고 다음 이미지로 이동합니다.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        try:
            updated_samples = samples_without_image(
                samples_path, root, entry.image_path
            )
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "Cannot update samples.jsonl", str(exc))
            return

        staged = []
        try:
            for path in (entry.image_path, entry.annotation_path):
                if path is None or not path.exists():
                    continue
                temporary = path.with_name(f".{path.name}.deleting")
                if temporary.exists():
                    raise FileExistsError(f"temporary file already exists: {temporary}")
                path.replace(temporary)
                staged.append((path, temporary))
        except OSError as exc:
            for original, temporary in reversed(staged):
                temporary.replace(original)
            QMessageBox.critical(self, "Cannot delete data", str(exc))
            return
        try:
            save_samples(samples_path, updated_samples)
        except OSError as exc:
            for original, temporary in reversed(staged):
                temporary.replace(original)
            QMessageBox.critical(self, "Cannot update samples.jsonl", str(exc))
            return

        cleanup_errors = []
        for _original, temporary in staged:
            try:
                temporary.unlink()
            except OSError as exc:
                cleanup_errors.append(str(exc))

        deleted_index = self.current_index
        entries = self.dataset.entries
        self.dataset = replace(
            self.dataset,
            entries=entries[:deleted_index] + entries[deleted_index + 1 :],
        )
        self.image_list.blockSignals(True)
        self.image_list.takeItem(deleted_index)
        self.image_list.setCurrentRow(-1)
        self.image_list.blockSignals(False)
        self.current_index = -1
        self._saved_annotations = ()
        self._set_dirty(False)
        if cleanup_errors:
            QMessageBox.warning(
                self,
                "Cleanup incomplete",
                "\n".join(cleanup_errors),
            )
        if self.dataset.entries:
            self.image_list.setCurrentRow(
                min(deleted_index, len(self.dataset.entries) - 1)
            )
            return
        self.canvas.clear_image()
        self.object_list.clear()
        self.annotation_path.setText("No annotation")
        self.warning_label.clear()
        self.statusBar().showMessage("No images remain")

    def _auto_visibility(self) -> None:
        # QApplication이 platform plugin을 초기화하기 전에 cv2를 import하면
        # cv2/qt/plugins가 PyQt5의 xcb 경로를 가로챌 수 있어 여기서 지연 import한다.
        from bounding_box_tool.occlusion import apply_auto_visibility

        entry = self._current_entry()
        if entry is None:
            return
        try:
            preserve_fully_occluded = bool(
                self.dataset is not None
                and self.dataset.collection_policy == "near-port"
            )
            result = apply_auto_visibility(
                entry.image_path,
                self.canvas.annotations,
                preserve_fully_occluded=preserve_fully_occluded,
            )
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, "Auto visibility failed", str(exc))
            return
        self.canvas.set_annotations(result.annotations)
        self._show_status(
            f"Auto visibility: deleted={result.deleted_objects}, "
            f"preserved fully occluded={result.preserved_occluded_objects}, "
            f"occluded keypoints={result.occluded_keypoints}"
        )

    def _sync_add_action(self, enabled: bool) -> None:
        self.add_action.blockSignals(True)
        self.add_action.setChecked(enabled)
        self.add_action.blockSignals(False)

    def _move_selection(self, offset: int) -> None:
        if self.image_list.count() == 0:
            return
        current = max(0, self.image_list.currentRow())
        target = min(max(current + offset, 0), self.image_list.count() - 1)
        self.image_list.setCurrentRow(target)

    def closeEvent(self, event) -> None:
        if self._confirm_changes():
            event.accept()
        else:
            event.ignore()

    def dragEnterEvent(self, event) -> None:
        if event.mimeData().hasUrls() and any(
            url.isLocalFile() for url in event.mimeData().urls()
        ):
            event.acceptProposedAction()

    def dropEvent(self, event) -> None:
        for url in event.mimeData().urls():
            if url.isLocalFile() and self.open_path(url.toLocalFile()):
                event.acceptProposedAction()
                return
