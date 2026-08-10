"""LabelImg-style read-only PyQt image and YOLO-pose viewer."""

from __future__ import annotations

from pathlib import Path

from bounding_box_tool.dataset import (ImageDataset, PoseAnnotation,
                                       load_annotations)
from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import (QColor, QKeySequence, QPainter, QPen, QPixmap,
                         QPolygonF)
from PyQt5.QtWidgets import (QAction, QFileDialog, QGraphicsItem,
                             QGraphicsPixmapItem, QGraphicsScene,
                             QGraphicsSimpleTextItem, QGraphicsView,
                             QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
                             QMainWindow, QMessageBox, QSplitter, QStyle,
                             QToolBar, QVBoxLayout, QWidget)

KEYPOINT_COLORS = (
    QColor("#ff3b30"),
    QColor("#ff9500"),
    QColor("#ff2dce"),
    QColor("#007aff"),
)


def class_color(class_id: int) -> QColor:
    """class마다 안정적으로 구분되는 밝은 색을 반환한다."""
    return QColor.fromHsv((class_id * 47 + 110) % 360, 210, 255)


class AnnotationCanvas(QGraphicsView):
    """image 위에 bbox와 keypoint를 그리는 확대·이동 가능한 read-only canvas."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._pixmap_item: QGraphicsPixmapItem | None = None
        self._overlay_items: list[QGraphicsItem] = []
        self._fit_mode = True
        self.setBackgroundBrush(QColor("#202124"))
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setRenderHints(
            self.renderHints() | QPainter.Antialiasing | QPainter.SmoothPixmapTransform
        )

    @property
    def image_size(self) -> tuple[int, int]:
        """현재 image의 pixel 크기를 반환한다."""
        if self._pixmap_item is None:
            return 0, 0
        pixmap = self._pixmap_item.pixmap()
        return pixmap.width(), pixmap.height()

    def set_image(
        self,
        image_path: Path,
        annotations: tuple[PoseAnnotation, ...],
    ) -> bool:
        """image를 교체하고 대응 annotation overlay를 다시 그린다."""
        pixmap = QPixmap(str(image_path))
        if pixmap.isNull():
            return False
        self._scene.clear()
        self._overlay_items.clear()
        self._pixmap_item = self._scene.addPixmap(pixmap)
        self._pixmap_item.setZValue(0)
        self._scene.setSceneRect(QRectF(pixmap.rect()))
        for annotation in annotations:
            self._draw_annotation(annotation, pixmap.width(), pixmap.height())
        self.resetTransform()
        self.fit_to_view()
        return True

    def _draw_annotation(
        self,
        annotation: PoseAnnotation,
        image_width: int,
        image_height: int,
    ) -> None:
        """정규화 bbox, polygon, keypoint 번호와 class label을 그린다."""
        color = class_color(annotation.class_id)
        x_center, y_center, width, height = annotation.bbox
        x = (x_center - width / 2.0) * image_width
        y = (y_center - height / 2.0) * image_height
        rectangle = QRectF(x, y, width * image_width, height * image_height)
        box_pen = QPen(color, 2.5)
        box_pen.setCosmetic(True)
        box_item = self._scene.addRect(rectangle, box_pen)
        box_item.setZValue(10)
        self._overlay_items.append(box_item)

        points = [
            QPointF(keypoint_x * image_width, keypoint_y * image_height)
            for keypoint_x, keypoint_y, visibility in annotation.keypoints
            if visibility > 0
        ]
        if len(points) >= 2:
            polygon_pen = QPen(QColor("#00e5ff"), 2.0)
            polygon_pen.setCosmetic(True)
            polygon_item = self._scene.addPolygon(QPolygonF(points), polygon_pen)
            polygon_item.setZValue(11)
            self._overlay_items.append(polygon_item)

        for index, (keypoint_x, keypoint_y, visibility) in enumerate(
            annotation.keypoints, 1
        ):
            if visibility <= 0:
                continue
            point = QPointF(keypoint_x * image_width, keypoint_y * image_height)
            marker = self._scene.addEllipse(
                -4.5,
                -4.5,
                9.0,
                9.0,
                QPen(Qt.white, 1.5),
                KEYPOINT_COLORS[index - 1],
            )
            marker.setPos(point)
            marker.setFlag(QGraphicsItem.ItemIgnoresTransformations)
            marker.setZValue(13)
            self._overlay_items.append(marker)

            number = QGraphicsSimpleTextItem(str(index))
            number.setBrush(KEYPOINT_COLORS[index - 1])
            number.setPen(QPen(Qt.black, 1.0))
            number.setPos(point + QPointF(6.0, -18.0))
            number.setFlag(QGraphicsItem.ItemIgnoresTransformations)
            number.setZValue(14)
            self._scene.addItem(number)
            self._overlay_items.append(number)

        label = QGraphicsSimpleTextItem(annotation.label)
        label.setBrush(color)
        label.setPen(QPen(Qt.black, 1.5))
        label.setPos(QPointF(rectangle.left(), max(0.0, rectangle.top() - 20.0)))
        label.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        label.setZValue(14)
        self._scene.addItem(label)
        self._overlay_items.append(label)

    def set_annotations_visible(self, visible: bool) -> None:
        """현재 overlay 표시 여부를 전환한다."""
        for item in self._overlay_items:
            item.setVisible(visible)

    def fit_to_view(self) -> None:
        """현재 image가 viewport 안에 맞도록 배율을 조정한다."""
        if self._pixmap_item is None:
            return
        self._fit_mode = True
        self.fitInView(self._pixmap_item, Qt.KeepAspectRatio)

    def zoom_by(self, factor: float) -> None:
        """지정 배율로 확대 또는 축소한다."""
        if self._pixmap_item is None:
            return
        current = self.transform().m11()
        if not 0.03 <= current * factor <= 50.0:
            return
        self._fit_mode = False
        self.scale(factor, factor)

    def wheelEvent(self, event) -> None:
        """mouse wheel로 cursor 중심 확대·축소한다."""
        self.zoom_by(1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15)

    def resizeEvent(self, event) -> None:
        """fit mode에서는 window 크기에 맞춰 image를 다시 배치한다."""
        super().resizeEvent(event)
        if self._fit_mode:
            self.fit_to_view()


class BoundingBoxViewer(QMainWindow):
    """image list, annotation canvas와 객체 목록을 갖는 read-only viewer."""

    def __init__(self, initial_path: str | Path | None = None, parent=None):
        super().__init__(parent)
        self.dataset: ImageDataset | None = None
        self.setWindowTitle("Bounding Box Tool — Read Only")
        self.resize(1500, 900)
        self.setAcceptDrops(True)

        self.image_list = QListWidget()
        self.image_list.setMinimumWidth(280)
        self.image_list.currentRowChanged.connect(self._show_image)
        self.object_list = QListWidget()
        self.object_list.setMinimumWidth(260)
        self.annotation_path = QLabel("No annotation")
        self.annotation_path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.annotation_path.setWordWrap(True)
        self.warning_label = QLabel("")
        self.warning_label.setStyleSheet("color: #ffb74d;")
        self.warning_label.setWordWrap(True)
        self.canvas = AnnotationCanvas()

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
        self._create_toolbar()
        self._create_menus()
        self.statusBar().showMessage("Open a dataset root or image folder")
        if initial_path is not None:
            self.open_path(initial_path)

    @staticmethod
    def _panel(title: str, *widgets: QWidget) -> QWidget:
        """sidebar heading과 widget들을 세로 panel로 묶는다."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        heading = QLabel(title)
        heading.setStyleSheet("font-weight: 600; font-size: 14px;")
        layout.addWidget(heading)
        for widget in widgets:
            layout.addWidget(widget)
        return panel

    def _create_actions(self) -> None:
        """menu와 toolbar가 공유할 read-only action을 생성한다."""
        self.open_action = QAction(
            self.style().standardIcon(QStyle.SP_DialogOpenButton),
            "Open Folder…",
            self,
        )
        self.open_action.setShortcut(QKeySequence.Open)
        self.open_action.triggered.connect(self._choose_folder)

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
        """LabelImg처럼 자주 쓰는 탐색 action을 상단에 배치한다."""
        toolbar = QToolBar("Viewer", self)
        toolbar.setMovable(False)
        for action in (
            self.open_action,
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
        """File/View menu를 생성한다."""
        file_menu = self.menuBar().addMenu("File")
        file_menu.addAction(self.open_action)
        file_menu.addSeparator()
        file_menu.addAction("Exit", self.close, QKeySequence.Quit)
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

    def _choose_folder(self) -> None:
        """사용자가 선택한 dataset 또는 image directory를 연다."""
        directory = QFileDialog.getExistingDirectory(self, "Open image or dataset folder")
        if directory:
            self.open_path(directory)

    def open_path(self, path: str | Path) -> bool:
        """경로를 scan하고 image list를 교체한다."""
        try:
            dataset = ImageDataset.open(path)
        except (FileNotFoundError, OSError, ValueError) as exc:
            QMessageBox.warning(self, "Cannot open folder", str(exc))
            return False
        self.dataset = dataset
        self.image_list.blockSignals(True)
        self.image_list.clear()
        for entry in dataset.entries:
            item = QListWidgetItem(entry.display_path)
            item.setToolTip(str(entry.image_path))
            self.image_list.addItem(item)
        self.image_list.blockSignals(False)
        self.setWindowTitle(
            f"Bounding Box Tool — {dataset.selected_path.name} — Read Only"
        )
        self.image_list.setCurrentRow(0)
        return True

    def _show_image(self, index: int) -> None:
        """선택 image와 annotation을 canvas 및 객체 목록에 표시한다."""
        if self.dataset is None or not 0 <= index < len(self.dataset.entries):
            return
        entry = self.dataset.entries[index]
        annotations, warnings = load_annotations(
            entry.annotation_path, self.dataset.class_names
        )
        if not self.canvas.set_image(entry.image_path, annotations):
            QMessageBox.warning(self, "Cannot load image", str(entry.image_path))
            return
        self.canvas.set_annotations_visible(self.overlay_action.isChecked())
        self.object_list.clear()
        for annotation in annotations:
            x, y, width, height = annotation.bbox
            item = QListWidgetItem(annotation.label)
            item.setToolTip(
                f"class={annotation.class_id}  x={x:.4f} y={y:.4f} "
                f"w={width:.4f} h={height:.4f}"
            )
            item.setForeground(class_color(annotation.class_id))
            self.object_list.addItem(item)
        if not annotations:
            self.object_list.addItem("No objects")
        self.annotation_path.setText(
            str(entry.annotation_path)
            if entry.annotation_path is not None
            else "No dataset annotation path"
        )
        self.warning_label.setText("\n".join(warnings))
        width, height = self.canvas.image_size
        self.statusBar().showMessage(
            f"{index + 1}/{len(self.dataset.entries)}  |  {width}×{height}  |  "
            f"objects: {len(annotations)}  |  {entry.image_path}"
        )

    def _move_selection(self, offset: int) -> None:
        """현재 image list selection을 앞뒤로 이동한다."""
        if self.image_list.count() == 0:
            return
        current = max(0, self.image_list.currentRow())
        target = min(max(current + offset, 0), self.image_list.count() - 1)
        self.image_list.setCurrentRow(target)

    def dragEnterEvent(self, event) -> None:
        """local file/directory drag를 허용한다."""
        if event.mimeData().hasUrls() and any(
            url.isLocalFile() for url in event.mimeData().urls()
        ):
            event.acceptProposedAction()

    def dropEvent(self, event) -> None:
        """첫 local file/directory drop 경로를 연다."""
        for url in event.mimeData().urls():
            if url.isLocalFile():
                self.open_path(url.toLocalFile())
                event.acceptProposedAction()
                return
