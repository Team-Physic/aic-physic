"""전체 task board 시야에서 수집하는 AIC policy 진입점."""

from phy_policy.ros.PortOffsetCollect import PortOffsetCollect


class BoardViewCollect(PortOffsetCollect):
    """보드 전경 거리와 카메라 각도를 무작위화해 수집한다."""

    collection_policy = "board-view"
