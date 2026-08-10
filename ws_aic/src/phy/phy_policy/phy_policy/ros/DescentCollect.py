"""보드 전경에서 포트 근처까지 내려오며 수집하는 AIC policy 진입점."""

from phy_policy.ros.PortOffsetCollect import PortOffsetCollect


class DescentCollect(PortOffsetCollect):
    """먼 거리부터 안전거리까지 순차 하강하며 수집한다."""

    collection_policy = "descent"
