"""포트 근처 img2pos 데이터를 수집하는 AIC policy 진입점."""

from phy_policy.ros.PortOffsetCollect import PortOffsetCollect


class NearPortCollect(PortOffsetCollect):
    """기존 coarse/near tier 분포로 포트 근처 데이터를 수집한다."""

    collection_policy = "near-port"
