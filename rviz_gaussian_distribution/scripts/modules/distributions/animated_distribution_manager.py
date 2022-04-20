import time

from . import DistributionManager
from .modules import Distribution, AnimatedDistribution


class AnimatedDistributionManager(DistributionManager):
    """
    すべての分布を管理するクラスのアニメーション用．
    """

    def __init__(self, animated_time=30):
        """
        Args:
            animated_time (int):

        """
        super(AnimatedDistributionManager, self).__init__()
        self._animated_time = animated_time
        self._before_time = 0

    def register(self, id_, distribution):
        """
        分布を指定のIDで登録．

        Args:
            id_ (int):
            distribution (Distribution):

        """
        if id_ in self._distribution_dict:
            raise ValueError(f"ID={id_} is already set.")

        self._distribution_dict[id_] = AnimatedDistribution(
            distribution.xyz, distribution.rgb, distribution.alpha, self._animated_time
        )

    def update(self):
        """
        呼び出す毎に前回の呼び出し時からの時間差を元に

        Returns:

        """
        if self._before_time == 0:
            # 呼び出し1回目の初期化
            self._before_time = time.time()

        now_time = time.time()
        diff = now_time - self._before_time
        for animator in self._distribution_dict.copy().values():
            animator.update(diff)

        self._before_time = now_time

    def is_complete(self):
        conditions = set([animator.is_complete() for animator in self._distribution_dict.copy().values()])
        if False not in conditions:
            return True
        return False
