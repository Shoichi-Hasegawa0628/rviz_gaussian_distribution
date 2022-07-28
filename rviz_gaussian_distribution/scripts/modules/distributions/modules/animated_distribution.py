from . import Distribution


class AnimatedDistribution(Distribution):
    """
    可視化用の情報を持つDistributionクラスをAnimation用に拡張したクラス．

    """

    def __init__(self, xyz, rgb, alpha, time):
        """

        Args:
            xyz (np.ndarray): [N, 3]
            rgb (np.ndarray): [N, 3]
            alpha (np.ndarray): [N, 1]
            time (float):

        """
        super(AnimatedDistribution, self).__init__(xyz, rgb, alpha)
        self._per_xyz = self._xyz / time
        self._per_xyz[..., 0:2] = 0

        self._copied_xyz = self._xyz.copy()
        self._xyz[..., 2] = 0

        self._now_time = 0
        self._target_time = time
        self._is_complete = False

    def update(self, time):
        """
        アニメーション用に一定間隔のフレーム毎に更新．
        timeはManagerクラスから与えられる．

        Args:
            time (float): 前回の更新から経過した秒数

        Returns:

        """
        if self._is_complete:
            return

        self._now_time += time
        if self._now_time > self._target_time:
            self._is_complete = True
            self._xyz = self._copied_xyz
        else:
            self._xyz += (self._per_xyz * time)

    def is_complete(self):
        return self._is_complete
