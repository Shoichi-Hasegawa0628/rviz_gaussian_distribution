from .modules import Distribution


class DistributionManager:
    """
    すべての分布を管理するクラス
    """

    def __init__(self):
        self._distribution_dict = {}

    def is_empty(self):
        """
        分布が1つも登録されていないかどうか．

        Returns:
            bool:

        """
        return len(self._distribution_dict) == 0

    def register(self, id_, distribution):
        """
        分布を指定のIDで登録．

        Args:
            id_ (int):
            distribution (VisualizedDistribution):

        """
        self._distribution_dict[id_] = distribution

    def unregister(self, id_):
        """
        登録されている指定のIDの分布を削除．

        Args:
            id_ (int):

        """
        if id_ not in self._distribution_dict:
            raise ValueError(f"ID={id_} is not set.")

        self._distribution_dict.pop(id_)

    def clear(self):
        """
        登録されているすべての分布を削除．

        """
        self._distribution_dict.clear()

    def get_distributions(self):
        """
        すべての分布をlist形式で返す．

        Returns:
            list[Distribution]:

        """
        return list(self._distribution_dict.values())
