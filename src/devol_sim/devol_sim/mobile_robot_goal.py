
class MobileRobotGoal:
    def __init__(self, name: str, x: float, y: float, z: float, yaw: float):
        self._name: str = name
        self._x: float = x
        self._y: float = y
        self._z: float = z
        self._yaw: float = yaw

    @property
    def name(self):
        return self._name

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    @property
    def yaw(self):
        return self._yaw

    def __str__(self) -> str:
        return (f"MobileRobotGoal: {self.name}\n"
            f"  Position: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f})\n"
            f"  Yaw: {self.yaw:.2f} rad")