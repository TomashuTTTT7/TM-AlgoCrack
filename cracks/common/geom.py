class GmMat3:
    def __init__(self, xx: float = 1.0, xy: float = 0.0, xz: float = 0.0, yx: float = 0.0, yy: float = 1.0, yz: float = 0.0, zx: float = 0.0, zy: float = 0.0, zz: float = 1.0):
        self.xx = xx
        self.xy = xy
        self.xz = xz

        self.yx = yx
        self.yy = yy
        self.yz = yz

        self.zx = zx
        self.zy = zy
        self.zz = zz

class GmVec3:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def SetMult(self, matrix: GmMat3) -> None:
        x = self.x * matrix.xx + self.y * matrix.xy + self.z * matrix.xz
        y = self.x * matrix.yx + self.y * matrix.yy + self.z * matrix.yz
        z = self.x * matrix.zx + self.y * matrix.zy + self.z * matrix.zz

        self.x = x
        self.y = y
        self.z = z

    def SetMultTranspose(self, matrix: GmMat3) -> None:
        x = self.x * matrix.xx + self.y * matrix.yx + self.z * matrix.zx
        y = self.x * matrix.xy + self.y * matrix.yy + self.z * matrix.zy
        z = self.x * matrix.xz + self.y * matrix.yz + self.z * matrix.zz

        self.x = x
        self.y = y
        self.z = z

    def Mult(self, matrix: GmMat3):
        x = self.x * matrix.xx + self.y * matrix.xy + self.z * matrix.xz
        y = self.x * matrix.yx + self.y * matrix.yy + self.z * matrix.yz
        z = self.x * matrix.zx + self.y * matrix.zy + self.z * matrix.zz

        return GmVec3(x, y, z)

    def MultTranspose(self, matrix: GmMat3):
        x = self.x * matrix.xx + self.y * matrix.yx + self.z * matrix.zx
        y = self.x * matrix.xy + self.y * matrix.yy + self.z * matrix.zy
        z = self.x * matrix.xz + self.y * matrix.yz + self.z * matrix.zz

        return GmVec3(x, y, z)

