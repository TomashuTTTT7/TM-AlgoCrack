import math

def SolveQuadratic(a: float, b: float, c: float) -> tuple[int, float, float]:
    delta = b*b - 4*a*c
    if delta < 0.0:
        return 0, 0, 0
    elif delta == 0.0:
        return 1, (-0.5*b)/a, 0
    else:
        sqrt_delta = math.sqrt(delta)
        return 2, (0.5 * (-b - sqrt_delta)) / a, (0.5 * (-b + sqrt_delta)) / a

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

class GmQuat:
    def __init__(self, w: float = 1.0, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def SetMult(self, other):
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w

        self.w = w
        self.x = x
        self.y = y
        self.z = z
        return self

    def Mult(self, other):
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w

        return GmQuat(w, x, y, z)

    def GetMat(self) -> GmMat3:
        xx = 1.0 - 2.0 * self.y * self.y - 2.0 * self.z * self.z
        xy = 2.0 * self.x * self.y - 2.0 * self.z * self.w
        xz = 2.0 * self.x * self.z + 2.0 * self.y * self.w
        yx = 2.0 * self.x * self.y + 2.0 * self.z * self.w
        yy = 1.0 - 2.0 * self.x * self.x - 2.0 * self.z * self.z
        yz = 2.0 * self.z * self.y - 2.0 * self.x * self.w
        zx = 2.0 * self.z * self.x - 2.0 * self.y * self.w
        zy = 2.0 * self.z * self.y + 2.0 * self.x * self.w
        zz = 1.0 - 2.0 * self.x * self.x - 2.0 * self.y * self.y

        return GmMat3(xx, xy, xz, yx, yy, yz, zx, zy, zz)

    def SetNorm(self):
        square = self.w * self.w + self.x * self.x + self.y * self.y + self.z + self.z
        inv = 1 / math.sqrt(square)

        self.w *= inv
        self.x *= inv
        self.y *= inv
        self.z *= inv
        return self

    def Norm(self):
        square = self.w * self.w + self.x * self.x + self.y * self.y + self.z + self.z
        inv = 1 / math.sqrt(square)

        return GmVec3(self.w * inv, self.x * inv, self.y * inv, self.z * inv)

class GmVec3:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def SetMult(self, matrix: GmMat3):
        x = self.x * matrix.xx + self.y * matrix.xy + self.z * matrix.xz
        y = self.x * matrix.yx + self.y * matrix.yy + self.z * matrix.yz
        z = self.x * matrix.zx + self.y * matrix.zy + self.z * matrix.zz

        self.x = x
        self.y = y
        self.z = z
        return self

    def SetMultTranspose(self, matrix: GmMat3):
        x = self.x * matrix.xx + self.y * matrix.yx + self.z * matrix.zx
        y = self.x * matrix.xy + self.y * matrix.yy + self.z * matrix.zy
        z = self.x * matrix.xz + self.y * matrix.yz + self.z * matrix.zz

        self.x = x
        self.y = y
        self.z = z
        return self

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

    def SetNorm(self):
        square = self.x * self.x + self.y * self.y + self.z + self.z
        inv = 1 / math.sqrt(square)

        self.x *= inv
        self.y *= inv
        self.z *= inv
        return self

    def Norm(self):
        square = self.x * self.x + self.y * self.y + self.z + self.z
        inv = 1 / math.sqrt(square)

        return GmVec3(self.x * inv, self.y * inv, self.z * inv)

    def Length(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

class GmVec2:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y
    
    def Length(self):
        return math.sqrt(self.x * self.x + self.y * self.y)
