class CFuncKeysReal:
    def __init__(self, keys: list) -> None:
        self.Xs = list()
        self.Ys = list()
        for key in keys:
            self.Xs.append(key[0])
            self.Ys.append(key[1])

    def GetBoundingIndices(self, x: float) -> tuple[int, int]:
        size = len(self.Xs)
        if size == 0:
            return -1, -1
        if size == 1:
            return 0, 0
        idx = size - 1
        rhs = 0
        if x <= self.Xs[idx] + 1e-05:
            lhs = 0
            for ix in self.Xs:
                lhs = rhs
                rhs += 1
                if rhs >= size:
                    rhs = 0
                if x > (self.Xs[lhs] - 1e-05) and x < (self.Xs[rhs] + 1e-05):
                    return lhs, rhs
            return lhs, rhs
        else:
            return idx, idx

    def ComputeBlendCoef(self, x: float) -> tuple[int, int, int, float]:
        lhs, rhs = self.GetBoundingIndices(x)
        if lhs == -1:
            return 0, lhs, rhs, -1
        if lhs == rhs:
            return 1, lhs, rhs, 0
        xlhs = self.Xs[lhs]
        xrhs = self.Xs[rhs]
        diff = xrhs - xlhs
        if abs(diff) < 1e-05:
            return 1, lhs, rhs, 0
        return 1, lhs, rhs, ((x - xlhs) / diff)

    #interpolation: 0 = linear, 1 = none
    def GetValue(self, x: float, interpolation: int) -> float:
        if (interpolation == 1): # none
            status, lhs, rhs, blend = self.ComputeBlendCoef(x)
            if (status != 0):
                return self.Ys[lhs]
            else:
                return 0
        else: # linear
            status, lhs, rhs, blend = self.ComputeBlendCoef(x)
            if (status != 0):
                return self.Ys[rhs] * blend + (1.0 - blend) * self.Ys[lhs]
            else:
                return 0

    def GetSlope(self, x: float) -> tuple[float, float]:
        lhs, rhs = self.GetBoundingIndices(x)
        if lhs == -1:
            return 0.0, 0.0
        if lhs == rhs:
            return 0.0, self.Ys[lhs]
        lx, ly = self.Xs[lhs], self.Ys[lhs]
        rx, ry = self.Xs[rhs], self.Ys[rhs]
        a = (ly - ry)/(lx - rx)
        b = ly - a*lx
        return a, b

    def Scale(self, sx: float, sy: float):
        for i in range(len(self.Xs)):
            self.Xs[i] /= sx
        for i in range(len(self.Ys)):
            self.Ys[i] *= sy
    
    def GetInverse(self):
        inverse = CFuncKeysReal([])
        inverse.Xs = self.Ys
        inverse.Ys = self.Xs
        return inverse

                