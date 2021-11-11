import common.funckeysreal as funckeysreal

# material_max_side_friction_multiplier
# for platform and road: 1.0
# for dirt:              0.25
# for grass:             0.15
# for air:               0.0

def GetFuncAbsoluteSpeedXFromSpeedForStadiumCar(material_max_side_friction_multiplier = 1.0) -> funckeysreal.CFuncKeysReal:
    MaxSideFrictionFromSpeed = funckeysreal.CFuncKeysReal([[0,80],[100,80],[200,75],[300,67],[400,60],[500,55]])
    MaxSideFrictionFromSpeed.Scale(3.6, 0.1 * material_max_side_friction_multiplier)
    return MaxSideFrictionFromSpeed

def GetPerfectSpeedslideAbsoluteLocalSpeedXForStadiumCar(speed_z: float, material_max_side_friction_multiplier = 1.0) -> float:
    MaxSideFrictionFromSpeed = funckeysreal.CFuncKeysReal([[0,80],[100,80],[200,75],[300,67],[400,60],[500,55]]) # init maximum side friction function points
    return (MaxSideFrictionFromSpeed.GetValue(speed_z * 3.6, 0) * material_max_side_friction_multiplier) / 10

# minimum speed_x to start gaining speed
def GetMinimumSpeedslideAbsoluteLocalSpeedXForStadiumCar(speed_z: float, material_max_side_friction_multiplier = 1.0) -> float:
    return GetPerfectSpeedslideAbsoluteLocalSpeedXForStadiumCar(speed_z, material_max_side_friction_multiplier) / 2

# the closer to 1.0, the better
def GetSpeedslideQualityForStadiumCar(speed_x: float, speed_z: float, material_max_side_friction_multiplier = 1.0) -> float:
    MaxSideFrictionFromSpeed = funckeysreal.CFuncKeysReal([[0,80],[100,80],[200,75],[300,67],[400,60],[500,55]]) # init maximum side friction function points
    maxsidefriction = MaxSideFrictionFromSpeed.GetValue(speed_z * 3.6, 0) * material_max_side_friction_multiplier
    sidefriction = 20 * abs(speed_x)
    if sidefriction > maxsidefriction:
        return (sidefriction - maxsidefriction) / maxsidefriction
    return 0.0
    