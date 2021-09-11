import common.funckeysreal as funckeysreal

# material_side_friction_multiplier
# for platform and road: 1.0
# for dirt:              0.25
# for grass:             0.15
# for air:               0.0

def GetPerfectSpeedslideAbsoluteLocalSpeedXForStadiumCar(speed_z: float, material_side_friction_multiplier = 1.0) -> float:
    MaxSideFrictionFromSpeed = funckeysreal.CFuncKeysReal([[0,80],[100,80],[200,75],[300,67],[400,60],[500,55]]) # init maximum side friction function points
    return (MaxSideFrictionFromSpeed.GetValue(speed_z * 3.6, 0) * material_side_friction_multiplier) / 10

# minimum speed_x to start gaining speed
def GetMinimumSpeedslideAbsoluteLocalSpeedXForStadiumCar(speed_z: float, material_side_friction_multiplier = 1.0) -> float:
    return GetPerfectSpeedslideAbsoluteLocalSpeedXForStadiumCar(speed_z, material_side_friction_multiplier) / 2

# the closer to 1.0, the better
def GetSpeedslideQualityForStadiumCar(speed_x: float, speed_z: float, material_side_friction_multiplier = 1.0) -> float:
    MaxSideFrictionFromSpeed = funckeysreal.CFuncKeysReal([[0,80],[100,80],[200,75],[300,67],[400,60],[500,55]]) # init maximum side friction function points
    maxsidefriction = MaxSideFrictionFromSpeed.GetValue(speed_z * 3.6, 0) * material_side_friction_multiplier
    sidefriction = 20 * abs(speed_x)
    if sidefriction > maxsidefriction:
        return (sidefriction - maxsidefriction) / maxsidefriction
    return 0.0
    