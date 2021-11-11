# speedslide predict bruteforce script by TomashuTTTT7
# Special thanks to Skycrafter for support and making the script from steer prediction algorithm

import struct
from common.funckeysreal import CFuncKeysReal
import speedslide_crack
import common.geom as geom
from tminterface.interface import TMInterface
from tminterface.client import Client, run_client
import tminterface.constants
import sys
import struct
import math

# Constants. Do not modify!
dt = 0.01
SteerTorqueFromSpeed = CFuncKeysReal([[-100,7],[0,16],[50,10.4],[100,8.5],[150,6.8],[200,5.5],[300,4.25],[400,4],[500,3.75],[600,3.75]])
time_diff = 20

class StateHistoryRecord:
    def __init__(self) -> None:
        self.local_speed = 0.0
        self.local_speedz = 0.0
        self.local_avely = 0.0
        self.rotation = geom.GmMat3()
        self.inverse_inertia = geom.GmMat3()
        self.turning_rate = 0.0

class MainClient(Client):
    def __init__(self) -> None:
        super(MainClient, self).__init__()
        self.state = None
        self.race_time = 0
        self.min = -1
        self.max = -1
        self.current_step = self.min
        self.inputs = {}
        self.records = {}
        self.prev = None
        self.iterations = 0

    def on_registered(self, iface: TMInterface) -> None:
        print(f'Registered to {iface.server_name}')

    def on_simulation_begin(self, iface: TMInterface):
        iface.remove_state_validation()
        iface.register_custom_command('sd')
        
    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        if command == 'sd':
            self.min = time_from + 10
            self.max = time_to + 10
            self.current_step = self.min

    def on_simulation_step(self, iface: TMInterface, _time: int):
        self.race_time = _time
        if self.race_time >= 0:
            self.state = iface.get_simulation_state()
            if self.race_time + 10 in self.inputs:
                iface.set_input_state(accelerate=self.state.input_accelerate, brake=self.state.input_brake, steer=self.inputs[self.race_time + 10])

            if self.race_time == self.current_step - time_diff:
                self.prev = self.state

            # rotation matrix
            xx = struct.unpack('f', self.state.dyna[464:468])[0]
            xy = struct.unpack('f', self.state.dyna[468:472])[0]
            xz = struct.unpack('f', self.state.dyna[472:476])[0]
            yx = struct.unpack('f', self.state.dyna[476:480])[0]
            yy = struct.unpack('f', self.state.dyna[480:484])[0]
            yz = struct.unpack('f', self.state.dyna[484:488])[0]
            zx = struct.unpack('f', self.state.dyna[488:492])[0]
            zy = struct.unpack('f', self.state.dyna[492:496])[0]
            zz = struct.unpack('f', self.state.dyna[496:500])[0]
            rot = geom.GmMat3(xx, xy, xz, yx, yy, yz, zx, zy, zz)

            # velocity
            vx = struct.unpack('f', self.state.dyna[512:516])[0]
            vy = struct.unpack('f', self.state.dyna[516:520])[0]
            vz = struct.unpack('f', self.state.dyna[520:524])[0]
            speed = geom.GmVec3(vx, vy, vz).MultTranspose(rot)

            # angular velocity
            ax = struct.unpack('f', self.state.dyna[536:540])[0]
            ay = struct.unpack('f', self.state.dyna[540:544])[0]
            az = struct.unpack('f', self.state.dyna[544:548])[0]
            ang = geom.GmVec3(ax, ay, az).MultTranspose(rot)

            # inverse inertia tensor
            iixx = struct.unpack('f', self.state.dyna[572:576])[0]
            iixy = struct.unpack('f', self.state.dyna[576:580])[0]
            iixz = struct.unpack('f', self.state.dyna[580:584])[0]
            iiyx = struct.unpack('f', self.state.dyna[584:588])[0]
            iiyy = struct.unpack('f', self.state.dyna[588:592])[0]
            iiyz = struct.unpack('f', self.state.dyna[592:596])[0]
            iizx = struct.unpack('f', self.state.dyna[596:600])[0]
            iizy = struct.unpack('f', self.state.dyna[600:604])[0]
            iizz = struct.unpack('f', self.state.dyna[604:608])[0]
            ii = geom.GmMat3(iixx, iixy, iixz, iiyx, iiyy, iiyz, iizx, iizy, iizz)

            record = StateHistoryRecord()
            record.local_avely = ang.y
            record.local_speed = speed.Length()
            record.local_speedz = speed.z
            record.inverse_inertia = ii
            record.rotation = rot
            record.turning_rate = struct.unpack('f', self.state.scene_mobil[1512:1516])[0]
            self.records[_time] = record

            if self.max >= self.race_time - time_diff >= self.min and self.race_time - time_diff == self.current_step:

                wheels_in_air = True
                wheel_size = tminterface.constants.SIMULATION_WHEELS_SIZE // 4
                for i in range(4):
                    current_offset = wheel_size * i
                    hasgroundcontact = struct.unpack('i', self.state.simulation_wheels[current_offset+292:current_offset+296])[0]
                    wheels_in_air &= (hasgroundcontact == 0)

                if wheels_in_air:
                    return

                if abs(speed.x) >= speedslide_crack.GetMinimumSpeedslideAbsoluteLocalSpeedXForStadiumCar(speed.z): # speedslide condition
                    target_speedz = speed.z
                    AbsoluteSpeedXFromSpeed = speedslide_crack.GetFuncAbsoluteSpeedXFromSpeedForStadiumCar()
                    while True:
                        a, b = AbsoluteSpeedXFromSpeed.GetSlope(target_speedz)
                        horizontal_speed = geom.GmVec2(speed.x, speed.z)
                        len = horizontal_speed.Length()
                        _a = 1 + a*a
                        _b = 2*a*b
                        _c = b*b - len*len
                        solutions, z1, z2 = geom.SolveQuadratic(_a, _b, _c)

                        if solutions > 0:
                            target_speedz = max(z1, z2)
                        else:
                            break

                        a2, b2 = AbsoluteSpeedXFromSpeed.GetSlope(target_speedz)
                        if a == a2 and b == b2: # check if on the same slope, if not, adjust to different slope
                            break

                    if target_speedz != speed.z:
                        theta1 = math.acos(speed.z/len)
                        theta2 = math.acos(target_speedz/len)

                        diff_angle = theta2 - theta1 # It probably does only work for flat surfaces, but works for any speed
                        # the precise equation to make it work always is extremely long and hard to simplify
                        if speed.x > 0.0:
                            diff_angle = -diff_angle # it has to work on left and right speedslide

                        prev_angle = self.records[_time - 10].local_avely * dt
                        # compensate angular velocity integration algorithm. Might lose accuracy if other torques are applied.
                        delta_angular_velocity_y = (2.0 * math.tan(math.atan(prev_angle * 0.5) + 0.5 * diff_angle) - prev_angle) / dt
                        delta_angular_acceleration_y = delta_angular_velocity_y / dt
                        
                        ang_accel_scale = geom.GmVec3(0.0, 1.0, 0.0)
                        ang_accel_scale.SetMult(self.records[_time - 20].rotation) # local torque -> global torque
                        ang_accel_scale.SetMult(self.records[_time - 20].inverse_inertia) # torque -> acceleration
                        ang_accel_scale.SetMultTranspose(self.records[_time - 20].rotation) # global acceleration -> local acceleration
                        delta_torque_y = delta_angular_acceleration_y / ang_accel_scale.y
                        
                        steer_torque_from_speed = SteerTorqueFromSpeed.GetValue(self.records[_time - 20].local_speedz * 3.6, 0)
                        x840 = struct.unpack('f', self.state.scene_mobil[2112:2116])[0] # 2.98759102821 for car. Some kind of steerability
                        
                        speed_value = self.records[_time - 20].local_speed
                        if speed_value < 0.7:
                            speed_steer_multiplier = 0.0
                        elif speed_value > 30.0:
                            speed_steer_multiplier = 1.0
                        else:
                            speed_steer_multiplier = math.sin((speed_value * math.pi * 0.5) / 30.0)

                        delta_turning_rate = -delta_torque_y / (speed_steer_multiplier * steer_torque_from_speed * x840)
                        new_turning_rate = self.records[_time - 10].turning_rate + delta_turning_rate # remember we calculated it for 3 ticks earlier

                        new_steer = int(new_turning_rate * 65536)

                        # limit range
                        if new_steer > 65536:
                            new_steer = 65536
                        if new_steer < -65536:
                            new_steer = -65536
                        
                        self.inputs[self.race_time - time_diff] = new_steer
                        self.iterations += 1

                if self.iterations == 1:
                    self.current_step += 10
                    self.iterations = 0
                    print(f'{_time - time_diff - 10} steer {self.inputs[self.race_time - time_diff]}')
                iface.rewind_to_state(self.prev)

    def on_simulation_end(self, iface, result: int):
        self.state = None
        self.race_time = 0
        self.current_step = self.min
        self.inputs = {}
        self.prev = None
        self.iterations = 0

def main():
    server_name = f'TMInterface{sys.argv[1]}' if len(sys.argv) > 1 else 'TMInterface0'
    print(f'Connecting to {server_name}...')
    run_client(MainClient(), server_name)

if __name__ == '__main__':
    main()
