import speedslide_crack
import common.geom as geom
from tminterface.interface import TMInterface
from tminterface.client import Client, run_client
import tminterface.constants
import sys
import struct

class MainClient(Client):
    def __init__(self) -> None:
        super(MainClient, self).__init__()

    def on_registered(self, iface: TMInterface) -> None:
        print(f'Registered to {iface.server_name}')

    def on_run_step(self, iface: TMInterface, _time: int):
        if _time >= 0:
            state = iface.get_simulation_state()

            wheels_in_air = True
            wheel_size = tminterface.constants.SIMULATION_WHEELS_SIZE // 4
            for i in range(4):
                current_offset = wheel_size * i
                hasgroundcontact = struct.unpack('i', state.simulation_wheels[current_offset+292:current_offset+296])[0]
                wheels_in_air &= (hasgroundcontact == 0)

            if wheels_in_air:
                return

            # rotation matrix
            xx = struct.unpack('f', state.dyna[464:468])[0]
            xy = struct.unpack('f', state.dyna[468:472])[0]
            xz = struct.unpack('f', state.dyna[472:476])[0]

            yx = struct.unpack('f', state.dyna[476:480])[0]
            yy = struct.unpack('f', state.dyna[480:484])[0]
            yz = struct.unpack('f', state.dyna[484:488])[0]

            zx = struct.unpack('f', state.dyna[488:492])[0]
            zy = struct.unpack('f', state.dyna[492:496])[0]
            zz = struct.unpack('f', state.dyna[496:500])[0]

            # velocity
            vx = struct.unpack('f', state.dyna[512:516])[0]
            vy = struct.unpack('f', state.dyna[516:520])[0]
            vz = struct.unpack('f', state.dyna[520:524])[0]

            speed = geom.GmVec3(vx, vy, vz).MultTranspose(geom.GmMat3(xx, xy, xz, yx, yy, yz, zx, zy, zz))
            # speed.x : speed sidewards
            # speed.y : speed upwards
            # speed.z : speed forwards

            quality = speedslide_crack.GetSpeedslideQualityForStadiumCar(speed.x, speed.z)
            if quality > 0.0:
                print(f"{_time} | {quality}")

def main():
    server_name = f'TMInterface{sys.argv[1]}' if len(sys.argv) > 1 else 'TMInterface0'
    print(f'Connecting to {server_name}...')
    run_client(MainClient(), server_name)

if __name__ == '__main__':
    main()
    