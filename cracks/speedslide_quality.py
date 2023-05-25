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
        self.mode = 'tmiconsole'

    def on_registered(self, iface: TMInterface) -> None:
        print(f'Registered to {iface.server_name}')
        iface.register_custom_command('sd_quality')
        
    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        if command == 'sd_quality':
            if len(args) >= 1:
                if args[0] == 'tmiconsole' or args[0] == 'extconsole':
                    self.mode=args[0]
                    iface.log('[SDCrack] Settings successfully changed', 'success')
                else:
                    iface.log('[SDCrack] Syntax: sd_quality <tmiconsole/extconsole>', 'error')
            else:
                iface.log('[SDCrack] Syntax: sd_quality <tmiconsole/extconsole>', 'error')

    def on_run_step(self, iface: TMInterface, _time: int):
        if _time >= 0:
            state = iface.get_simulation_state()

            wheels_in_air = True
            for simulation_wheel in state.simulation_wheels:
                wheels_in_air &= (simulation_wheel.real_time_state.has_ground_contact == 0)

            if wheels_in_air:
                return

            rotation = state.dyna.current_state.rotation.to_numpy()
            linear_speed = state.dyna.current_state.linear_speed.to_numpy()

            xx = rotation[0][0]
            xy = rotation[0][1]
            xz = rotation[0][2]
            yx = rotation[1][0]
            yy = rotation[1][1]
            yz = rotation[1][2]
            zx = rotation[2][0]
            zy = rotation[2][1]
            zz = rotation[2][2]

            vx = linear_speed[0]
            vy = linear_speed[1]
            vz = linear_speed[2]

            speed = geom.GmVec3(vx, vy, vz).MultTranspose(geom.GmMat3(xx, xy, xz, yx, yy, yz, zx, zy, zz))
            # speed.x : speed sidewards
            # speed.y : speed upwards
            # speed.z : speed forwards

            quality = speedslide_crack.GetSpeedslideQualityForStadiumCar(speed.x, speed.z)
            if quality > 0.0:
                if self.mode == 'tmiconsole':
                    iface.log(f"{_time} | {quality}")
                else:
                    print(f"{_time} | {quality}")

def main():
    server_name = f'TMInterface{sys.argv[1]}' if len(sys.argv) > 1 else 'TMInterface0'
    print(f'Connecting to {server_name}...')
    run_client(MainClient(), server_name)

if __name__ == '__main__':
    main()
    
