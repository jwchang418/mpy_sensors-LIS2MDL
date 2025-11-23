import lis2mdl
import time

mg = lis2mdl.LIS2MDL()
mg.set_cfg_a(comp_temp_en=True,md=0x00)
print(f'id = {mg.get_who_am_i()}')
while True:
    print(f'mag xyz = {mg.get_mag_xyz()}')
    time.sleep(0.1)