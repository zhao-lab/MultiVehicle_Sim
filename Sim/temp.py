import os
import glob
import sys

try:
    sys.path.append(glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time

if __name__=='__main__':
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(3.0)
        world = client.get_world()
        previous_settings = world.get_settings()
        world.apply_settings(carla.WorldSettings(
            synchronous_mode=True,
            fixed_delta_seconds=1.0 / 10.0))
        spawn_points = world.get_map().get_spawn_points()
     
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.toyota.*'))
        actors = []
        print("number of spawn points: {}".format(len(spawn_points)))
        for i in range(len(spawn_points)): 
            actor = world.try_spawn_actor(vehicle_bp, spawn_points[i % len(spawn_points)])
            if actor==None:
                continue
            actor.set_autopilot()
            actors.append(actor)
        last_number = 0
        while True:
            frame = world.tick()
            # print(frame - last_number == 1)
            last_number = frame
            # time.sleep(0.3)
    finally:
        for actor in actors:
            actor.destroy()
        world.apply_settings(previous_settings)