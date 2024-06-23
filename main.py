#!/usr/bin/env python3.7

from world import World
import ourmqtt

def main():
    sim_world = World()

    try:
        sim_world.spawn_platoon()
        ourmqtt.initialize_communication()
        while True:
            data = sim_world.get_platoon_data()
            ourmqtt.send_data(data)
            sim_world.apply_control(ourmqtt.get_controls())

    finally:
        sim_world.destroy_platoon()
        print('\n==================')
        print(  'Platoon Destroyed.')
        print(  '==================')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except RuntimeError as e:
        print(e)
