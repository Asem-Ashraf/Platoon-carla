#!/usr/bin/env python3.7

from Simulator.world import World
import Communication.ourmqtt as ourmqtt


def main():
    simWorld = World()
    try:
        simWorld.world.tick()
        simWorld.world.tick()
        simWorld.spawnPlatoon(record=True)
        simWorld.world.tick()
        simWorld.world.tick()
        ourmqtt.initComms()
        while True:
            simWorld.applyPlatoonControls(ourmqtt.sendDataGetControls(simWorld.getPlatoonData()))
            simWorld.world.tick()
    finally:
        simWorld.destroyPlatoon()
        print("\n==================\nPlatoon Destroyed.\n==================")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExit by user using keyboard interrupt. Bye!")
    except RuntimeError as e:
        print("\n\n\nUnexpected Runtime error:", e)
