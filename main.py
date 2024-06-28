#!/usr/bin/env python3.7

from world import World
import ourmqtt
import json


def main():
    simWorld = World()
    try:
        simWorld.spawnPlatoon(record=True)
        client = ourmqtt.initComms()
        while True:
            simWorld.applyPlatoonControls(
                json.loads(
                    ourmqtt.sendDataGetControls(
                        client,
                        json.dumps(simWorld.getPlatoonData()),
                    )
                )
            )
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
