#!/usr/bin/env python3.7

import argparse

def parseArguments():
    argparser = argparse.ArgumentParser(
        description='CARLA Platoon')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--count',
        default=2,
        type=int,
        help='Number of vehicles (including leader) (default: 2)')
    argparser.add_argument(
        '--spacing',
        default=15,
        type= int,
        help='Spacings between vehicles at spawn time (default: 15)')
    argparser.add_argument(
        '--timeout',
        default=10.0,
        type=int,
        help='Time to wait for the server before quitting (default: 10.0 seconds)')
    argparser.add_argument(
        '--reload-map',
        action='store_true',
        help='Reload the map (default: False)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.tesla.cybertruck',
        help='actor filter (default: "vehicle.tesla.model3")')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')

    args = argparser.parse_args()

    if args.spacing <0:
        print("spacing negative number")
        print("Exiting...")
        exit()
    if args.timeout<0:
        print("timeout negative number")
        print("Exiting...")
        exit()
    return args
