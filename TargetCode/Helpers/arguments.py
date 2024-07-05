#!/usr/bin/env python3.7

import argparse

def parseArguments():
    argparser = argparse.ArgumentParser( description='Target Platoon')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=1883,
        type=int,
        help='TCP port to listen to (default: 1883)')
    argparser.add_argument(
        '--id',
        default=1,
        type=int,
        help='Follower ID. ID: 1 means first follwer')
    argparser.add_argument(
        '-N',
        default=100,
        type= int,
        help='')
    argparser.add_argument(
        '-T',
        default=0.02,
        type=float,
        help='')

    args = argparser.parse_args()

    return args
