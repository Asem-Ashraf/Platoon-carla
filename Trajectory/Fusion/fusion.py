import paho.mqtt as V2VCommunication
import Communication.ourmqtt as ourmqtt


def getVehicleID(): # CAN BE HARDCODED
    return int(ourmqtt.trgt_id)

def getN(): # CAN BE HARDCODED
    # uses mqtt
    # called once
    pass

def getTs(): # CAN BE HARDCODED
    # uses mqtt
    # called once
    pass


def getCurrentStates():
    if ourmqtt.trgt_flag:
        ourmqtt.trgt_flag = False
        return ourmqtt.trgt_state


def getLeaderStates():
    if ourmqtt.leader_flag:
        ourmqtt.leader_flag = False
        return ourmqtt.leader_state


def getFrontVehicleStates(id):
    if ourmqtt.front_flag:
        ourmqtt.front_flag = False
        return ourmqtt.front_state
