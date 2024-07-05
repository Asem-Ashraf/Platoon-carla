import Communication.ourmqtt as V2V


def getMyCurrentStates():
    while True:
        if V2V.ourclient.trgt_flag:
            V2V.ourclient.trgt_flag = False
            print(V2V.ourclient.trgt_state)
            return V2V.ourclient.trgt_state


def getLeaderStates():
    while True:
        if V2V.ourclient.leader_flag:
            V2V.ourclient.leader_flag = False
            print(V2V.ourclient.leader_state)
            return V2V.ourclient.leader_state


def getFrontVehicleStates(id):
    while True:
        if V2V.ourclient.front_flag:
            V2V.ourclient.front_flag = False
            print(V2V.ourclient.front_state)
            return V2V.ourclient.front_state
