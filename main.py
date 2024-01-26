from smlib import loader

from chargebuffer import chargebuffer_fsm
# from chargeslider import chargeslider_fsm
# from topactuator import topactuator_fsm
# from central import central_fsm
# from bottomactuator import bottomactuator_fsm
# from dischargeslider import dischargeslider_fsm
# from dischargebuffer import dischargebuffer_fsm


def main():

    # load the fsms
    l = loader()

    l.load(chargebuffer_fsm,"chargebuffer_fsm")
    # l.load(chargeslider_fsm,"chargeslider_fsm")
    # l.load(topactuator_fsm,"topactuator_fsm")
    # l.load(central_fsm,"central_fsm")
    # l.load(bottomactuator_fsm,"bottomactuator_fsm")
    # l.load(dischargeslider_fsm,"dischargeslider_fsm")
    # l.load(dischargebuffer_fsm,"dischargebuffer_fsm")

    # start execution
    l.start()



if __name__ == '__main__':
    main()
