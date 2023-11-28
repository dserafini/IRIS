from smlib import loader

#from "NOME_FILE" import "NOME_CLASSE"
from Modifiche_IRIS_NOVA2 import IRIS_FSM
#from  Recovery_Iris import RECOVERY_IRIS_FSM


def main():

    # load the fsms
    l = loader()

    #l.load("NOME_CLASSE", "Nome_casuale")
    l.load(IRIS_FSM,"IRIS_FSM")
    #l.load(RECOVERY_IRIS_FSM,"RECOVERY_IRIS_FSM")


    # start execution
    l.start()



if __name__ == '__main__':
    main()
