import yaml

def getInitialPosition(id):
    with open("./Deprecated/PyCrazySwarm/crazyflies.yaml","r") as file:
        data =yaml.load(file,Loader=yaml.FullLoader)
        for agent in data["crazyflies"]:
            if(id == agent["id"] or str(id) == str(agent["id"])):
                print(agent)

getInitialPosition(226)