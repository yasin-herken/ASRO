import yaml
import pathlib
import os

def agent_yaml(address = "0xE7E7E7E7E7",channel = 80,initialPosition = [0.0,0.0,0.0]):
    address = int(address[-2:],16)
    print(address)
    agent ={
        'address':address,
        'channel':channel,
        'initialPosition': initialPosition
    }
    return agent

class yaml_file:
    def __init__(self,path='./crazyflies.yaml',agents_yaml=[{'address':1,'channel':80,'initialPosition':[0.0,0.0,0.0]}]):
        self.path = pathlib.Path(path)   
        self.agents_yaml = agents_yaml
        self.yaml_content={
            'crazyflies': 
            [
                {
                'id': agents_yaml[0]['address'], 
                'channel': agents_yaml[0]['channel'],
                'type': 'default',
                'initialPosition': agents_yaml[0]['initialPosition']
                }
            ]
        }

    def yaml_delete_file(self):
        os.remove(self.path)

    def yaml_create_file(self):
        with open(self.path,"w") as file:
            file.write("")

    def yaml_add_default_file(self):
        if os.stat(self.path).st_size == 0:
            with open(self.path,"w") as file:
                # Add default content to the yaml
                yaml.dump(self.yaml_content,file)

    def yaml_update_file(self):
        with open(self.path,"r") as file:
            data = yaml.safe_load(file)
            for content in data['crazyflies']:
                counter = 0
                for agent_yaml in self.agents_yaml:
                    address =agent_yaml['address']
                    channel =agent_yaml['channel']
                    if (int(content['id']) == address):
                        new_content =[{
                            'id':address,
                            'channel': channel,
                            'type': "default",
                            "initialPosition": [0.0,0.0,0.0]
                        }]
                        data['crazyflies'][counter] = new_content
                        with open(self.path,"w") as file:
                            yaml.dump(data,file)
                    counter = counter + 1

    def yaml_extend_file(self):
        counter = 0
        for agent_yaml in self.agents_yaml:
            print("here")
            address =agent_yaml['address']
            channel =agent_yaml['channel']
            extend_content=[
            {
                'id': address, 
                'channel': channel,
                'type': 'default',
                'initialPosition': [0.0, 0.0, 0.0]
            }
            ]
                
            self.yaml_content.get("crazyflies").extend(extend_content)
            
            with open(self.path,"w") as file:
                yaml.dump(self.yaml_content,file)
            counter = counter + 1

    def yaml_check_file(self):
        with open(self.path,"r") as file:
            data = yaml.safe_load(file)
            lengthData=len(data['crazyflies'])
            checklist = data['crazyflies']
            for i in range(0,lengthData-1):
                for j in range(i+1,lengthData-1):
                    if checklist[i]['id'] == checklist[j]['id']:
                        del checklist[i]
                        with open(self.path,"w") as file:
                            yaml.dump(data,file)
            

data1 = agent_yaml()
data2 = agent_yaml("0xE7E7E7E702",81,[1.0,1.0,0.0])
data = []
data.extend([data1,data2])
obj = yaml_file('./test.yaml',data)
obj.yaml_extend_file()
obj.yaml_check_file()
obj.yaml_add_default_file()
obj.yaml_create_file()
obj.yaml_delete_file()
obj.yaml_update_file()

'''def scanRadio(addresses):
    channels = [80,81]
        for address in addresses:
        output = subprocess.check_output(["rosrun","crazyflie_tools","scan","--address",address])
        output = output.decode('utf-8')
        if output =="":
            print(f"Not found: {address}")
        else:
            A = output.split('/')
            channel.append(A[3])
            print(A[3])
    yaml_adding_agents(addresses,channels)
    
    return'''
