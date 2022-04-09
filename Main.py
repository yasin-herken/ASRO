import time
import numpy

from Agent import Agent

from Constants import Mission
from Constants import State
from Constants import MissionInfo
from Constants import FormationShape

from Constants import FORMATION_TRIANGLE

AGENT_COUNT = 3
RUNNING = True
TICK_RATE = 60 # Hz

if __name__ == "__main__":
    agents = []
    currTime = time.perf_counter()
    lastTime = time.perf_counter()
    deltaTime = 0.0

    # Create new agents
    for i in range(AGENT_COUNT):
        agents.append(Agent(i))

    time.sleep(1)

    try:
        while RUNNING:
            currTime = time.perf_counter()
            deltaTime = currTime - lastTime
            if deltaTime < 1.0 / TICK_RATE:
                continue

            # Update virtual agents
            for agent in agents:
                agent.tick(deltaTime)

            # Check their states and if they are all the same
            states = []
            for agent in agents:
                states.append(agent.getState())

            swarmState = states[0]
            isSwarmStateValid = True

            for i in states:
                if i != swarmState:
                    isSwarmStateValid = False
                    break
            
            # Check their missions and if they are all the same
            missions = []
            for agent in agents:
                missions.append(agent.getMission())

            swarmMission = missions[0]
            isSwarmMissionValid = True

            for i in missions:
                if i != swarmMission:
                    isSwarmMissionValid = False
                    break

            # TODO: This is not a good method. Please create a better swarming and mission control!!!
            # Mission Control
            # print(f"swarmMission: {swarmMission}, swarmState: {swarmState}")
            if isSwarmStateValid and isSwarmMissionValid:
                # If they are just started then formation should start
                if swarmMission == Mission.NONE and swarmState == State.STATIONARY:
                    missionInfo = MissionInfo(
                        angularVelocity = 0.0,
                        formationShape = FormationShape.TRIANGLE,
                        formationMatrix = FORMATION_TRIANGLE,
                        maxVelocity = 1.0,
                        minimumSafeDistance = 2.5,
                        otherAgents = agents,
                        rotateAngle = 0.0,
                        targetPoint = numpy.array([0.0, 0.0, 0.0]),
                    )
                    for agent in agents:
                        agent.updateMission(Mission.FORMING, missionInfo)
                    print("Mission formation has been started.")
                
                # If they are done forming and ready to be on mission then start the scenario
                elif swarmMission == Mission.FORMING and swarmState == State.ON_MISSION:
                    missionInfo = MissionInfo(
                        angularVelocity = 0.0,
                        formationShape = FormationShape.TRIANGLE,
                        formationMatrix = FORMATION_TRIANGLE,
                        maxVelocity = 3.0,
                        minimumSafeDistance = 2.5,
                        otherAgents = agents,
                        rotateAngle = 0.0,
                        targetPoint = numpy.array([30.0, 30.0, -3.0]),
                    )
                    for agent in agents:
                        agent.updateMission(Mission.MOVING, missionInfo)
                    print("Mission moving has been started.")

            else:
                print("Not all agents are in the same state!")

            lastTime = time.perf_counter()
            
    except KeyboardInterrupt:
        print("Good bye!")
        for agent in agents:
            agent.kill()

    except Exception as e:
        print(e)
        exit(-1)
