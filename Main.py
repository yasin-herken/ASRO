import time
from cv2 import rotate
import numpy

from datetime import datetime

from Agent import Agent

from Constants import State
from Constants import Mission
from Constants import MissionInfo
from Constants import FormationShape

from Constants import FORMATION_TRIANGLE

AGENT_COUNT = 3

SWARM_POSITION = numpy.array([0.0, 0.0, 0.0])
SWARM_HEADING = numpy.array([0.0, 0.0, 0.0])

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

            # Calculate swarm position (mid point)
            for agent in agents:
                SWARM_POSITION += agent.getPosition()
            SWARM_POSITION /= len(agents)

            # Calculate swarm heading (a unit vector showing the front of the swarm)
            frontAgent = agents[0]
            distanceToFrontAgentFromSwarmPos = frontAgent.getPosition() - SWARM_POSITION
            SWARM_HEADING = distanceToFrontAgentFromSwarmPos / numpy.linalg.norm(distanceToFrontAgentFromSwarmPos)

            # TODO: This is not a good method. Please create a better swarming and mission control!!!
            # Mission Control
            # print(f"swarmMission: {swarmMission}, swarmState: {swarmState}")
            if isSwarmStateValid and isSwarmMissionValid:
                # If they are just started then start taking them off the ground
                if swarmMission == Mission.NONE and swarmState == State.STATIONARY:
                    missionInfo = MissionInfo(
                        formationShape = None,
                        formationMatrix = None,
                        maxVelocity = 1.0,
                        minimumSafeDistance = 0.0,
                        otherAgents = None,
                        rotateAngle = 0.0,
                        angularVelocity = 0.0,
                        oneRotateStepAngle = 0.0,
                        currentRotateStep = 0,
                        targetRotateStep = 0,
                        targetPoint = numpy.array([0.0, 0.0, 0.0]),
                    )
                    for agent in agents:
                        agent.updateMission(Mission.TAKE_OFF, missionInfo)
                    print(f"[{datetime.now()}] [Main] Mission TAKE_OFF started")
                
                # If they are done taking off and ready to take formation then start taking formation
                elif swarmMission == Mission.TAKE_OFF and swarmState == State.HOVERING:
                    missionInfo = MissionInfo(
                        formationShape = FormationShape.TRIANGLE,
                        formationMatrix = FORMATION_TRIANGLE,
                        maxVelocity = 2.0,
                        minimumSafeDistance = 2.5,
                        otherAgents = agents,
                        rotateAngle = 0.0,
                        angularVelocity = 0.0,
                        oneRotateStepAngle = 0.0,
                        currentRotateStep = 0,
                        targetRotateStep = 0,
                        targetPoint = numpy.array([0.0, 0.0, 0.0]),
                    )
                    for agent in agents:
                        agent.updateMission(Mission.TAKE_FORMATION, missionInfo)
                    print(f"[{datetime.now()}] [Main] Mission TAKE_FORMATION started")

                # If they are hovering and ready to rotate then start rotating 
                elif swarmMission == Mission.TAKE_FORMATION and swarmState == State.HOVERING:
                    # Calculate smooth rotation parameters
                    rotateAngle = 90.0 # degrees
                    angularVelocity = 15.0 # degrees per second
                    targetRotateStep = (rotateAngle / angularVelocity) * TICK_RATE
                    oneRotateStep = rotateAngle / targetRotateStep
                    currentRotateStep = 0

                    missionInfo = MissionInfo(
                        formationShape = FormationShape.TRIANGLE,
                        formationMatrix = FORMATION_TRIANGLE,
                        maxVelocity = 0.0,
                        minimumSafeDistance = 2.5,
                        otherAgents = agents,
                        rotateAngle = rotateAngle,
                        angularVelocity = angularVelocity,
                        oneRotateStepAngle = oneRotateStep,
                        currentRotateStep = currentRotateStep,
                        targetRotateStep = targetRotateStep,
                        targetPoint = 0.0,
                    )
                    for agent in agents:
                        agent.updateMission(Mission.ROTATE, missionInfo)
                    print(f"[{datetime.now()}] [Main] Mission ROTATE started")

                # If they are hovering and ready to move then start moving to target(s)
                elif swarmMission == Mission.ROTATE and swarmState == State.HOVERING:
                    missionInfo = MissionInfo(
                        formationShape = FormationShape.TRIANGLE,
                        formationMatrix = FORMATION_TRIANGLE,
                        maxVelocity = 3.0,
                        minimumSafeDistance = 2.5,
                        otherAgents = agents,
                        rotateAngle = 0.0,
                        angularVelocity = 0.0,
                        oneRotateStepAngle = 0.0,
                        currentRotateStep = 0,
                        targetRotateStep = 0,
                        targetPoint = numpy.array([30.0, 30.0, -3.0]),
                    )
                    for agent in agents:
                        agent.updateMission(Mission.MOVE, missionInfo)
                    print(f"[{datetime.now()}] [Main] Mission MOVE started")
            else:
                pass
                # print("Not all agents are in the same state!")

            lastTime = time.perf_counter()
            
    except KeyboardInterrupt:
        print("Good bye!")
        for agent in agents:
            agent.kill()

    except Exception as e:
        print(e)
        exit(-1)
