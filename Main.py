import time
import numpy
import csv

from datetime import datetime

from Agent import Agent

from Constants import State
from Constants import Mission
from Constants import MissionInfo
from Constants import FormationShape
from Constants import angleBetween
from Constants import getMagnitude
from Constants import FORMATION_TRIANGLE

AGENT_COUNT = 3

SWARM_POSITION = numpy.array([0.0, 0.0, 0.0])
SWARM_HEADING = numpy.array([0.0, 0.0, 0.0])

TARGET_POS_COUNT = 0

RUNNING = True
TICK_RATE = 100 # Hz

if __name__ == "__main__":
    agents = []
    currTime = time.perf_counter()
    lastTime = time.perf_counter()
    deltaTime = 0.0

    # Record agent info
    csvFile = open("data.csv", "w", newline='', encoding='utf-8')
    csvWriter = csv.writer(csvFile)

    csvHeader = [
        "Time",

        "agent_0_pos_x", "agent_0_pos_y", "agent_0_pos_z",
        "agent_0_vel_x", "agent_0_vel_y", "agent_0_vel_z",
        "agent_0_formation_control_x", "agent_0_formation_control_y", "agent_0_formation_control_z",
        "agent_0_avoidance_control_x", "agent_0_avoidance_control_y", "agent_0_avoidance_control_z",
        "agent_0_trajectory_control_x", "agent_0_trajectory_control_y", "agent_0_trajectory_control_z",
        "agent_0_formation_control_force",
        "agent_0_avoidance_control_force", 
        "agent_0_trajectory_control_force",

        "agent_1_pos_x", "agent_1_pos_y", "agent_1_pos_z",
        "agent_1_vel_x", "agent_1_vel_y", "agent_1_vel_z",
        "agent_1_formation_control_x", "agent_1_formation_control_y", "agent_1_formation_control_z",
        "agent_1_avoidance_control_x", "agent_1_avoidance_control_y", "agent_1_avoidance_control_z",
        "agent_1_trajectory_control_x", "agent_1_trajectory_control_y", "agent_1_trajectory_control_z",
        "agent_1_formation_control_force",
        "agent_1_avoidance_control_force", 
        "agent_1_trajectory_control_force",

        "agent_2_pos_x", "agent_2_pos_y", "agent_2_pos_z",
        "agent_2_vel_x", "agent_2_vel_y", "agent_2_vel_z",
        "agent_2_formation_control_x", "agent_2_formation_control_y", "agent_2_formation_control_z",
        "agent_2_avoidance_control_x", "agent_2_avoidance_control_y", "agent_2_avoidance_control_z",
        "agent_2_trajectory_control_x", "agent_2_trajectory_control_y", "agent_2_trajectory_control_z",
        "agent_2_formation_control_force",
        "agent_2_avoidance_control_force", 
        "agent_2_trajectory_control_force",
    ]

    csvWriter.writerow(csvHeader)

    # Create new agents
    for i in range(AGENT_COUNT):
        agents.append(Agent(i))

    time.sleep(1)

    startTime = time.perf_counter()
    endTime = time.perf_counter()

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
            SWARM_POSITION = numpy.array([0.0, 0.0, 0.0])
            for agent in agents:
                SWARM_POSITION += agent.getPosition()
            SWARM_POSITION /= len(agents)

            # Calculate swarm heading (a unit vector showing the front of the swarm)
            frontAgent = agents[0]
            distanceToFrontAgentFromSwarmPos = frontAgent.getPosition() - SWARM_POSITION
            SWARM_HEADING = distanceToFrontAgentFromSwarmPos / numpy.linalg.norm(distanceToFrontAgentFromSwarmPos)

            x = round(SWARM_POSITION[0], 2)
            y = round(SWARM_POSITION[1], 2)
            z = round(SWARM_POSITION[2], 2)

            # print(f"Swarm Position: {SWARM_POSITION}, Swarm Heading: {SWARM_HEADING}")

            # print(SWARM_HEADING)

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
                        targetPoint = numpy.array([0.0, 0.0, 0.0])
                    )
                    for agent in agents:
                        agent.updateMission(Mission.TAKE_OFF, missionInfo)
                    msg = f"[{datetime.now()}] [Main] Mission TAKE_OFF started"
                    print(msg)
                    agents[0].logInfo(msg)
                
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
                        targetPoint = numpy.array([0.0, 0.0, 0.0])
                    )
                    for agent in agents:
                        agent.updateMission(Mission.TAKE_FORMATION, missionInfo)
                    msg = f"[{datetime.now()}] [Main] Mission TAKE_FORMATION started"
                    print(msg)
                    agents[0].logInfo(msg)

                # If they are hovering and ready to rotate then start rotating_0
                elif swarmMission == Mission.TAKE_FORMATION and swarmState == State.DONE_FORMING:
                    # Head north
                    targetHeading = numpy.array([0.0, 1.0, 0.0])

                    oneWay = angleBetween(targetHeading, SWARM_HEADING)
                    otherWay = angleBetween(SWARM_HEADING, targetHeading)

                    rotateAngle = min(oneWay, otherWay)

                    missionInfo = MissionInfo(
                        formationShape = FormationShape.TRIANGLE,
                        formationMatrix = FORMATION_TRIANGLE,
                        maxVelocity = 0.0,
                        minimumSafeDistance = 2.5,
                        otherAgents = agents,
                        rotateAngle = rotateAngle,
                        angularVelocity = 30.0,
                        targetPoint = numpy.array([0.0, 0.0, 0.0])
                    )
                    for agent in agents:
                        agent.updateMission(Mission.ROTATE, missionInfo)
                    msg = f"[{datetime.now()}] [Main] Mission ROTATE started 0"
                    print(msg)
                    agents[0].logInfo(msg)

                # If they are hovering and ready to move then start moving to target_0
                elif swarmMission == Mission.ROTATE and swarmState == State.DONE_ROTATING and TARGET_POS_COUNT == 0:
                    missionInfo = MissionInfo(
                        formationShape = FormationShape.TRIANGLE,
                        formationMatrix = FORMATION_TRIANGLE,
                        maxVelocity = 3.0,
                        minimumSafeDistance = 2.5,
                        otherAgents = agents,
                        rotateAngle = 0.0,
                        angularVelocity = 0.0,
                        targetPoint = numpy.array(
                            [SWARM_POSITION[0] + 20.0, SWARM_POSITION[1], 0.25]
                        )
                    )
                    for agent in agents:
                        agent.updateMission(Mission.MOVE, missionInfo)
                    msg = f"[{datetime.now()}] [Main] Mission MOVE started 0"
                    print(msg)
                    agents[0].logInfo(msg)

                    TARGET_POS_COUNT += 1

                # If they are hovering and ready to rotate then start rotating_1
                elif swarmMission == Mission.MOVE and swarmState == State.DONE_MOVING and TARGET_POS_COUNT == 1:
                    # Head south
                    targetHeading = numpy.array([1.0, 0.0, 0.0])

                    oneWay = angleBetween(targetHeading, SWARM_HEADING)
                    otherWay = angleBetween(SWARM_HEADING, targetHeading)

                    rotateAngle = min(oneWay, otherWay)

                    missionInfo = MissionInfo(
                        formationShape = FormationShape.TRIANGLE,
                        formationMatrix = FORMATION_TRIANGLE,
                        maxVelocity = 0.0,
                        minimumSafeDistance = 2.5,
                        otherAgents = agents,
                        rotateAngle = rotateAngle,
                        angularVelocity = 15.0,
                        targetPoint = numpy.array([0.0, 0.0, 0.0])
                    )
                    for agent in agents:
                        agent.updateMission(Mission.ROTATE, missionInfo)
                    msg = f"[{datetime.now()}] [Main] Mission ROTATE started 1"
                    print(msg)
                    agents[0].logInfo(msg)
                
                # If they are hovering and ready to move then start moving to target_1
                elif swarmMission == Mission.ROTATE and swarmState == State.DONE_ROTATING and TARGET_POS_COUNT == 1:
                    missionInfo = MissionInfo(
                        formationShape = FormationShape.TRIANGLE,
                        formationMatrix = FORMATION_TRIANGLE,
                        maxVelocity = 3.0,
                        minimumSafeDistance = 2.5,
                        otherAgents = agents,
                        rotateAngle = 0.0,
                        angularVelocity = 0.0,
                        targetPoint = numpy.array(
                            [SWARM_POSITION[0], SWARM_POSITION[1] + 20.0, 0.25]
                        )
                    )
                    for agent in agents:
                        agent.updateMission(Mission.MOVE, missionInfo)
                    msg = f"[{datetime.now()}] [Main] Mission MOVE started 1"
                    print(msg)
                    agents[0].logInfo(msg)

                    TARGET_POS_COUNT += 1
                
                # If they are hovering and ready to rotate then start rotating_2
                elif swarmMission == Mission.MOVE and swarmState == State.DONE_MOVING and TARGET_POS_COUNT == 2:
                    # Head south
                    targetHeading = numpy.array([0.0, -1.0, 0.0])

                    oneWay = angleBetween(targetHeading, SWARM_HEADING)
                    otherWay = angleBetween(SWARM_HEADING, targetHeading)

                    rotateAngle = min(oneWay, otherWay)

                    missionInfo = MissionInfo(
                        formationShape = FormationShape.TRIANGLE,
                        formationMatrix = FORMATION_TRIANGLE,
                        maxVelocity = 0.0,
                        minimumSafeDistance = 2.5,
                        otherAgents = agents,
                        rotateAngle = rotateAngle,
                        angularVelocity = 30.0,
                        targetPoint = numpy.array([0.0, 0.0, 0.0])
                    )
                    for agent in agents:
                        agent.updateMission(Mission.ROTATE, missionInfo)
                    msg = f"[{datetime.now()}] [Main] Mission ROTATE started 2"
                    print(msg)
                    agents[0].logInfo(msg)
                
                # If they are hovering and ready to move then start moving to target_2
                elif swarmMission == Mission.ROTATE and swarmState == State.DONE_ROTATING and TARGET_POS_COUNT == 2:
                    missionInfo = MissionInfo(
                        formationShape = FormationShape.TRIANGLE,
                        formationMatrix = FORMATION_TRIANGLE,
                        maxVelocity = 3.0,
                        minimumSafeDistance = 2.5,
                        otherAgents = agents,
                        rotateAngle = 0.0,
                        angularVelocity = 0.0,
                        targetPoint = numpy.array(
                            [SWARM_POSITION[0] - 20.0, SWARM_POSITION[1], 0.25]
                        )
                    )
                    for agent in agents:
                        agent.updateMission(Mission.MOVE, missionInfo)
                    msg = f"[{datetime.now()}] [Main] Mission MOVE started 2"
                    print(msg)
                    agents[0].logInfo(msg)

                    TARGET_POS_COUNT += 1
                
                # If they are hovering and ready to rotate then land
                elif swarmMission == Mission.MOVE and swarmState == State.DONE_MOVING and TARGET_POS_COUNT == 3:
                    missionInfo = MissionInfo(
                        formationShape = None,
                        formationMatrix = None,
                        maxVelocity = 0.0,
                        minimumSafeDistance = 0.0,
                        otherAgents = MissionInfo,
                        rotateAngle = 0.0,
                        angularVelocity = 0.0,
                        targetPoint = numpy.array([0.0, 0.0, 0.0])
                    )
                    for agent in agents:
                        agent.updateMission(Mission.LAND, missionInfo)
                    msg = f"[{datetime.now()}] [Main] Mission ended. Now landing"
                    print(msg)
                    agents[0].logInfo(msg)
                elif swarmMission == Mission.LAND and swarmState == State.DONE_LANDING:
                    msg = f"[{datetime.now()}] [Main] Simulation complete!"
                    print(msg)
                    agents[0].logInfo(msg)
                    break
            else:
                pass
            
            # update endTime
            endTime = time.perf_counter()

            # write to csv file
            row = []
            row.append(round((endTime - startTime), 4))

            for agent in agents:
                pos = agent.getPosition()
                vel = agent.getVelocity()

                formationControl = agent.getFormationControl()
                avoidanceControl = agent.getAvoidanceControl()
                trajectoryControl = agent.getTrajectoryControl()

                row.append(pos[0])
                row.append(pos[1])
                row.append(pos[2])

                row.append(vel[0])
                row.append(vel[1])
                row.append(vel[2])

                row.append(formationControl[0])
                row.append(formationControl[1])
                row.append(formationControl[2])

                row.append(avoidanceControl[0])
                row.append(avoidanceControl[1])
                row.append(avoidanceControl[2])

                row.append(trajectoryControl[0])
                row.append(trajectoryControl[1])
                row.append(trajectoryControl[2])

                row.append(round(getMagnitude(formationControl), 4))
                row.append(round(getMagnitude(avoidanceControl), 4))
                row.append(round(getMagnitude(trajectoryControl), 4))
            
            csvWriter.writerow(row)

            lastTime = time.perf_counter()

        print(f"Simulation took: {round((endTime - startTime), 4)} seconds")

        csvFile.close()


    except KeyboardInterrupt:
        print("Good bye!")
        for agent in agents:
            agent.kill()

    except Exception as e:
        print(e.format_exc())
        exit(-1)
