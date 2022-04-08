import time

from Agent import Agent

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

    try:
        while RUNNING:
            currTime = time.perf_counter()
            deltaTime = currTime - lastTime
            if deltaTime < 1.0 / TICK_RATE:
                continue

            for agent in agents:
                agent.tick(deltaTime)

            lastTime = time.perf_counter()
    except KeyboardInterrupt:
        print("Good bye!")
    except Exception as e:
        print(e)
        exit(-1)
