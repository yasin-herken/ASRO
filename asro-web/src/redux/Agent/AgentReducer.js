import axios from 'axios';

import { TAKEOFF, LAND,LAND_ALL, SELECT_AGENT, GET_AGENT_POSE, GET_AGENTS, TAKE_OFF_ALL } from './AgentTypes';
import { SCENARIO_1,SCENARIO_2,SCENARIO_3,SCENARIO_4,SCENARIO_5 } from './AgentTypes';

const INITIAL_STATE = {
    agentCount: 0,
    agents: {},

    selectedAgentName: "",
    selectedAgentAddress: "",
    selectedAgentStatus: "",
    selectedAgentState: "",

    selectedAgentX: 0.0,
    selectedAgentY: 0.0,
    selectedAgentZ: 0.0,

    lastCommand: "",
};

export const AgentReducer = (state = INITIAL_STATE, action) => {

    var newState = Object.assign({}, state);
    var target = newState.selectedAgentName;
    switch (action.type) {
        case TAKEOFF:
            newState.lastCommand = "Taking off " + target;
            axios.get('http://localhost:5000/mission_takeoff?target='+target)
                .then(res => {
                    // console.log(res);
                    // console.log(res.data);
            });
            break;
        case LAND:
            target = newState.selectedAgentName
            newState.lastCommand = "Landing " + target;
            axios.get('http://localhost:5000/mission_land?target='+target)
                .then(res => {
                    // console.log(res);
                    // console.log(res.data);
            });
            break;
        case TAKE_OFF_ALL:
            newState.lastCommand = "Taking off all";
            axios.get('http://localhost:5000/mission_takeoff_all')
                .then(res => {
                    // console.log(res);
                    // console.log(res.data);
            });
            break;
        case LAND_ALL:
            newState.lastCommand = "Landing all" ;
            axios.get('http://localhost:5000/mission_land_all')
                .then(res => {
                    // console.log(res);
                    // console.log(res.data);
            });
            break;
        case SELECT_AGENT:
            newState.selectedAgentName = action.payload.name;
            newState.selectedAgentAddress = action.payload.address;
            newState.selectedAgentStatus = action.payload.status;
            newState.selectedAgentState = action.payload.state;
            break;

        case GET_AGENT_POSE:
            target = newState.selectedAgentName;
            axios.get('http://localhost:5000/get_pose?target='+target).then(resp => {
                newState.selectedAgentX = resp.data.posX;
                newState.selectedAgentY = resp.data.posY;
                newState.selectedAgentZ = resp.data.posZ;
                newState.selectedAgentState = resp.data.state;
            });
            break;

        case GET_AGENTS:
            axios.get('http://localhost:5000/get_agents').then(resp => {
                newState.agents = resp.data;
            });
            break;
        case SCENARIO_1:
            newState.lastCommand="Scenario 1";
            axios.get('http://localhost:5000/mission_one')
                .then(res => {
                    // console.log(res);
                    // console.log(res.data);
            });
            break;
        case SCENARIO_2:
            newState.lastCommand="Scenario 2";
            break;
        case SCENARIO_3:
            newState.lastCommand="Scenario 3";
            break;
        case SCENARIO_4:
            newState.lastCommand="Scenario 4";
            break;
        case SCENARIO_5:
            newState.lastCommand="Scenario 5";
            break;
        default:
            break;

    }

    return newState;

};