import {
    TAKE_OFF,
    TAKE_OFF_ALL,
    LAND,
    LAND_ALL,
    SELECT_AGENT,
    GET_AGENTS,
    GET_AGENT_POSE,
    SCENARIO_1,
    SCENARIO_2,
    SCENARIO_3,
    SCENARIO_4,
    SCENARIO_5
} from "../constants/AgentConstants";

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

    switch (action.type) {
        case TAKE_OFF:
            break;
        case LAND:
            break;
        case TAKE_OFF_ALL:
            break;
        case LAND_ALL:
            break;
        case SELECT_AGENT:
            break;
        case GET_AGENT_POSE:
            break;
        case GET_AGENTS:
            break;
        case SCENARIO_1:
            break;
        case SCENARIO_2:
            break;
        case SCENARIO_3:
            break;
        case SCENARIO_4:
            break;
        case SCENARIO_5:
            break;
        default:
            break;
    }

    return newState;

};