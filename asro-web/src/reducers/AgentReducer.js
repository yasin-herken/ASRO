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

    selectedAgent: {
        "name": "",
        "address": "",
        "status": "",
        "state": "",
        "coords": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        }
    },

    lastCommand: "",
};

export const AgentReducer = (state = INITIAL_STATE, action) => {

    var newState = Object.assign({}, state);

    switch (action.type) {
        case TAKE_OFF:
            newState.lastCommand = "Requesting take off for " + action.target + ".";
            break;
        case LAND:
            newState.lastCommand = "Requesting landing for " + action.target + ".";
            break;
        case TAKE_OFF_ALL:
            newState.lastCommand = "Requesting take off for all.";
            break;
        case LAND_ALL:
            newState.lastCommand = "Requesting landing for all.";
            break;
        case SELECT_AGENT:
            newState.selectedAgent = action.agent;
            newState.selectedAgent.coords = {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            };
            newState.lastCommand = "Selected " + action.agent.name;
            break;
        case GET_AGENT_POSE:
            newState.selectedAgent.state = action.pose.state;
            newState.selectedAgent.coords = action.pose.coords;
            break;
        case GET_AGENTS:
            newState.lastCommand = "Loading agents " + action.target;
            newState.agents = action.agents;
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