import { TAKEOFF, LAND, SELECT_AGENT } from './AgentTypes';

const INITIAL_STATE = {
    agentCount: 0,
    
    selectedAgentName: "",
    selectedAgentAddress: "",
    selectedAgentStatus: "",
    selectedAgentState: "",

    lastCommand: "",
};

export const AgentReducer = (state = INITIAL_STATE, action) => {

    var newState = Object.assign({}, state);

    switch (action.type) {

        case TAKEOFF:
            newState.lastCommand = "TAKE OFF!"
            break;
        case LAND:
            newState.lastCommand = "LAND!"
            break;
        case SELECT_AGENT:
            newState.selectedAgentName = action.payload.name;
            newState.selectedAgentAddress = action.payload.address;
            newState.selectedAgentStatus = action.payload.status;
            newState.selectedAgentState = action.payload.state;
            break;
         default:
            break;

    }

    return newState;

};