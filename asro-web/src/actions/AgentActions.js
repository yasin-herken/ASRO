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

import store from '../store';
import axios from 'axios';

const hostAddress = "http://localhost:5000";
//const hostAddress = "http://10.211.55.12:5000";

export const takeOff = () => async (dispatch) => {
    // Get the selectedAgent
    var target = store.getState().asro.selectedAgent.name;
    if (target === "") {
        return;
    }

    // Send a request to backend
    axios.get(hostAddress + '/mission_takeoff?target='+target).then(response => {
        dispatch(
            {
                type: TAKE_OFF,
                target: target,
            }
        )
    });
};

export const takeOffAll = () => async (dispatch) => {
    // Send a request to backend
    axios.get(hostAddress + '/mission_takeoff_all').then(response => {
        dispatch(
            {
                type: TAKE_OFF_ALL,
            }
        )
    });
};

export const land = () => async (dispatch) => {
    // Get the selectedAgent
    var target = store.getState().asro.selectedAgent.name;

    if (target === "") {
        return;
    }

    // Send a request to backend
    axios.get(hostAddress + '/mission_land?target='+target).then(response => {
        dispatch(
            {
                type: LAND,
                target: target,
            }
        )
    });
};

export const landAll = () => async (dispatch) => {
    // Send a request to backend
    axios.get(hostAddress + '/mission_land_all').then(response => {
        dispatch(
            {
                type: LAND_ALL,
            }
        )
    });
};

export const selectAgent = (agent) => async (dispatch) => {
    dispatch(
        {
            type: SELECT_AGENT,
            agent: agent
        }
    )
};

export const getAgents = () => async (dispatch) => {
    // Send a request to backend
    axios.get(hostAddress + '/get_agents').then(response => {
        dispatch(
            {
                type: GET_AGENTS,
                agents: response.data
            }
        )
    });
};

export const getAgentPose = () => async (dispatch) => {
    // Get the selectedAgent
    var target = store.getState().asro.selectedAgent.name;
    if (target === "") {
        return;
    }


    // Send a request to backend
    axios.get(hostAddress + '/get_pose?target='+target).then(response => {
        dispatch(
            {
                type: GET_AGENT_POSE,
                pose: {
                    "coords": {
                        "x": response.data.posX,
                        "y": response.data.posY,
                        "z": response.data.posZ
                    },                  
                    "state": response.data.state
                }
            }
        )
    });
};