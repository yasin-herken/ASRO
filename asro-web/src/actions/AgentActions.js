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
import {useDispatch} from "react-redux";
import axios from 'axios';

export const takeOff = (target) => async (dispatch) => {
    // Send a request to backend
    axios.get('http://localhost:5000/mission_takeoff?target='+target).then(response => {
        console.log(response);
    });
};

export const takeOffAll = () => async (dispatch) => {
    // Send a request to backend
    axios.get('http://localhost:5000/mission_takeoff_all').then(response => {
        console.log(response);
    });
};

export const land = (target) => async (dispatch) => {
    // Send a request to backend
    axios.get('http://localhost:5000/mission_land?target='+target).then(response => {
        console.log(response);
    });
};

export const landAll = () => async (dispatch) => {
    // Send a request to backend
    axios.get('http://localhost:5000/mission_land_all').then(response => {
        console.log(response);
    });
};

export const selectAgent = () => async (dispatch) => {
    
};

export const getAgents = () => async (dispatch) => {
    // Send a request to backend
    axios.get('http://localhost:5000/get_agents').then(response => {
        console.log(response);
    });
};

export const getAgentPose = (target) => async (dispatch) => {
    // Send a request to backend
    axios.get('http://localhost:5000/get_pose?target='+target).then(response => {
        console.log(response);
    });
};