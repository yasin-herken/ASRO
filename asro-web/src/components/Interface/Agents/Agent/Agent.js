import {
    selectAgent
} from "../../../../actions/AgentActions"

import "./Agent.css";

import React  from 'react';
import {useDispatch} from "react-redux";

function Agent({name, address, status, state}) {

    const dispatch = useDispatch();

    function OnClick(event) {
        dispatch(selectAgent({
            "name": name,
            "address": address,
            "status": status,
            "state": state
        }));
    }
    
    return (
        <div className='Agent' onClick={OnClick}>
            <div className='Agent-Info-One'>
                <h2 className='Agent-Info-Name'>
                    {name}
                </h2>
                <h5 className='Agent-Info-Address'>
                    {address}
                </h5>
            </div>
            <div className='Agent-Info-Two'>
                <h4 className={'Agent-Info-Status ' + ((status === "online") ? 'Green-Text' : 'Red-Text')}>
                    {status}
                </h4>
                <h4 className='Agent-Info-State'>
                    {state}
                </h4>
            </div>
        </div>
    );
}

export default Agent;