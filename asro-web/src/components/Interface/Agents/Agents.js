import React  from 'react';
import {useSelector} from "react-redux";

import "./Agents.css"

import Agent from "./Agent/Agent";

function Agents() {
    const agents = useSelector((state) => state.asro.agents);

    return (
        <div className='Agents'>
            <h2 className='Agents-Title'>
                Agents
            </h2>
            <div className='Agents-Container'>
                <div className='Agents-Scroll-List'>
                    {ListAgents()}
                </div>
            </div>
        </div>
    );

    function ListAgents() {
        
        var retValue = []

        if (Object.keys(agents).length !== 0 ) {
            var names = agents.names;
            var addresses = agents.addresses;

            for (var i = 0; i < names.length; i++) {
                retValue.push(
                    <Agent key={names[i]} name={names[i]} address={addresses[i]} status="online" state="STATIONARY"/>
                )
            }
        } else {
            retValue.push(
                <h3 key={"no_agents_to_show"}>
                    No agents to show
                </h3>
            );
        }
        

        return retValue;
    }
}

export default Agents;