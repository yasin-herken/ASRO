import React  from 'react';

import "./Agents.css"

import Agent from "./Agent/Agent";

function Agents() {
    return (
        <div className='Agents'>
            <h2 className='Agents-Title'>
                Agents
            </h2>
            <div className='Agents-Container'>
                <div className='Agents-Scroll-List'>
                    <Agent name="Agent 1" address="radio:/0/100/2M" status="online" state="TAKING_OFF"/>
                    <Agent name="Agent 2" address="radio:/0/110/2M" status="online" state="TAKING_OFF"/>
                    <Agent name="Agent 3" address="radio:/0/120/2M" status="online" state="HOVERING"/>
                    <Agent name="Agent 4" address="radio:/0/130/2M" status="offline" state="N/A"/>
                </div>
            </div>
        </div>
    );
}

export default Agents;