import React  from 'react';

import "./Scenario.css"

import Start from "../../../../static/Start.svg";

function Scenario({name}) {
    return (
        <div className='Scenario'>
            <h3 className='Scenario-Name'>
                {name}
            </h3>
            <button type='button' className='Scenario-Button'>
                <img src={Start} width="20px" className='Scenario-Img'/>
            </button>
        </div>
    );
}

export default Scenario;