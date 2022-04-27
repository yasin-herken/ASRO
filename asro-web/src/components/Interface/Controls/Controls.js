import React  from 'react';

import "./Controls.css";

import Scenario from './Scenario/Scenario';

function Controls() {
    return (
        <div className='Controls'>
            <h2 className='Controls-Title'>
                Controls
            </h2>
            <div className='Controls-Container'>
                <div className='Controls-Scenarios'>
                    <Scenario name="Scenario 1"/>
                    <Scenario name="Scenario 2"/>
                    <Scenario name="Scenario 3"/>
                    <Scenario name="Scenario 4"/>
                    <Scenario name="Scenario 5"/>
                </div>
                <div className='Controls-Formation'>
                    <h2 className='Controls-Formation-Title'>
                        Formation
                    </h2>
                    <div className='Controls-Formation-Container'>

                    </div>
                </div>
                <div className='Controls-Commands'>
                    <h2 className='Controls-Commands-Title'>
                        Commands
                    </h2>
                    <div className='Controls-Commands-Container'>

                    </div>
                </div>
            </div>
        </div>
    );
}

export default Controls;