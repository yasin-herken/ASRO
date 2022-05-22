import "./Controls.css";
import Scenario from './Scenario/Scenario';
import CustomButton from './CustomButton/CustomButton';

import React  from 'react';
import {useSelector} from "react-redux";

function Controls() {
    const agent = useSelector((state) => state.agent);

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
                        <CustomButton name="TRIANGLE"/>
                        <CustomButton name="ARROWHEAD"/>
                        <CustomButton name="STAR"/>
                        <CustomButton name="SQUARE"/>
                        <CustomButton name="PENTAGON"/>
                    </div>
                </div>
                <div className='Controls-Commands'>
                    <h2 className='Controls-Commands-Title'>
                        Commands
                    </h2>
                    <div className='Controls-Commands-Container'>
                        <CustomButton name="TAKE OFF"/>
                        <CustomButton name="LAND"/>
                        <CustomButton name="TAKE_OFF_ALL"/>
                        <CustomButton name="LAND_ALL"/>

                    </div>
                </div>
                <div className='Controls-Commands-LastAction'>
                    {(agent.lastCommand === "" ? "last command will be displayed here" : agent.lastCommand)}
                </div>
            </div>
        </div>
    );
}

export default Controls;