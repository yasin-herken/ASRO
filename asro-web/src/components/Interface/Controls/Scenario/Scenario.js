import React  from 'react';

import "./Scenario.css"
import {useDispatch} from "react-redux";
import Start from "../../../../static/Start.svg";
import {SCENARIO_1, SCENARIO_2,SCENARIO_3,SCENARIO_4,SCENARIO_5} from "../../../../redux/Agent/AgentTypes";

function Scenario({name}) {
    const dispatch = useDispatch();

    function OnClick(event) {
        var cmd;
        if (event.target.firstChild.nodeName === "H3") {
            cmd = event.target.firstChild.innerText;
        } else {
            cmd = event.target.firstChild.data;
        }
        switch (cmd) {
            case "Scenario 1":
                dispatch({
                    type: SCENARIO_1,
                    payload: {}
                })
                break;
            case "Scenario 2":
                dispatch({
                    type: SCENARIO_2,
                    payload: {}
                })
                break;
            case "Scenario 3":
                dispatch({
                    type: SCENARIO_3,
                    payload: {}
                })
                break;
            case "Scenario 4":
                dispatch({
                    type: SCENARIO_4,
                    payload: {}
                })
                break;
            case "Scenario 5":
                dispatch({
                    type: SCENARIO_5,
                    payload: {}
                })
                break;
            default:
                alert("??");
                break;
        }
    }
    return (
        <div className='Scenario' onClick={OnClick}>
            <h3 className='Scenario-Name'>
                {name}
            </h3>
            <button type='button' className='Scenario-Button' >
                <img src={Start} width="20px" className='Scenario-Img'/>
            </button>
        </div>
    );
}

export default Scenario;