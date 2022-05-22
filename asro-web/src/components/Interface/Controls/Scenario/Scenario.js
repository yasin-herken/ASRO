

import "./Scenario.css";
import React  from 'react';
import {useDispatch} from "react-redux";
import Start from "../../../../static/Start.svg";

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
                dispatch();
                break;
            case "Scenario 2":
                dispatch();
                break;
            case "Scenario 3":
                dispatch();
                break;
            case "Scenario 4":
                dispatch();
                break;
            case "Scenario 5":
                dispatch();
                break;
            default:
                break;
        }
    }
    return (
        <div className='Scenario' onClick={OnClick}>
            <h3 className='Scenario-Name'>
                {name}
            </h3>
            <button type='button' className='Scenario-Button' >
                <img src={Start} alt={"Start " + name} width="20px" className='Scenario-Img'/>
            </button>
        </div>
    );
}

export default Scenario;