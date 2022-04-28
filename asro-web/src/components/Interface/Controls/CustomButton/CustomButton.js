import React  from 'react';

import "./CustomButton.css"

import {useDispatch} from "react-redux";
import {TAKEOFF, LAND,LAND_ALL,TAKE_OFF_ALL} from "../../../../redux/Agent/AgentTypes";

function CustomButton({name}) {
    const dispatch = useDispatch();

    function OnClick(event) {
        var cmd;
        if (event.target.firstChild.nodeName === "H3") {
            cmd = event.target.firstChild.innerText;
        } else {
            cmd = event.target.firstChild.data;
        }

        switch (cmd) {
            case "TAKE OFF":
                dispatch({
                    type: TAKEOFF,
                    payload: {}
                })
                break;
            case "LAND":
                dispatch({
                    type: LAND,
                    payload: {}
                })
                break;
            case "TAKE_OFF_ALL":
                dispatch({
                    type: TAKE_OFF_ALL,
                    payload: {}
                })
                break;
            case "LAND_ALL":
                dispatch({
                    type: LAND_ALL,
                    payload: {}
                })
                break;
            default:
                break;
        }
    }

    return (
        <div className='CustomButton' onClick={OnClick}>
            <h3 className='CustomButton-Name'>
                {name}
            </h3>
        </div>
    );
}

export default CustomButton;