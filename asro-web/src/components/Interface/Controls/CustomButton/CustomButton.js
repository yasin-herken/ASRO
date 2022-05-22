import {
    takeOff,
    takeOffAll,
    land,
    landAll
} from "../../../../actions/AgentActions";
import "./CustomButton.css"

import React  from 'react';
import {useDispatch} from "react-redux";

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
                dispatch(takeOff())
                break;
            case "LAND":
                dispatch(land())
                break;
            case "TAKE_OFF_ALL":
                dispatch(takeOffAll())
                break;
            case "LAND_ALL":
                dispatch(landAll())
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