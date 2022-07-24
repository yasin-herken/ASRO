import {
    getAgents,
    getAgentPose
} from "../../../actions/AgentActions";

import {
    Airspeed,
    Altimeter,
    AttitudeIndicator,
    HeadingIndicator,
    TurnCoordinator,
    Variometer
} from './SvgRun/index.js'
import "./Dashboard.css";
import Container from 'react-bootstrap/Container';
import React, { useEffect }  from 'react';
import {useSelector} from "react-redux";
import {useDispatch} from "react-redux";
function Dashboard() {
    const asroState = useSelector((state) => state.asro);
    const dispatch = useDispatch();

    useEffect(() => {
        dispatch(getAgents());
    }, [dispatch]);

    
    useEffect(() => {
        setInterval(() => {
            if (asroState.selectedAgent.name !== "") {
                dispatch(getAgentPose());
            }

        }, 1000);
      }, [asroState.selectedAgent.name, dispatch]);

    return (        
            <Container className="Dashboard">
                <h2 className='Dashboard-Title'>
                    Dashboard
                </h2>
                <div className="Dashboard-Indicators-Container">
                    <Airspeed speed={Math.random() * 160} showBox={false} />
                    
                    <HeadingIndicator heading={Math.random() * 360} showBox={false} />
                    
                    <AttitudeIndicator roll={(Math.random() - 0.5) * 120} pitch={(Math.random() - 0.5) * 40} showBox={false} />
                    
                </div>
            </Container>
    );
}

export default Dashboard;