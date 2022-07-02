import {
    getAgents,
    getAgentPose
} from "../../../actions/AgentActions";

import "./Dashboard.css";

import Module1 from "../../../static/Module1.svg";
import Module2 from "../../../static/Module2.svg";
import Module3 from "../../../static/Module3.svg";
import Module4 from "../../../static/Module4.svg";
import InfoTuple from './InfoTuple/InfoTuple';

import React, { useEffect }  from 'react';
import Plot from 'react-plotly.js';
import {useSelector} from "react-redux";
import {useDispatch} from "react-redux";

function Dashboard() {
    const asroState = useSelector((state) => state.asro);
    const dispatch = useDispatch();
    
    function UpdateBorder() {
        var plotDiv = document.getElementsByClassName("main-svg"); //class name of plotly main area
        plotDiv[0].style.borderRadius = "32px"; //or however round you want
        return;
    }

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
        <div className='Dashboard'>

            <h2 className='Dashboard-Title'>
                Dashboard
            </h2>
            <div className='Dashboard-Container'>
                <div className='Dashboard-Map'>
                    <Plot
                        id="plot-gang"
                        type='scatter'
                        mode='markers'
                        style={{width: '100%', height: '356px'}}
                        layout={
                            {
                                hovermode: false,
                                paper_bgcolor: 'rgba(15, 53, 15, 1)',
                                plot_bgcolor: 'rgba(0, 0, 0, 0)',
                                margin: {b: 24, l: 24, t: 24, r: 24},
                                yaxis: {
                                    gridcolor: 'rgba(177, 186, 177, 0.5)',
                                    zerolinecolor: 'rgba(255, 255, 255, 1)',
                                    tickfont : {
                                        color : 'rgba(255, 170, 0, 1)'
                                    }
                                },
                                xaxis: {
                                    gridcolor: 'rgba(177, 186, 177, 0.5)',
                                    zerolinecolor: 'rgba(255, 255, 255, 1)',
                                    tickfont : {
                                        color : 'rgba(255, 170, 0, 1)'
                                    }
                                }
                            }
                        }
                        config={{staticPlot: false, responsive: false}}
                        x={[-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]}
                        y={[-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]}
                        
                    />

                    <div style={{display: 'none'}}>
                        {setTimeout(function() { UpdateBorder(); }, 500)}
                    </div>
                </div>
                <div className='Dashboard-Indicators'>
                    <h2 className='Dashboard-Indicators-Name'>
                    {(asroState.selectedAgent.name === "" ? "Please Select an Agent" : asroState.selectedAgent.name)}
                    </h2>
                    <div className='Dashboard-Indicators-Container'>
                        <img src={Module1} width='20%' alt='Module1'/>
                        <img src={Module2} width='20%' alt='Module2'/>
                        <img src={Module3} width='20%' alt='Module3'/>
                        <img src={Module4} width='20%' alt='Module4'/>
                    </div>
                    <div className='Dashboard-Indicators-Info'>
                        <InfoTuple
                            param="x"
                            value={(asroState.selectedAgent.coords.x === null ? "N/A" : asroState.selectedAgent.coords.x)}/>
                        <InfoTuple
                            param="y"
                            value={(asroState.selectedAgent.coords.y === null ? "N/A" : asroState.selectedAgent.coords.y)}/>
                        <InfoTuple
                            param="z"
                            value={(asroState.selectedAgent.coords.z === null ? "N/A" : asroState.selectedAgent.coords.z)}/>
                        <InfoTuple 
                            param="state"
                            value={(asroState.selectedAgent.state === null ? "N/A" : asroState.selectedAgent.state)}/>
                    </div>
                </div>
            </div>
        </div>
    );
}

export default Dashboard;