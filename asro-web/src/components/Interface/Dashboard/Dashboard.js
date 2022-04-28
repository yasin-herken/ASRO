import React, { }  from 'react';
import Plot from 'react-plotly.js';
import {useSelector} from "react-redux";

import "./Dashboard.css";

import Module1 from "../../../static/Module1.svg";
import Module2 from "../../../static/Module2.svg";
import Module3 from "../../../static/Module3.svg";
import Module4 from "../../../static/Module4.svg";

import InfoTuple from './InfoTuple/InfoTuple';

function Dashboard() {
    const agent = useSelector((state) => state.agent);
    
    function UpdateBorder() {
        var plotDiv = document.getElementsByClassName("main-svg"); //class name of plotly main area
        plotDiv[0].style.borderRadius = "32px"; //or however round you want
        return;
    }

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
                    {(agent.selectedAgentName === "" ? "Please Select an Agent" : agent.selectedAgentName)}
                    </h2>
                    <div className='Dashboard-Indicators-Container'>
                        <img src={Module1} width='20%'/>
                        <img src={Module2} width='20%'/>
                        <img src={Module3} width='20%'/>
                        <img src={Module4} width='20%'/>
                    </div>
                    <div className='Dashboard-Indicators-Info'>
                        <InfoTuple param="x" value="0.0"/>
                        <InfoTuple param="y" value="0.0"/>
                        <InfoTuple param="z" value="0.0"/>
                        <InfoTuple 
                            param="state"
                            value={(agent.selectedAgentState === "" ? "N/A" : agent.selectedAgentState)}/>
                    </div>
                </div>
            </div>
        </div>
    );
}

export default Dashboard;