import React, { }  from 'react';
import Plot from 'react-plotly.js';
import {
    Airspeed,
    Altimeter,
    AttitudeIndicator,
    HeadingIndicator,
    Variometer
} from 'react-flight-indicators';

import "./Dashboard.css";

function Dashboard() {
    
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
                        style={{width: '100%'}}
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
                    
                </div>
            </div>
        </div>
    );
}

export default Dashboard;