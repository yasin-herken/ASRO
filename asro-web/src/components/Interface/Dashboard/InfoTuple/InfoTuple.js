import React, { }  from 'react';

import "./InfoTuple.css"

function InfoTuple({param, value}) {
    return (
        <div className='InfoTuple'>
            <div className='InfoTuple-Param'>
                {param}
            </div>
            <h3 className='InfoTuple-Value'>
                {value}
            </h3>
        </div>
    );
}

export default InfoTuple;