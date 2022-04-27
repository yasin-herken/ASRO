import React  from 'react';

import "./Interface.css"

import Agents from "./Agents/Agents";
import Dashboard from "./Dashboard/Dashboard";
import Controls from "./Controls/Controls";

function Interface() {
    return (
        <div className="Interface">
            <Agents />
            <Dashboard />
            <Controls />
        </div>
    );
}

export default Interface;