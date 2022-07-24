import React  from 'react';

import "./NavigationBar.css";

import Title from "../../static/Title.js";

function NavigatioBar() {
    return (
        <div className="NavigationBar">
            <Title alt="YILDIZLARARASI Title" className='Title' />
        </div>
    );
}

export default NavigatioBar;