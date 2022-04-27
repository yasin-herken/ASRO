import React  from 'react';

import "./NavigationBar.css";

import Title from "../../static/Title.svg"

function NavigatioBar() {
    return (
        <div className="NavigationBar">
            <img src={Title} alt="YILDIZLARARASI Title" className='Title'/>
        </div>
    );
}

export default NavigatioBar;