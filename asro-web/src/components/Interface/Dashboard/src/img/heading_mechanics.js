import * as React from "react";

const SVGComponent = (props) => (
  <svg
    id="Layer_2"
    xmlns="http://www.w3.org/2000/svg"
    xmlnsXlink="http://www.w3.org/1999/xlink"
    x="0px"
    y="0px"
    width="400.667px"
    height="400.666px"
    viewBox="0 0 400.667 400.666"
    enableBackground="new 0 0 400.667 400.666"
    xmlSpace="preserve"
    {...props}
  >
    <filter filterUnits="objectBoundingBox" id="AI_Shadow_1">
      <feGaussianBlur stdDeviation={5} result="blur" in="SourceAlpha" />
      <feOffset dy={0} dx={0} result="offsetBlurredAlpha" in="blur" />
      <feMerge>
        <feMergeNode in="offsetBlurredAlpha" />
        <feMergeNode in="SourceGraphic" />
      </feMerge>
    </filter>
    <g filter="url(#AI_Shadow_1)">
      <path
        fill="none"
        stroke="#FF2A00"
        strokeWidth={3}
        strokeMiterlimit={10}
        d="M200.38,81.417c0,0-7.042,11.625-11.292,24.75 s-7.125,51.375-7.125,51.375l-54.213,39.3c0,0-13.543,8.502-16.3,15.598c-0.958,2.468-1.491,5.852-1.612,7.852 c-0.222,3.681,0,13.749,0,13.75c0.125,2.125,1.5,3.875,6,2.375s69.5-23.125,69.5-23.125l2.75,54.25c0,0-16.723,12.949-21.875,17.75 c-1.232,1.148-3.054,2.765-4.063,4.875c-1.045,2.187-1.161,5.537-1.188,7.25c-0.038,2.437-0.131,8.147,0.188,9.063 c0.5,1.438,1.063,1.918,3.135,1.918c2.625,0,12.928-5.73,12.928-5.73l18.75-9.25l4.333,12.25l-4.333-12.25l4.333,12.25h0.168 l4.333-12.25l18.75,9.25c0,0,10.303,5.73,12.928,5.73c2.072,0,2.635-0.48,3.135-1.918c0.318-0.916,0.226-6.625,0.188-9.063 c-0.027-1.713-0.143-5.063-1.188-7.25c-1.008-2.11-2.83-3.727-4.063-4.875c-5.152-4.801-21.875-17.75-21.875-17.75l2.75-54.25 c0,0,65,21.625,69.5,23.125s5.875-0.25,6-2.375c0-0.001,0.222-10.069,0-13.75c-0.121-2-0.653-5.384-1.612-7.852 c-2.756-7.096-16.3-15.598-16.3-15.598l-54.213-39.3c0,0-2.875-38.25-7.125-51.375S200.38,81.417,200.38,81.417V61"
      />
    </g>
    <g filter="url(#AI_Shadow_1)">
      <polygon
        fill="#FF2A00"
        points="200.381,332.311 194.542,344.439 206.542,344.439  "
      />
    </g>
    <g filter="url(#AI_Shadow_1)">
      <polygon
        fill="#FFFFFF"
        points="294.272,294.239 298.743,307.01 307.272,298.481  "
      />
    </g>
    <g filter="url(#AI_Shadow_1)">
      <polygon
        fill="#FFFFFF"
        points="105.938,294.771 94.146,298.898 102.02,306.771  "
      />
    </g>
    <g filter="url(#AI_Shadow_1)">
      <polygon
        fill="#FF2A00"
        points="68.291,200.382 56.291,194.604 56.291,206.477  "
      />
    </g>
    <g filter="url(#AI_Shadow_1)">
      <polygon
        fill="#FFFFFF"
        points="106.001,106.041 101.801,94.041 93.787,102.055  "
      />
    </g>
    <g filter="url(#AI_Shadow_1)">
      <polygon
        fill="#FFFFFF"
        points="294.731,105.894 306.519,101.767 298.646,93.894  "
      />
    </g>
    <g filter="url(#AI_Shadow_1)">
      <polygon
        fill="#FF2A00"
        points="332.311,200.285 344.44,206.125 344.441,194.125  "
      />
    </g>
    <g filter="url(#AI_Shadow_1)">
      <polygon
        fill="#FF2A00"
        points="200.619,68.022 206.458,55.893 194.458,55.892  "
      />
    </g>
  </svg>
);

export default SVGComponent;
