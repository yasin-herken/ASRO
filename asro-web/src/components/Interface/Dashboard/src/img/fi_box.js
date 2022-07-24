import * as React from "react";

const SVGComponent = (props) => (
  console.log(props)
  (
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
        fill="#110F0F"
        d="M387.667,375c0,6.627-5.373,12-12,12h-350c-6.627,0-12-5.373-12-12V25c0-6.627,5.373-12,12-12h350 c6.627,0,12,5.373,12,12V375z"
      />
      <path
        fill="none"
        stroke="#FFFFFF"
        strokeWidth={0.5}
        strokeMiterlimit={10}
        d="M387.667,375c0,6.627-5.373,12-12,12h-350 c-6.627,0-12-5.373-12-12V25c0-6.627,5.373-12,12-12h350c6.627,0,12,5.373,12,12V375z"
      />
    </g>
  </svg>)
);

export default SVGComponent;
