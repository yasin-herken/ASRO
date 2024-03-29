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
      <polygon
        fill="#FFFFFF"
        stroke="#B2B2B2"
        strokeWidth={0.5}
        strokeMiterlimit={10}
        points="76.445,196.417 68.082,200  76.423,203.583 184.334,203.583 184.334,196.417  "
      />
      <path
        fill="#232323"
        stroke="#353535"
        strokeWidth={0.5}
        strokeMiterlimit={10}
        d="M239.042,196.417 c-3.563-3.563-8.918,0-13.063,0c-0.787,0-3.148,0-3.148,0h-11.969c-1.51-4.271-5.573-7.337-10.362-7.337s-8.852,3.065-10.362,7.337 h-5.804v7.167h5.745c1.464,4.355,5.572,7.496,10.42,7.496s8.956-3.141,10.42-7.496h12.036c0,0,2.314,0,3.085,0 c3.874,0,9.458,3.542,13,0C241.124,201.501,241.25,198.625,239.042,196.417z"
      />
    </g>
  </svg>
);

export default SVGComponent;
