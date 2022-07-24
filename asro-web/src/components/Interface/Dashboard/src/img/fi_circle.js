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
        fill="#232323"
        d="M200.333,33.5c-91.956,0-166.5,74.544-166.5,166.5s74.544,166.5,166.5,166.5 c91.957,0,166.5-74.544,166.5-166.5S292.29,33.5,200.333,33.5z M200.667,350.099c-82.714,0-149.767-67.053-149.767-149.767 c0-82.713,67.053-149.767,149.767-149.767s149.767,67.053,149.767,149.767C350.433,283.046,283.38,350.099,200.667,350.099z"
      />
      <path
        fill="none"
        stroke="#353535"
        strokeWidth={1.3}
        strokeMiterlimit={10}
        d="M200.333,33.5 c-91.956,0-166.5,74.544-166.5,166.5s74.544,166.5,166.5,166.5c91.957,0,166.5-74.544,166.5-166.5S292.29,33.5,200.333,33.5z  M200.667,350.099c-82.714,0-149.767-67.053-149.767-149.767c0-82.713,67.053-149.767,149.767-149.767 s149.767,67.053,149.767,149.767C350.433,283.046,283.38,350.099,200.667,350.099z"
      />
    </g>
  </svg>
);

export default SVGComponent;
