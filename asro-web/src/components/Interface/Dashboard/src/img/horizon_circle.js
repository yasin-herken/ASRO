import * as React from "react";

const SVGComponent = (props) => (
  <svg
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
    <g id="Layer_2">
      <linearGradient
        id="SVGID_2_"
        gradientUnits="userSpaceOnUse"
        x1={200.667}
        y1={349.9331}
        x2={200.667}
        y2={50.3999}
      >
        <stop
          offset={0.4999}
          style={{
            stopColor: "#503723",
          }}
        />
        <stop
          offset={0.5001}
          style={{
            stopColor: "#558EBB",
          }}
        />
      </linearGradient>
      <path
        fill="url(#SVGID_2_)"
        filter="url(#AI_Shadow_1)"
        d="M200.667,50.4C117.953,50.4,50.9,117.453,50.9,200.167 s67.053,149.767,149.767,149.767s149.767-67.053,149.767-149.767S283.38,50.4,200.667,50.4z M200.667,300.5 c-55.413,0-100.334-44.921-100.334-100.334c0-55.412,44.921-100.333,100.334-100.333C256.079,99.833,301,144.753,301,200.166 C301,255.579,256.079,300.5,200.667,300.5z"
      />
      <line
        fill="none"
        stroke="#FFFFFF"
        strokeWidth={4}
        strokeMiterlimit={10}
        x1={50.667}
        y1={200.333}
        x2={100.333}
        y2={200.333}
      />
      <line
        fill="none"
        stroke="#FFFFFF"
        strokeWidth={4}
        strokeMiterlimit={10}
        x1={300.749}
        y1={200.333}
        x2={350.433}
        y2={200.333}
      />
      <line
        fill="none"
        stroke="#FFFFFF"
        strokeWidth={4}
        strokeMiterlimit={10}
        x1={78.415}
        y1={131.582}
        x2={112.082}
        y2={151.915}
      />
      <line
        fill="none"
        stroke="#FFFFFF"
        strokeWidth={4}
        strokeMiterlimit={10}
        x1={288.751}
        y1={151.915}
        x2={323.585}
        y2={131.582}
      />
      <line
        fill="none"
        stroke="#FFFFFF"
        strokeWidth={4}
        strokeMiterlimit={10}
        x1={149.418}
        y1={113.75}
        x2={129.418}
        y2={79.25}
      />
      <line
        fill="none"
        stroke="#FFFFFF"
        strokeWidth={4}
        strokeMiterlimit={10}
        x1={252.084}
        y1={113.75}
        x2={272.751}
        y2={79.25}
      />
      <polygon
        fill="#FFFFFF"
        points="185.918,60.085 215.251,60.085 200.585,98.918  "
      />
      <polygon
        fill="#FFFFFF"
        points="119.084,117.417 112.082,124.251 126.584,131.582  "
      />
      <polygon
        fill="#FFFFFF"
        points="274.084,131.582 281.251,117.417 288.751,124.5  "
      />
    </g>
  </svg>
);

export default SVGComponent;
