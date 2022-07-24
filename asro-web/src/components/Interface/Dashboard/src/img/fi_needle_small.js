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
        points="211.838,160.802 200.343,132.368 200.338,132.386 200.333,132.368 188.838,160.802 200.333,207  200.338,206.977 200.343,207  "
      />
      <polygon
        fill="none"
        stroke="#B2B2B2"
        strokeWidth={0.5}
        strokeMiterlimit={10}
        points="211.838,160.802 200.343,132.368  200.338,132.386 200.333,132.368 188.838,160.802 200.333,207 200.338,206.977 200.343,207  "
      />
    </g>
    <g filter="url(#AI_Shadow_1)">
      <path
        fill="#232323"
        d="M200.331,196.091c0,0-9.492,18.192-9.331,24.818c0.062,2.56,1.828,4.228,9.331,4.228 s9.114-1.709,9.146-4.228c0.083-6.507-8.809-24.493-8.809-24.493"
      />
      <path
        fill="none"
        stroke="#353535"
        strokeWidth={0.5}
        strokeMiterlimit={10}
        d="M200.331,196.091c0,0-9.492,18.192-9.331,24.818 c0.062,2.56,1.828,4.228,9.331,4.228s9.114-1.709,9.146-4.228c0.083-6.507-8.809-24.493-8.809-24.493"
      />
    </g>
  </svg>
);

export default SVGComponent;
