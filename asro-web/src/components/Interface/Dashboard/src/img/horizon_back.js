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
    <linearGradient
      id="backHorizon_1_"
      gradientUnits="userSpaceOnUse"
      x1={200.667}
      y1={350.0991}
      x2={200.667}
      y2={50.5659}
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
    <circle
      id="backHorizon"
      fill="url(#backHorizon_1_)"
      cx={200.667}
      cy={200.333}
      r={149.767}
    />
  </svg>
);

export default SVGComponent;
