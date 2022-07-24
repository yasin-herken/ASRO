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
    <line
      fill="none"
      stroke="#FF7F00"
      strokeWidth={5}
      strokeLinecap="round"
      strokeMiterlimit={10}
      x1={130}
      y1={200.457}
      x2={176}
      y2={200.457}
    />
    <line
      fill="none"
      stroke="#FF7F00"
      strokeWidth={5}
      strokeLinecap="round"
      strokeMiterlimit={10}
      x1={225}
      y1={200.457}
      x2={271}
      y2={200.457}
    />
    <line
      fill="none"
      stroke="#FF7F00"
      strokeWidth={5}
      strokeLinecap="round"
      strokeMiterlimit={10}
      x1={200.534}
      y1={200.457}
      x2={200.534}
      y2={200.457}
    />
    <line
      fill="none"
      stroke="#FF7F00"
      strokeWidth={5}
      strokeLinecap="round"
      strokeMiterlimit={10}
      x1={200.399}
      y1={103.417}
      x2={190.483}
      y2={130.583}
    />
    <line
      fill="none"
      stroke="#FF7F00"
      strokeWidth={5}
      strokeLinecap="round"
      strokeMiterlimit={10}
      x1={190.483}
      y1={130.583}
      x2={210.316}
      y2={130.583}
    />
    <line
      fill="none"
      stroke="#FF7F00"
      strokeWidth={5}
      strokeLinecap="round"
      strokeMiterlimit={10}
      x1={210.316}
      y1={130.583}
      x2={200.399}
      y2={103.417}
    />
    <g filter="url(#AI_Shadow_1)">
      <path
        fill="#232323"
        d="M82.663,298.167c28.157,33.81,70.564,55.333,118.004,55.333s89.847-21.524,118.004-55.333H82.663z"
      />
      <path
        fill="none"
        stroke="#353535"
        strokeWidth={1.3}
        strokeMiterlimit={10}
        d="M82.663,298.167 c28.157,33.81,70.564,55.333,118.004,55.333s89.847-21.524,118.004-55.333H82.663z"
      />
    </g>
    <line
      fill="none"
      stroke="#FFFFFF"
      strokeWidth={4}
      strokeMiterlimit={10}
      x1={200.667}
      y1={298.667}
      x2={200.667}
      y2={329.167}
    />
    <line
      fill="none"
      stroke="#FFFFFF"
      strokeWidth={4}
      strokeMiterlimit={10}
      x1={254.667}
      y1={298.667}
      x2={254.667}
      y2={308.917}
    />
    <line
      fill="none"
      stroke="#FFFFFF"
      strokeWidth={4}
      strokeMiterlimit={10}
      x1={146.667}
      y1={298.667}
      x2={146.667}
      y2={308.917}
    />
    <g filter="url(#AI_Shadow_1)" />
  </svg>
);

export default SVGComponent;
