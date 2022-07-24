import altitude_pressure from '../src/img/altitude_pressure.js';
import altitude_ticks from '../src/img/altitude_ticks.js';
import fi_box from '../src/img/fi_box.js';
import fi_circle from '../src/img/fi_circle.js';
import fi_needle_small from '../src/img/fi_needle_small.js';
import fi_needle from '../src/img/fi_needle';
import fi_tc_airplane from '../src/img/fi_tc_airplane';
import heading_mechanics from '../src/img/heading_mechanics.js';
import heading_yaw from '../src/img/heading_yaw.js';
import horizon_ball from '../src/img/horizon_ball.js';
import horizon_circle from '../src/img/horizon_circle.js';
import horizon_mechanics from '../src/img/horizon_mechanics.js';
import speed_mechanics from '../src/img/speed_mechanics.js';
import turn_coordinator from '../src/img/turn_coordinator.js';
import vertical_mechanics from '../src/img/vertical_mechanics.js';
import horizon_back from '../src/img/horizon_back.js';
import React from 'react';

function _extends() {
  _extends = Object.assign || function (target) {
    for (var i = 1; i < arguments.length; i++) {
      var source = arguments[i];

      for (var key in source) {
        if (Object.prototype.hasOwnProperty.call(source, key)) {
          target[key] = source[key];
        }
      }
    }

    return target;
  };

  return _extends.apply(this, arguments);
}


var constants = {
  pitch_bound: 30,
  vario_bound: 1.95,
  airspeed_bound_l: 0,
  airspeed_bound_h: 160
};
var box = {
  width: '100%',
  height: '100%',
  position: 'absolute',
  top: 0,
  left: 0
};

var Instrument = function Instrument(_ref) {
  var children = _ref.children,
      showBox = _ref.showBox,
      size = _ref.size;
  return /*#__PURE__*/React.createElement("div", {
    className: "instrument heading",
    style: {
      height: size != null ? size : '250px',
      width: size != null ? size : '250px',
      position: 'relative',
      display: 'inline-block',
      overflow: 'hidden'
    }
  }, (showBox != null ? showBox : true) && /*#__PURE__*/React.createElement(fi_box, {
    className: "background box",
    style: box,
    alt: ""
  }), children);
};

export var HeadingIndicator = function HeadingIndicator(params) {
  var _params$heading;

  return /*#__PURE__*/React.createElement(Instrument, params, /*#__PURE__*/React.createElement("div", {
    className: "heading box",
    style: _extends({}, box, {
      transform: "rotate(" + ((_params$heading = -params.heading) != null ? _params$heading : 0) + "deg)"
    })
  }, /*#__PURE__*/React.createElement(heading_yaw, {
    className: "box",
    style: box,
    alt: ""
  })),
   /*#__PURE__*/React.createElement("div", {
    className: "mechanics box",
    style: box
  }, /*#__PURE__*/React.createElement(heading_mechanics, {
    className: "box",
    style: box,
    alt: ""
  }), /*#__PURE__*/React.createElement(fi_circle, {
    className: "box",
    style: box,
    alt: ""
  })));
};
export var Variometer = function Variometer(params) {
  var _params$vario;

  var vario = ((_params$vario = params.vario) != null ? _params$vario : 0) / 1000;
  if (vario > constants.vario_bound) vario = constants.vario_bound;else if (vario < -constants.vario_bound) vario = -constants.vario_bound;
  vario = vario * 90;
  return /*#__PURE__*/React.createElement(Instrument, params, /*#__PURE__*/React.createElement(vertical_mechanics, {
    className: "box",
    style: box,
    alt: ""
  }), /*#__PURE__*/React.createElement("div", {
    className: "vario box",
    style: _extends({}, box, {
      transform: "rotate(" + vario + "deg)"
    })
  }, /*#__PURE__*/React.createElement(fi_needle, {
    className: "box",
    style: box,
    alt: ""
  })), /*#__PURE__*/React.createElement("div", {
    className: "mechanics box",
    style: box
  }, /*#__PURE__*/React.createElement(fi_circle, {
    className: "box",
    style: box,
    alt: ""
  })));
};
export var TurnCoordinator = function TurnCoordinator(params) {
  return /*#__PURE__*/React.createElement(Instrument, params, /*#__PURE__*/React.createElement(turn_coordinator, {
    className: "box",
    style: box,
    alt: ""
  }), /*#__PURE__*/React.createElement("div", {
    className: "turn box",
    style: _extends({}, box, {
      transform: "rotate(" + params.turn + "deg)"
    })
  }, /*#__PURE__*/React.createElement(fi_tc_airplane, {
    className: "box",
    style: box,
    alt: ""
  })), /*#__PURE__*/React.createElement("div", {
    className: "mechanics box",
    style: box
  }, /*#__PURE__*/React.createElement(fi_circle, {
    className: "box",
    style: box,
    alt: ""
  })));
};
export var Airspeed = function Airspeed(params) {
  var _params$speed;

  var speed = (_params$speed = params.speed) != null ? _params$speed : 0;
  if (speed > constants.airspeed_bound_h) speed = constants.airspeed_bound_h;else if (speed < constants.airspeed_bound_l) speed = constants.airspeed_bound_l;
  speed = 90 + speed * 2;
  return /*#__PURE__*/React.createElement(Instrument, params, /*#__PURE__*/React.createElement(speed_mechanics, {
    className: "box",
    style: box,
    alt: ""
  }), /*#__PURE__*/React.createElement("div", {
    className: "speed box",
    style: _extends({}, box, {
      transform: "rotate(" + speed + "deg)"
    })
  }, /*#__PURE__*/React.createElement(fi_needle, {
    className: "box",
    style: box,
    alt: ""
  })), /*#__PURE__*/React.createElement("div", {
    className: "mechanics box",
    style: box
  }, /*#__PURE__*/React.createElement(fi_circle, {
    className: "box",
    style: box,
    alt: ""
  })));
};
export var Altimeter = function Altimeter(params) {
  var _params$altitude, _params$pressure;

  var altitude = (_params$altitude = params.altitude) != null ? _params$altitude : 0;
  var needle = 90 + altitude % 1000 * 360 / 1000;
  var needleSmall = altitude / 10000 * 360;
  var pressure = (_params$pressure = params.pressure) != null ? _params$pressure : 1013.25;
  pressure = 2 * pressure - 1980;
  return /*#__PURE__*/React.createElement(Instrument, params, /*#__PURE__*/React.createElement("div", {
    className: "pressure box",
    style: _extends({}, box, {
      transform: "rotate(" + pressure + "deg)"
    })
  }, /*#__PURE__*/React.createElement(altitude_pressure, {
    className: "box",
    style: box,
    alt: ""
  })), /*#__PURE__*/React.createElement(altitude_ticks, {
    className: "box",
    style: box,
    alt: ""
  }), /*#__PURE__*/React.createElement("div", {
    className: "needleSmall box",
    style: _extends({}, box, {
      transform: "rotate(" + needleSmall + "deg)"
    })
  }, /*#__PURE__*/React.createElement(fi_needle_small, {
    className: "box",
    style: box,
    alt: ""
  })), /*#__PURE__*/React.createElement("div", {
    className: "needle box",
    style: _extends({}, box, {
      transform: "rotate(" + needle + "deg)"
    })
  }, /*#__PURE__*/React.createElement(fi_needle, {
    className: "box",
    style: box,
    alt: ""
  })), /*#__PURE__*/React.createElement("div", {
    className: "mechanics box",
    style: box
  }, /*#__PURE__*/React.createElement(fi_circle, {
    className: "box",
    style: box,
    alt: ""
  })));
};
export var AttitudeIndicator = function AttitudeIndicator(params) {
  var _params$pitch, _params$roll;

  var pitch = (_params$pitch = params.pitch) != null ? _params$pitch : 0;

  if (pitch > constants.pitch_bound) {
    pitch = constants.pitch_bound;
  } else if (pitch < -constants.pitch_bound) {
    pitch = -constants.pitch_bound;
  }

  return /*#__PURE__*/React.createElement(Instrument, params, /*#__PURE__*/React.createElement("div", {
    className: "roll box",
    style: _extends({}, box, {
      top: '0%',
      transform: "rotate(" + ((_params$roll = params.roll) != null ? _params$roll : 0) + "deg)"
    })
  }, /*#__PURE__*/React.createElement(horizon_back, {
    className: "box",
    alt: "",
    style: _extends({}, box)
  }), /*#__PURE__*/React.createElement("div", {
    className: "pitch box",
    style: _extends({}, box, {
      top: pitch * 0.7 + "%"
    })
  }, /*#__PURE__*/React.createElement(horizon_ball, {
    className: "box",
    style: box,
    alt: ""
  })), /*#__PURE__*/React.createElement(horizon_circle, {
    className: "box",
    style: box,
    alt: ""
  })), /*#__PURE__*/React.createElement("div", {
    className: "mechanics box",
    style: box
  }, /*#__PURE__*/React.createElement(horizon_mechanics, {
    className: "box",
    style: box,
    alt: ""
  }), /*#__PURE__*/React.createElement(fi_circle, {
    className: "box",
    style: box,
    alt: ""
  })));
};

//exports.Airspeed = Airspeed;
//exports.Altimeter = Altimeter;
//exports.AttitudeIndicator = AttitudeIndicator;
//exports.HeadingIndicator = HeadingIndicator;
//exports.TurnCoordinator = TurnCoordinator;
//exports.Variometer = Variometer;
//export  {HeadingIndicator};
//# sourceMappingURL=index.js.map