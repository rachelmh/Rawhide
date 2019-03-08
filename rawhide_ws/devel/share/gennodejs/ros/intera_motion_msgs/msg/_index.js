
"use strict";

let TrajectoryOptions = require('./TrajectoryOptions.js');
let TrajectoryAnalysis = require('./TrajectoryAnalysis.js');
let WaypointOptions = require('./WaypointOptions.js');
let WaypointSimple = require('./WaypointSimple.js');
let Trajectory = require('./Trajectory.js');
let TrackingOptions = require('./TrackingOptions.js');
let EndpointTrackingError = require('./EndpointTrackingError.js');
let MotionStatus = require('./MotionStatus.js');
let Waypoint = require('./Waypoint.js');
let JointTrackingError = require('./JointTrackingError.js');
let InterpolatedPath = require('./InterpolatedPath.js');
let MotionCommandResult = require('./MotionCommandResult.js');
let MotionCommandActionResult = require('./MotionCommandActionResult.js');
let MotionCommandActionFeedback = require('./MotionCommandActionFeedback.js');
let MotionCommandFeedback = require('./MotionCommandFeedback.js');
let MotionCommandAction = require('./MotionCommandAction.js');
let MotionCommandGoal = require('./MotionCommandGoal.js');
let MotionCommandActionGoal = require('./MotionCommandActionGoal.js');

module.exports = {
  TrajectoryOptions: TrajectoryOptions,
  TrajectoryAnalysis: TrajectoryAnalysis,
  WaypointOptions: WaypointOptions,
  WaypointSimple: WaypointSimple,
  Trajectory: Trajectory,
  TrackingOptions: TrackingOptions,
  EndpointTrackingError: EndpointTrackingError,
  MotionStatus: MotionStatus,
  Waypoint: Waypoint,
  JointTrackingError: JointTrackingError,
  InterpolatedPath: InterpolatedPath,
  MotionCommandResult: MotionCommandResult,
  MotionCommandActionResult: MotionCommandActionResult,
  MotionCommandActionFeedback: MotionCommandActionFeedback,
  MotionCommandFeedback: MotionCommandFeedback,
  MotionCommandAction: MotionCommandAction,
  MotionCommandGoal: MotionCommandGoal,
  MotionCommandActionGoal: MotionCommandActionGoal,
};
