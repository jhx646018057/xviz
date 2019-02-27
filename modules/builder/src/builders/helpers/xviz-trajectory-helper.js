// Copyright (c) 2019 Uber Technologies, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import {_Pose as Pose} from 'math.gl';
import * as turf from '@turf/turf';
import {addMetersToLngLat} from 'viewport-mercator-project';

/**
 * Given vertices and a base pose, transform the vertices to `basePose` relative coordinates
 * @param vertices {Array} list of [x, y, z] or [x, y]
 * @param basePose {Object} {x, y, z, longitude, latitude, altitude, roll, pitch, yaw}
 * @returns {Array} list of vertices in relative coordinates
 */
export function getRelativeCoordinates(vertices, basePose) {
  if (!(basePose instanceof Pose)) {
    basePose = new Pose(basePose);
  }

  const transformMatrix = basePose.getTransformationMatrix();
  return vertices.map(p => transformMatrix.transformVector(p));
}

/**
 * Generate trajectory for list of poses with given start frame and end frame
 * @param poses {Array}, frames of pose data,
 *   each frame contains a `pose` entry with {x, y, z, longitude, latitude, altitude, roll, pitch, yaw}
 * @param startFrame {Number}, start frame of trajectory
 * @param endFrame {Number}, end frame of trajectory
 * @returns {Array} trajectory, list of vertices
 */
export function getPoseTrajectory({poses, startFrame, endFrame}) {
  const positions = [];
  const iterationLimit = Math.min(endFrame, poses.length);

  for (let i = startFrame; i < iterationLimit; i++) {
    positions.push(poses[i].pose);
  }

  const startPose = poses[startFrame].pose;
  // world coordinate system to startPose coordinate system
  const transformMatrix = new Pose(startPose).getTransformationMatrix().invert();

  return positions.map(currPose => {
    // offset vector in world coordinate system
    const offset = getGeospatialVector(startPose, currPose);

    // transform currPose to startPose coordinate system
    const relativeOffset = transformMatrix.transformVector(offset);

    return [relativeOffset.x, relativeOffset.y, relativeOffset.z];
  });
}

/**
 * Return transform matrix that can be used to transform
 * data in `from` pose coordinate system into the `to` pose coordinate system
 *
 * @param from {Object} {longitude, latitude, pitch, roll, yaw}
 * @param to {Object} {longitude, latitude, pitch, roll, yaw}
 * @returns {Object} transformation matrix that converts 'from' relative coordinates into 'to' relative coordinates
 */
export function getGeospatialToPoseTransform(from, to) {
  // Since 'to' is the target, get the vector from 'to -> from'
  // and use that to set the position of 'from Pose'
  const offset = getGeospatialVector(from, to);

  const fromPose = new Pose({
    x: 0,
    y: 0,
    z: 0,
    pitch: from.pitch,
    roll: from.roll,
    yaw: from.yaw
  });

  const toPose = new Pose({
    x: offset[0],
    y: offset[1],
    z: offset[2],
    pitch: to.pitch,
    roll: to.roll,
    yaw: to.yaw
  });

  // there is a bug in math.gl https://github.com/uber-web/math.gl/issues/33
  // pose.getTransformationMatrixFromPose and pose.getTransformationMatrixFromPose are flipped
  return fromPose.getTransformationMatrixFromPose(toPose);
}

/**
 * Get object trajectory in pose relative coordinates
 * @param targetObject {Object} {id, x, y, z, ...}
 * @param objectFrames {Array}, all the frames of objects, (object: {id, x, y, z})
 * @param poseFrames {Array}, all the frames of base poses (pose: {longitude, latitude, altitude})
 * @param startFrame {Number}, start frame of trajectory
 * @param endFrame {Number}, end frame of trajectory
 * @returns {Array} trajectory, list of vertices
 */
export function getObjectTrajectory({
  targetObject,
  objectFrames,
  poseFrames,
  startFrame,
  endFrame
}) {
  const vertices = [];
  const startVehiclePose = poseFrames[startFrame].pose;
  const limit = Math.min(endFrame, targetObject.lastFrame);
  const motions = getObjectMotions(targetObject, objectFrames, startFrame, limit);

  for (let i = 0; i < motions.length; i++) {
    const step = motions[i];

    const currVehiclePose = poseFrames[startFrame + i].pose;

    // matrix to convert data from currVehiclePose relative to startVehiclePose relative.
    const transformMatrix = getGeospatialToPoseTransform(currVehiclePose, startVehiclePose);

    // objects in curr frame are meters offset based on currVehiclePose
    // need to convert to the coordinate system of the startVehiclePose
    const p = transformMatrix.transformVector([step.x, step.y, step.z]);
    vertices.push([p.x, p.y, p.z]);
  }

  return vertices;
}

/* eslint-disable complexity */
/**
 * Get the meter vector from Geospatial coordinates in world coordinate system
 *
 * @param from {Object} {longitude, latitude, altitude, x, y, z}
 * @param to {Object} {longitude, latitude, altitude, x, y, z}
 * @returns {Array} Vector [x, y, z] in meters
 */
export function getGeospatialVector(from, to) {
  const fromCoord = addMetersToLngLat(
    [from.longitude || 0, from.latitude || 0, from.altitude || 0],
    [from.x || 0, from.y || 0, from.z || 0]
  );

  const toCoord = addMetersToLngLat(
    [to.longitude || 0, to.latitude || 0, to.altitude || 0],
    [to.x || 0, to.y || 0, to.z || 0]
  );

  const distInMeters = turf.distance(fromCoord, toCoord, {units: 'meters'});

  // Bearing is degrees from north, positive is clockwise
  const bearing = turf.bearing(fromCoord, toCoord);
  const bearingInRadians = turf.degreesToRadians(bearing);

  const diffZ = toCoord[2] - fromCoord[2];

  return [
    distInMeters * Math.sin(bearingInRadians),
    distInMeters * Math.cos(bearingInRadians),
    diffZ
  ];
}
/* eslint-enable complexity */

function getFrameObjects(frames, frameNumber) {
  if (frames instanceof Map) {
    return frames.get(frameNumber);
  }
  if (frames instanceof Array) {
    return frames[frameNumber];
  }
  return null;
}

/**
 * Generate motions for target object
 * @param targetObject {Object} {startFrame, endFrame, id, x, y, z,...}
 * @param objectFrames {Map | Array}, either a Map (key is frameNumber, value is list of objects) or an array of frames
 * @param startFrame {Number}
 * @param endFrame {Number}
 * @returns {Array} list of motions from given startFrame to endFrame
 */
function getObjectMotions(targetObject, objectFrames, startFrame, endFrame) {
  startFrame = Math.max(targetObject.firstFrame, startFrame);
  endFrame = Math.min(targetObject.lastFrame, endFrame);

  const motions = [];
  for (let i = startFrame; i < endFrame; i++) {
    const objects = getFrameObjects(objectFrames, i);
    const object = objects.find(obj => obj.id === targetObject.id);
    motions.push(object);
  }

  return motions;
}
