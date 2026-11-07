import * as THREE from "three";

/**
 * Calculate end-effector positions from a planned joint trajectory without moving the displayed model.
 * @param {object} model - Loaded URDF robot model.
 * @param {object} message - ROS JointTrajectory message.
 * @param {string} tipName - Name of the desired URDF link.
 * @returns {THREE.Vector3[]} Positions in the robot root coordinate frame.
 */
export function trajectoryPoints(model, message, tipName = "grasp_frame") {
  if (!model || !Array.isArray(message?.joint_names) || !Array.isArray(message?.points)) return [];
  const robot = model.clone();
  robot.position.set(0, 0, 0);
  robot.quaternion.identity();
  robot.scale.set(1, 1, 1);
  const tip = robot.links?.[tipName];
  if (!tip) return [];
  const points = [];
  for (const waypoint of message.points) {
    if (!Array.isArray(waypoint.positions) || waypoint.positions.length !== message.joint_names.length ||
        !waypoint.positions.every(Number.isFinite)) continue;
    message.joint_names.forEach((name, index) => robot.setJointValue(name, waypoint.positions[index]));
    robot.updateMatrixWorld(true);
    points.push(tip.getWorldPosition(new THREE.Vector3()));
  }
  return points;
}
