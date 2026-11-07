import assert from "node:assert/strict";
import test from "node:test";
import { URDFJoint, URDFLink, URDFRobot } from "urdf-loader/src/URDFClasses.js";
import { trajectoryPoints } from "../src/trajectory-layer.js";

test("planned joint positions produce root-frame tip positions without moving the live robot", () => {
  const robot = new URDFRobot();
  robot.urdfName = "base_link";
  robot.position.set(4, 3, 2);
  const joint = new URDFJoint();
  joint.urdfName = "slide";
  joint.jointType = "prismatic";
  joint.limit.lower = -2;
  joint.limit.upper = 2;
  const tip = new URDFLink();
  tip.urdfName = "grasp_frame";
  tip.position.set(0, 0, 0.5);
  robot.add(joint);
  joint.add(tip);
  robot.links = { base_link: robot, grasp_frame: tip };
  robot.joints = { slide: joint };
  robot.colliders = {};
  robot.visual = {};
  const points = trajectoryPoints(robot, {
    joint_names: ["slide"], points: [{ positions: [0] }, { positions: [1] }],
  });
  assert.deepEqual(points.map((point) => point.toArray()), [[0, 0, 0.5], [1, 0, 0.5]]);
  assert.equal(joint.jointValue[0], 0);
  assert.deepEqual(robot.position.toArray(), [4, 3, 2]);
});
