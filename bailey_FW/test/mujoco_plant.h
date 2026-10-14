// Simulator option (-DSIM_MUJOCO, built by build_mujoco.sh): the robot's
// physics from the MuJoCo engine instead of the equations in Plant::step.
//
// A 3D robot: the body on a free joint (it can pitch, roll, turn and fall
// over), two wheels hinged to it and rolling on the floor with friction (so
// they can slip), and each stepper as a stiff position servo that turns its
// wheel relative to the body along the step rate FastAccelStepper produces.
// The sensors, the firmware and the scenarios are the same as with the
// built-in equations; Plant's geometry and masses build the model.
#include <mujoco/mujoco.h>

struct MjWorld {
  mjModel *m = nullptr;
  mjData *d = nullptr;
  int builtFor = -1;              // Plant::id the model was built for
  int qBody = 0, vBody = 0, qWl = 0, qWr = 0, vWl = 0, vWr = 0, body = 0, wheelGeom = 0;
  double phiL = 0, phiR = 0;      // commanded wheel angles relative to the body
  // What the scenario last saw: if it changes Plant's state (a push, letting
  // go, picking it up), the same change is made to MuJoCo's state.
  double theta = 0, omega = 0, vl = 0, vr = 0;
};
static MjWorld mjw;
static const double MJ_TRACK = 0.20;   // m between the wheels

// Pitch (rad, + = leaning forward), heading (rad) and the body's rotation.
static void mjPose(double &pitch, double &heading, mjtNum R[9]) {
  mju_quat2Mat(R, mjw.d->qpos + mjw.qBody + 3);
  // Body axes in world coordinates are R's columns: x forward, y left (the axle), z up.
  // The heading comes from the axle, which stays level however far the body
  // pitches (the forward axis would flip over at 90 degrees).
  heading = atan2(-R[1], R[4]);
  double fx = cos(heading), fy = sin(heading);
  pitch = atan2(R[2] * fx + R[5] * fy, R[8]);
}

// Puts the body at the given pitch about its axle, keeping position and heading.
static void mjSetPitch(double pitch) {
  double p, heading;
  mjtNum R[9];
  mjPose(p, heading, R);
  mjtNum qYaw[4], qPitch[4], q[4], z[3] = { 0, 0, 1 }, y[3] = { 0, 1, 0 };
  mju_axisAngle2Quat(qYaw, z, heading);
  mju_axisAngle2Quat(qPitch, y, pitch);
  mju_mulQuat(q, qYaw, qPitch);
  mju_copy(mjw.d->qpos + mjw.qBody + 3, q, 4);
  mjw.d->qpos[mjw.qBody + 2] = mjw.m->geom_size[mjw.wheelGeom * 3];   // axle at wheel height (e.g. picked up after a fall)
}

static void mujocoBuild(Plant &p, double dt) {
  if (mjw.d) mj_deleteData(mjw.d);
  if (mjw.m) mj_deleteModel(mjw.m);
  double r = p.wheelRadius, l = p.comHeight, mb = p.bodyMass, mw = p.wheelMass / 2;
  double ic = mb * l * (p.bodyLength - l);   // pitch inertia about the centre of mass
  double iw = 0.5 * mw * r * r, height = std::max(2 * l, 0.1);
  char xml[4096];
  snprintf(xml, sizeof xml, R"(
<mujoco>
  <option timestep="%g" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default><geom friction="1 0.005 0.0001" condim="3" solref="0.004 1"/></default>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 1"/>
    <body name="body" pos="0 0 %g">
      <freejoint/>
      <inertial pos="0 0 %g" mass="%g" diaginertia="%g %g %g"/>
      <geom type="box" size="0.04 0.08 %g" pos="0 0 %g" mass="0"/>
      <body name="wl" pos="0 %g 0">
        <joint name="wl" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>
        <geom type="sphere" size="%g" mass="0"/>
      </body>
      <body name="wr" pos="0 %g 0">
        <joint name="wr" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>
        <geom type="sphere" size="%g" mass="0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position joint="wl" kp="2000" kv="4"/>
    <position joint="wr" kp="2000" kv="4"/>
  </actuator>
</mujoco>)",
           dt, r, l, mb, ic + mb * 0.08 * 0.08 / 3, ic, mb * (0.08 * 0.08 + 0.16 * 0.16) / 12, height / 2, height / 2,
           MJ_TRACK / 2, mw, iw / 2, iw, iw / 2, r,
           -MJ_TRACK / 2, mw, iw / 2, iw, iw / 2, r);
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "bailey.xml", xml, strlen(xml));
  char err[512] = "";
  mjw.m = mj_loadXML("bailey.xml", &vfs, err, sizeof err);
  mj_deleteVFS(&vfs);
  if (!mjw.m) { fprintf(stderr, "MuJoCo: %s\n", err); exit(2); }
  mjw.d = mj_makeData(mjw.m);
  int jb = 0;                     // the free joint is the model's first joint
  mjw.qBody = mjw.m->jnt_qposadr[jb];
  mjw.vBody = mjw.m->jnt_dofadr[jb];
  int jl = mj_name2id(mjw.m, mjOBJ_JOINT, "wl"), jr = mj_name2id(mjw.m, mjOBJ_JOINT, "wr");
  mjw.qWl = mjw.m->jnt_qposadr[jl];
  mjw.qWr = mjw.m->jnt_qposadr[jr];
  mjw.vWl = mjw.m->jnt_dofadr[jl];
  mjw.vWr = mjw.m->jnt_dofadr[jr];
  mjw.body = mj_name2id(mjw.m, mjOBJ_BODY, "body");
  mjw.wheelGeom = mjw.m->body_geomadr[mj_name2id(mjw.m, mjOBJ_BODY, "wl")];
  mjSetPitch(p.theta);
  mj_forward(mjw.m, mjw.d);
  mjw.phiL = mjw.phiR = 0;
  mjw.theta = p.theta;
  mjw.omega = p.omega;
  mjw.vl = p.vl;
  mjw.vr = p.vr;
  mjw.builtFor = p.id;
}

// One step: wheel speeds relative to the body in rad/s (from the steppers).
static void mujocoStep(Plant &p, double dt, double wl, double wr) {
  if (mjw.builtFor != p.id) mujocoBuild(p, dt);
  mjData *d = mjw.d;
  // Changes the scenario made to the robot since the last step.
  if (p.theta != mjw.theta) mjSetPitch(p.theta);
  if (p.omega != mjw.omega) d->qvel[mjw.vBody + 4] = p.omega;       // pitch rate, body frame
  if (p.vl != mjw.vl || p.vr != mjw.vr) {                           // picked up: everything stops
    for (int i = 0; i < 3; i++) d->qvel[mjw.vBody + i] = 0;
    d->qvel[mjw.vWl] = d->qvel[mjw.vWr] = 0;
  }
  // Steppers: stiff servos on the commanded angles.
  mjw.phiL += wl * dt;
  mjw.phiR += wr * dt;
  d->ctrl[0] = mjw.phiL;
  d->ctrl[1] = mjw.phiR;
  // A disturbance (rad/s^2 on the pitch) as a torque about the axle.
  mjtNum R[9];
  double pitch, heading;
  mjPose(pitch, heading, R);
  double ieff = p.bodyJ() + p.wheelM() + 2 * p.bodyMass * p.wheelRadius * p.comHeight;
  for (int i = 0; i < 3; i++) d->xfrc_applied[6 * mjw.body + 3 + i] = R[3 * i + 1] * p.disturbance * ieff;

  if (p.held) {
    // Someone holds the body still: only the wheels move.
    mjtNum pose[7];
    mju_copy(pose, d->qpos + mjw.qBody, 7);
    mj_step(mjw.m, d);
    mju_copy(d->qpos + mjw.qBody, pose, 7);
    for (int i = 0; i < 6; i++) d->qvel[mjw.vBody + i] = 0;
    mj_forward(mjw.m, d);
  } else {
    mj_step(mjw.m, d);
  }

  mjPose(pitch, heading, R);
  double omega = d->qvel[mjw.vBody + 4];
  p.alpha = (omega - p.omega) / dt;
  p.theta = pitch;
  p.omega = omega;
  // Ground speed of each wheel: the body's forward speed plus the turning.
  double vx = d->qvel[mjw.vBody], vy = d->qvel[mjw.vBody + 1];
  double forward = vx * cos(heading) + vy * sin(heading);
  double yawRate = R[6] * d->qvel[mjw.vBody + 3] + R[7] * d->qvel[mjw.vBody + 4] + R[8] * d->qvel[mjw.vBody + 5];
  if (p.held) {
    p.vl = p.ul;
    p.vr = p.ur;
  } else {
    p.vl = forward - yawRate * MJ_TRACK / 2;
    p.vr = forward + yawRate * MJ_TRACK / 2;
  }
  mjw.theta = p.theta;
  mjw.omega = p.omega;
  mjw.vl = p.vl;
  mjw.vr = p.vr;
}
