#!/usr/bin/env python3
"""Checks the simulator's robot model (sim.cpp, Plant) against the MuJoCo physics engine.

    python3 -m pip install mujoco numpy
    python3 bailey_FW/test/mujoco_check.py

The robot: a rigid body on a wheel axle, the stepper turning the wheel relative
to the body (its housing is bolted to it, here a stiff position servo), rolling
without slip (an exact constraint: x = r (pitch + wheel angle)). Three open-loop
tests; the models get the wheel acceleration MuJoCo actually produced.

  * "Lagrange": the equations in sim.cpp's Plant::step (what the simulator uses);
  * "cart-pole": pitch'' = (g sin(pitch) - a cos(pitch)) / L, the simulator's
    model before firmware 3.0, which ignores that the wheels turn with the body
    and that the motors push back on it.
"""
import mujoco
import numpy as np

# Bailey: 94 mm wheels, 1.5 kg body with its centre of mass 15 cm above the axle.
r, mb, l, body_length, mw = 0.047, 1.5, 0.15, 0.20, 0.2      # same as Plant's defaults
Ic = mb * l * (body_length - l)                               # body inertia about its centre of mass
Iw = 0.5 * mw * r * r
dt = 0.0001
XML = f"""
<mujoco>
  <option timestep="{dt}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <worldbody>
    <body name="axle" pos="0 0 {r}">
      <joint name="x" type="slide" axis="1 0 0"/>
      <joint name="pitch" type="hinge" axis="0 1 0"/>
      <body name="body"><inertial pos="0 0 {l}" mass="{mb}" diaginertia="{Ic} {Ic} {Ic}"/></body>
      <body name="wheel">
        <joint name="wheel" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="{mw}" diaginertia="{Iw} {Iw} {Iw}"/>
      </body>
    </body>
  </worldbody>
  <tendon><fixed name="roll"><joint joint="x" coef="1"/><joint joint="pitch" coef="{-r}"/><joint joint="wheel" coef="{-r}"/></fixed></tendon>
  <equality><tendon tendon1="roll" solref="0.002 1"/></equality>
  <actuator><position joint="wheel" kp="2000" kv="20"/></actuator>
</mujoco>"""

model = mujoco.MjModel.from_xml_string(XML)
jp, jx = model.joint("pitch").qposadr[0], model.joint("x").qposadr[0]
vw = model.joint("wheel").dofadr[0]


def run_mujoco(pitch0, accel, T):
    d = mujoco.MjData(model)
    d.qpos[jp], d.qpos[jx] = pitch0, r * pitch0
    mujoco.mj_forward(model, d)
    phi = phid = prev = 0.0
    pitch, a_rel = [], []
    for i in range(int(T / dt)):
        phid += accel(i * dt) / r * dt          # commanded wheel angle relative to the body
        phi += phid * dt
        d.ctrl[0] = phi
        mujoco.mj_step(model, d)
        a_rel.append(r * (d.qvel[vw] - prev) / dt)
        prev = d.qvel[vw]
        pitch.append(d.qpos[jp])
    return np.array(pitch), np.array(a_rel)


def run_model(pitch0, a_rel, kind):
    th, w = pitch0, 0.0
    M = mw * r * r + Iw + mb * r * r
    J = mb * l * body_length
    out = []
    for a in a_rel:
        if kind == "cart-pole":
            alpha = (9.81 * np.sin(th) - a * np.cos(th)) / body_length
        else:
            c, s = np.cos(th), np.sin(th)
            alpha = (mb * 9.81 * l * s + mb * r * l * s * w * w - a / r * (M + mb * r * l * c)) / (J + M + 2 * mb * r * l * c)
        w += alpha * dt
        th += w * dt
        out.append(th)
    return np.array(out)


worst = 0
for name, pitch0, accel in [
    ("falling from 2 deg, wheels held", np.radians(2), lambda t: 0.0),
    ("upright, wheels accelerate 1 m/s^2", 0.0, lambda t: 1.0),
    ("3 deg, 2 Hz wobble 1.5 m/s^2", np.radians(3), lambda t: 1.5 * np.sin(2 * np.pi * 2 * t)),
]:
    T = 0.35
    mj, a_rel = run_mujoco(pitch0, accel, T)
    for kind in ("Lagrange", "cart-pole"):
        mo = run_model(pitch0, a_rel, kind)
        rms = np.degrees(np.sqrt(np.mean((mo - mj) ** 2)))
        if kind == "Lagrange":
            worst = max(worst, rms)
        print(f"{name:36s} {kind:9s}: pitch after {T} s MuJoCo {np.degrees(mj[-1]):7.2f}, model {np.degrees(mo[-1]):7.2f} deg"
              f" (RMS difference {rms:.3f} deg)")

M = mw * r * r + Iw + mb * r * r
den = mb * l * body_length + M + 2 * mb * r * l
p1, p3 = mb * 9.81 * l / den, (M + mb * r * l) / r / den
print(f"\nLinearised: pitch'' = (g pitch - kappa a) / L with L = {9.81 / p1:.3f} m, kappa = {p3 * 9.81 / p1:.2f}"
      f" (cart-pole: L = {body_length} m, kappa = 1)")
print("OK: the simulator's model matches MuJoCo" if worst < 0.05 else "MISMATCH")
raise SystemExit(0 if worst < 0.05 else 1)
