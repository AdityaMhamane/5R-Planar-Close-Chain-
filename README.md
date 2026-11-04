# 5R Planar Closed-Chain Robot

Contains MuJoCo model, inverse-kinematics demo for a 5R planar closed-chain robot along with CAD files for making your 5R planar ( in `Build/`). The Python demo computes joint angle (solves IK) for a 5R Planar and moves the actuators following a joint-space trapezoidal velocity profile.

## Previews

| CAD assembly | MuJoCo model |
| --- | --- |
| ![CAD assembly of MuJoCo model](<Images&Videos/5R_CAD.gif>) | ![MuJoCo model](<Images&Videos/5R_XML.gif>) |
**Inverse kinematics and joint-space trajectory**

<p align="center">
  <img src="Images&amp;Videos/5R_IK.gif" alt="Inverse-kinematics simulation" width="560">
</p>

**5R planar build**

| Current CAD | Fabrication |
| --- | --- |
| <img src="Build/Images&amp;GIFs/assembly_new_oct31%20v3.png" alt="Current CAD" width="420"> | <img src="Build/Images&amp;GIFs/FrontViewWhatsApp%20Image%202026-11-02%20at%2012.26.55%20PM.jpeg" alt="Fabrication" width="420"> |

## Simulation Requirements

- Python 3
- MuJoCo's Python package

Install MuJoCo with:

```bash
python -m pip install mujoco
```

## Run

Run these commands from the repository root so the XML file can find its mesh assets:

```bash
# Open the model in the MuJoCo viewer
python scene.py

# Run the inverse-kinematics and trajectory demo
python IK.py
```

`scene.py` opens the model without commanding a motion. `IK.py` targets the end-effector position `(0.0, -0.250)` and moves the two actuators from their initial positions to the calculated joint angles. Its default trajectory uses 1 second of acceleration, 1 second of constant velocity, and 1 second of deceleration.

To try another target, edit `target_x` and `target_y` near the bottom of `IK.py`. The inverse-kinematics function expects coordinates in meters; this model's arms extend in the negative Y direction. Targets outside the calculated reachable workspace are reported and do not produce a joint command.

## Project files

| Path | Description |
| --- | --- |
| `5RPlanar.xml` | MuJoCo robot model, mesh references, actuators, and closed-chain constraint |
| `IK.py` | Inverse kinematics and joint-space trapezoidal trajectory demo |
| `scene.py` | Opens and steps the model in the MuJoCo viewer |
| `meshes/` | STL meshes used by the model |
| `CADs/` | CAD assembly files for 3RRR model used in mujoco simulation |
| `Build/` | Contains CAD assembly files(.step) of 5RRR planar I have builded |
| `Images&Videos/` | Simulation and CAD previews |

## Planned improvements

1. Make the target position and trajectory timing configurable instead of editing `IK.py`.
2. Improve documentation for mechanism dimensions, coordinate frame, etc.
3. ~~Documenting the build for this 5R Planar CAD model.~~. Check `Build/` on how to build your 5R planar 