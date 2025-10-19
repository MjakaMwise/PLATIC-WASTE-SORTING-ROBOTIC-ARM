# Plastic Sorting Robotic Arm 🤖♻️

## Overview

This project showcases a **robotic arm** capable of **detecting, picking, and sorting plastic waste** based on predefined parameters such as **color, type, or size**. The aim is to demonstrate how robotics and automation can support **sustainable recycling practices** by reducing manual effort and improving sorting efficiency.

## Features

* 🔍 **Plastic detection** using sensors (or optional computer vision).
* ✋ **Robotic arm manipulation** to pick and place objects.
* ♻️ **Automated sorting** into designated bins (e.g., PET, HDPE, LDPE).
* ⚡ **Real-time control system** for efficient operation.
* 🌍 Focus on **environmental sustainability** through smart waste management.

## Tech Stack

* **Hardware**: Robotic arm, servo motors, sensors (IR/Color/Camera).
* **Software**: Arduino / Raspberry Pi for microcontroller logic.
* **Libraries**: OpenCV (if vision-based sorting), control libraries for motor actuation.
* **Languages**: C++ (Arduino), Python (for image processing/AI).

## How It Works

1. **Detection**: The sensor or camera identifies the type of plastic.
2. **Decision**: Control logic classifies the item into the correct category.
3. **Action**: The robotic arm picks the item and places it in the designated bin.
4. **Feedback**: System logs or LEDs confirm successful sorting.

## Applications

* ✅ Recycling centers.
* ✅ Educational projects in robotics & AI.
* ✅ Demonstrating real-world use of automation in environmental sustainability.

## Future Improvements

* 🧠 AI-based classification for higher accuracy.
* 📊 Data collection on sorted materials.
* 🔋 Energy-efficient operation.

---

⚡ *This project is a step toward merging robotics with sustainability, showing how technology can make recycling smarter and more efficient.*


IK,X=10,Y=5,Z=8
```
This moves the end effector to position (10cm, 5cm, 8cm) and automatically calculates the servo angles.

**Move with gripper control:**
```
IK,X=10,Y=5,Z=8,G=45
```
Same as above, but also sets the gripper to 45°.

**Utility commands:**
```
HOME     - Reset all servos to 90°
STATUS   - Show current angles and calculated position
```

## Coordinate System

- **Origin**: Center of the base
- **X-axis**: Forward (positive = away from base)
- **Y-axis**: Left (positive = left side)
- **Z-axis**: Up (positive = higher)
- **Units**: Centimeters

## How It Works

1. **Base angle** (Servo 0): Calculated using `atan2(Y, X)` - rotates to point toward target
2. **Shoulder & Elbow** (Servos 1 & 2): Solved using the law of cosines in the vertical plane
3. The arm uses "elbow down" configuration (more natural and stable)

## Reachable Workspace

- **Maximum reach**: 10cm (5cm + 5cm arms fully extended)
- **Minimum reach**: 0cm (arms folded)
- **Height range**: 0cm to 15cm (5cm base + 10cm arms)

## Example Commands
```
IK,X=8,Y=0,Z=5     - Reach straight forward
IK,X=5,Y=5,Z=10    - Reach up and to the left
IK,X=0,Y=7,Z=5     - Reach directly to the left
HOME               - Center position
STATUS             - Check where the arm currently is
```

Your original direct control still works:
```
0,=120;1,=45;2,=135    - Set specific angles
