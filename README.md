# Venom-Vision

Venom-Vision is a walking, scorpion-inspired robot that combines multi-leg locomotion with on-board AI vision. An ESP32-CAM module performs real-time object detection, while a Nano-class microcontroller coordinates the servos and path-finding logic so the robot can react intelligently to its surroundings.

---

## Project Overview

At a high level, Venom-Vision explores how low-cost hardware can be used to build an autonomous, animal-like robot:

- A **scorpion-style chassis** with multiple legs, claws, and tail segments driven by servo motors.
- An **ESP32-CAM** module that acts as the robot’s “eyes,” running an embedded machine-learning model.
- A **Nano-based main controller** that acts as the “brain,” translating detection results into motion and behaviour.
- A dedicated **data-collection sketch** for capturing camera images to train custom models (e.g., using Edge Impulse).

The goal is to create a small, self-contained platform for experimenting with embedded AI, legged locomotion, and reactive behaviours.

---

## 3D CAD Design

<img width="1373" height="745" alt="image" src="https://github.com/user-attachments/assets/847e699d-88d0-40b3-b726-2faac19ba79f" />

---

## System Architecture

### 1. Locomotion & Control (`NANO_MAIN`)

The **NANO_MAIN** firmware drives the servos that move the legs, claws, and tail. It encodes gaits (walking patterns), turning behaviour, and high-level state machines such as:

- Idle stance vs. active exploration.
- Turning or side-stepping to avoid obstacles.
- Tracking or approaching detected targets.

The Nano receives compact messages from the ESP32-CAM (for example, “object detected on left/right/center”) and converts these into motion commands, giving the robot a sense of direction and purpose.

### 2. Vision & Inference (`ESP32_CAM`)

The **ESP32_CAM** firmware is responsible for:

- Initializing and configuring the camera.
- Capturing image frames from the robot’s point of view.
- Running an on-device ML model (image classification or object detection).
- Sending summarized results to the Nano controller.

Because inference runs directly on the ESP32-CAM, Venom-Vision doesn’t need a tethered computer: perception and decision-making happen entirely on the robot, with the Nano interpreting semantic cues like “target ahead” or “obstacle close.”

### 3. Dataset Collection (`Collect_Images_for_EdgeImpulse`)

The **Collect_Images_for_EdgeImpulse** sketch is used to build the training dataset that powers the vision system. It helps:

- Stream or capture images from the ESP32-CAM.
- Gather varied examples of the environment or specific target objects.
- Prepare data for labeling and training in an ML pipeline such as Edge Impulse.

By iterating on this dataset, you can teach Venom-Vision to recognize new objects or refine its detection performance without changing the overall hardware design.

---

## Behaviour & Capabilities

With all subsystems working together, Venom-Vision can:

- **Walk and steer** using a scorpion-like gait controlled by the Nano.
- **Perceive its environment** through the ESP32-CAM’s camera feed.
- **Detect objects or obstacles** via a trained neural network model.
- **React in real time**, adjusting its path or behaviour based on detection results.

This makes the project a compact testbed for:

- Embedded computer vision on microcontrollers.
- Multi-legged robot kinematics and coordination.
- Closed-loop control where AI perception directly drives physical behaviour.

---

## Project Structure

```text
Venom-Vision/
├── Collect_Images_for_EdgeImpulse/   # Camera data-collection sketch for ML training
├── ESP32_CAM/                        # Vision + inference firmware for the ESP32-CAM module
├── NANO_MAIN/                        # Main controller firmware for locomotion and behaviour
├── LICENSE                           # MIT license
└── README.md                         # Project description


