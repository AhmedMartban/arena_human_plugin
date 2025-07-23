# arena_human_plugin

This repository provides the **`arena_human_plugin`** - a **generic Gazebo Garden plugin for controlling human pedestrians** in the **Arena-Rosnav navigation framework**.

The plugin serves as a **universal interface** between human simulation systems and Gazebo, enabling standardized pedestrian control across different human behavior simulators.

---

## 🎯 Purpose & Vision

The **Arena Human Plugin** is designed to be a **generic foundation** for all human simulators in Arena-Rosnav. It provides:

- **Standardized pedestrian control** via `arena_people_msgs`
- **ECM-based entity management** for Gazebo actors
- **Simulator-agnostic interface** for future human behavior systems
- **Modular architecture** separating generic control from simulator-specific logic

---

## 🔗 Related Projects & Integration

### Current Integration
- **HuNavSim** (pedestrian simulation): [https://github.com/robotics-upo/hunav_sim](https://github.com/robotics-upo/hunav_sim)
- **HuNav Gazebo Plugin**: [https://github.com/Arena-Rosnav/hunav_gz_plugin](https://github.com/Arena-Rosnav/hunav_gz_plugin)
- **Arena-Rosnav**: [https://github.com/Arena-Rosnav](https://github.com/Arena-Rosnav)

### Future Extensions
Additional human simulators will be integrated through this standardized interface, including:
- **Pedestrian behavior models**
- **Crowd simulation systems** 
- **Social navigation frameworks**

---

## 📡 Communication Interface

### Input Topics
- **`/arena_peds`** (`arena_people_msgs/Pedestrians`) - Standardized pedestrian state and control commands

### Message Format
The plugin uses the **Arena People Messages** (`arena_people_msgs`) package:
- **`Pedestrian.msg`** - Individual pedestrian state (position, orientation, velocity, animation)
- **`Pedestrians.msg`** - Collection of all pedestrians with header information

---

## 🏗️ Architecture

```
┌─────────────────────┐    arena_people_msgs    ┌──────────────────────┐
│   Human Simulator   │ ──────────────────────> │  Arena Human Plugin  │
│   (e.g. HuNavSim)   │    /arena_peds          │   (Generic Control)  │
└─────────────────────┘                         └──────────────────────┘
                                                            │
                                                            ▼
                                                 ┌──────────────────────┐
                                                 │   Gazebo ECM/Actors  │
                                                 │  (Visual Simulation) │
                                                 └──────────────────────┘
```

---

## 🧠 Plugin Features

### Core Functionality
- **Actor Management**: Spawn, update, and control Gazebo actors
- **Position Control**: Set actor positions and orientations via ECM
- **Animation Integration**: Coordinate actor animations with movement
- **Transform Broadcasting**: Publish TF transforms for each pedestrian

### Technical Capabilities
- **EntityComponentManager** integration for efficient Gazebo control
- **Real-time updates** via subscription to `arena_peds` topic
- **Thread-safe** actor state management
- **Configurable** animation states and behaviors

---

## 🚀 Getting Started

### Prerequisites
- **ROS 2 Humble**
- **Gazebo Garden** (`gz-sim8`)
- **arena_people_msgs** package



## 🔄 Integration with Human Simulators

### Current: HuNavSim
HuNavSim integration works through:
1. **HunavManager** computes pedestrian behaviors
2. **Arena People Messages** published to `/arena_peds`
3. **Arena Human Plugin** applies commands to Gazebo actors
4. **HuNav Gazebo Plugin** handles simulator-specific features (obstacles, etc.)

### Future: Additional Simulators
New human simulators can integrate by:
1. Publishing to `/arena_peds` topic using `arena_people_msgs`
2. Following the standardized message format
3. Utilizing this plugin for Gazebo visualization

---

## 📜 License

MIT License – see [LICENSE](./LICENSE) for details.

---

