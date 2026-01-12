# Andreas Meyer - Software Projects Portfolio

**MSc Mechanical Engineering Student** | ETH Zurich  
**Focus:** Software Engineering and Robotics

[![Email](https://img.shields.io/badge/Email-andrmeyer@ethz.ch-red)](mailto:andrmeyer@ethz.ch)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-blue)](https://linkedin.com/in/andreas-meyer-67674432b)

---

## 📌 About This Portfolio

This repository catalogs my programming projects and coursework from ETH Zurich. Many projects remain **private** due to **academic integrity policies**, but I'm happy to share code samples with potential employers upon request. Below you'll find detailed descriptions of my work, the technologies used, and the skills developed.

**Available to share:**
- Architecture and approach documentation
- Algorithm explanations and pseudocode
- Performance results and analysis
- Code samples with employer verification

**Contact me at andrmeyer@ethz.ch or andreasm323@gmail.com to request access.**

---

## 🏆 Competitions & Achievements

### 🥇 Dynamic Programming and Optimal Control Competition (2024)

**Result: 1st Place | 2x faster than 2nd place competitor**

**Challenge:** Solve a Stochastic Shortest Path problem as efficiently as possible using only Python, NumPy, and SciPy.

**Problem:** Navigate an agent through a stochastic grid environment with:
- Probabilistic transitions (flow fields, currents)
- Multiple cost factors (time, thruster, collision penalties)
- Moving obstacles (drones)
- Goal: Find optimal policy minimizing expected cost-to-go

**Why I Won:**
- **Algorithmic optimization:** Transformed O(K²L) nested loops into O(KL) vectorized operations
- **Matrix operations:** Transposed probability tensor for cache-friendly access
- **Numerical efficiency:** Minimized memory allocations and data copying

**Technologies:** Python, NumPy, SciPy, dynamic programming, optimization

**Skills Demonstrated:**
- Algorithm design and complexity analysis
- Performance optimization and profiling
- Numerical computing
- Competitive programming

---

## 💻 Coursework

### Stochastics and Machine Learning

Two comprehensive machine learning projects applying regression and deep learning to real-world robotics problems.

---

#### **Project 1: Distance Estimation for Quadruped Robot** 

**Client:** Robotic Systems Lab (RSL), ETH Zurich  
**Robot:** ANYmal quadruped robot  
**Task:** Build lightweight ML distance estimator as backup for advanced perception systems

**Problem Statement:**
Given an RGB image from ANYmal's camera, estimate the distance to the closest obstacle in meters. Dataset: 300×300 images captured in office environment with ground truth from depth sensors.


**Results**
- **Final MAE:** 9.727 cm (Baseline for 6.0: 12 cm)

**Technologies:** Python, scikit-learn, NumPy, Pandas, Matplotlib, PCA, GridSearchCV

**Skills Demonstrated:**
- Machine learning pipeline design
- Feature engineering and dimensionality reduction
- Hyperparameter optimization
- Model selection and ensemble methods
- Cross-validation and avoiding overfitting

---

#### **Project 2: Image Segmentation for Object Detection**

**Task:** Identify ETH-branded coffee mugs in images using binary segmentation

**Problem Statement:**
Given RGB images (378×252 pixels), output binary masks identifying pixels belonging to ETH mugs (vs ETH t-shirts and Zurich mugs).

**Technologies:** Python, PyTorch, CNN, U-Net architecture, data augmentation, GPU training

**Skills Demonstrated:**
- Deep learning for computer vision
- CNN architecture design (encoder-decoder)
- Training pipeline implementation
- Data augmentation strategies
- Transfer learning concepts
- Model ensembling

---

### Planning and Decision Making for Autonomous Robots

**Course Website:** [pdm4ar.github.io/exercises](https://pdm4ar.github.io/exercises/)  
**Instructor:** Prof. Emilio Frazzoli

Comprehensive study of motion planning and decision-making algorithms for autonomous systems.

**Topics Covered:**

**1. Graph-Based Search**
```
Algorithms Implemented:
- Breadth-First Search (BFS)
- Depth-First Search (DFS)  
- Dijkstra's Algorithm
- A* Search with admissible heuristics
- Bidirectional search
```

**2. Sampling-Based Planning**
```
Algorithms Implemented:
- RRT (Rapidly-Exploring Random Tree)
- RRT* (asymptotically optimal variant)
- PRM (Probabilistic Roadmap)
- RRT-Connect (bidirectional RRT)
```
---

**3. Dynamic Programming**
```
Applications:
- Value Iteration for cost-to-go
- Policy Iteration
- Grid-based path planning
```
---

**4. Trajectory Optimization**
```
Methods:
- Minimum snap trajectory generation
- Time-optimal trajectories
- Quadratic programming (QP) for smooth paths
- Constraints: velocity, acceleration, jerk limits
```

---

**5. Markov Decision Processes (MDPs)**
```
Concepts:
- Value Iteration for MDPs
- Policy evaluation and improvement
- Partially Observable MDPs (POMDPs)
- Monte Carlo Tree Search (MCTS)
```

**Technologies:** Python, NumPy, SciPy, Matplotlib, graph algorithms, sampling-based planners

**Skills Demonstrated:**
- Algorithm implementation from papers
- Computational geometry
- Data structures (trees, graphs, k-d trees, priority queues)
- Performance optimization
- Mathematical modeling

---

### Introduction to Mechatronics and Robotics

**Institution:** ETH Zurich - Multi-Scale Robotics Lab (MSRL)

Comprehensive hands-on course covering embedded systems, control theory, and robotic system integration.

**Labs Completed:**

**Lab 0: C Programming Fundamentals**
- Linux/microcontroller C programming
- Makefiles for build automation
- Binary/hexadecimal manipulation
- Bit operations for hardware control

---

**Lab 2: Analog Filtering & Signal Processing**
- Moving average filter (O(1) update)
- Low-pass IIR filters
- Waveform generation (sine, square, triangle)
- Frequency response analysis

---

**Lab 3: Motor Control**
- Servo motor control via PWM
- DC motor H-bridge control
- Gear ratio calculations
- Torque and speed relationships

---

**Lab 4: Data Acquisition & Sensors**
- 12-bit ADC interfacing
- Hall effect sensor for magnetic flux density
- Linear calibration (least squares fitting)
- Sensor signal conditioning

---

**Lab 5: Computer Vision**
- Camera calibration (intrinsic/extrinsic parameters)
- Chessboard corner detection
- Lens distortion correction
- Object detection using HSV color space
- Contour detection and tracking

---

**Lab 6: Closed-Loop Vision Control**
- Ball balancing system using visual feedback
- PID controller implementation
- Real-time control loop (100 Hz)
- Anti-windup for integral term

**Technologies:** C, Python, OpenCV, microcontrollers, PWM, ADC, PID control, computer vision

**Skills Demonstrated:**
- Embedded systems programming
- Real-time control systems
- Signal processing and filtering
- Computer vision and image processing
- Hardware-software integration
- Control theory application

---

### Introduction to ROS (Robot Operating System)

Comprehensive hands-on training in ROS for robotics applications.

**Topics Covered:**

**1. ROS Architecture**
- Nodes, topics, services, actions
- Publisher-subscriber pattern
- Parameter server
- Launch files for system orchestration

---

**2. Sensor Integration**
- LiDAR (laser scan processing)
- RGB-D cameras (depth + color)
- IMU (inertial measurement unit)
- Wheel encoders for odometry

---

**3. Navigation Stack**
- AMCL (Adaptive Monte Carlo Localization)
- move_base for path planning
- Costmap generation (global + local)
- DWA (Dynamic Window Approach) for local planning


---

**4. Simulation**
- Gazebo for 3D robot simulation
- RViz for visualization
- URDF robot descriptions
- World files for environment modeling

---

**Final Project: Autonomous Navigation**
- Mobile robot navigating maze environment
- Obstacle avoidance using LiDAR
- SLAM for map building
- Path planning with A* on occupancy grid
- Trajectory execution with velocity control

**Technologies:** ROS, Python, C++, Gazebo, RViz, SLAM, navigation algorithms

**Skills Demonstrated:**
- Distributed robotics architecture
- Message passing systems
- Real-time sensor processing
- Navigation and localization
- System integration

---

## 🎓 Bachelor Thesis

### Autonomous Mobile Robot Simulation Framework (Grade: 5.75/6.0)

**Institution:** Institute for Dynamic Systems and Control (IDSC), ETH Zurich  
**Advisors:** Dr. Dejan Milojevic, Dr. Andrea Censi  
**Research Group:** Prof. Emilio Frazzoli's team  
**Period:** April - September 2025

**Objective:** Extend the CODEI (Co-Design of Embodied Intelligence) framework for autonomous vehicle simulation in warehouse and delivery robot applications.

**Key Contributions:**

**1. Modular ROS2 Architecture**

**2. Motion Planning Implementation**
- Sampling-based planners: RRT, RRT*, PRM (via OMPL library)
- Configuration space: SE(2) for mobile robots
- Collision checking: Spatio-temporal validation
- Cost function: Time + path smoothness + safety margin

---

**3. Dynamic Environment Simulation**
- Social Force Model for pedestrian dynamics
- Lanelet2 integration for HD semantic maps
- Dynamic obstacle prediction
- Real-time collision avoidance

---

**4. Docker Deployment**

**Technologies:** ROS2 Jazzy, Gazebo, OMPL, Lanelet2, Python, C++, Docker, CODEI framework

**Skills Demonstrated:**
- System architecture design
- Motion planning algorithms
- Simulation framework development
- Software integration
- Academic research and documentation

---

## 🚀 Extracurricular Projects

### ARIS (Academic Space Initiative Switzerland)

Student organization designing and building rockets and satellites.

---

#### **Project: Nicollier - Rocket Recovery System** (2023-2024)

**Role:** Guidance, Navigation & Control (GNC) Engineer

**Challenge:** Autonomous parachute-steered rocket recovery system

**My Contributions:**

**1. Wind Estimation Algorithm (Rust)**

**Why Rust?**
- Memory safety without garbage collection
- Zero-cost abstractions
- Predictable performance for embedded systems
- Growing aerospace industry adoption

---

**Technologies:** Rust, embedded systems, Kalman filtering, control theory, sensor fusion

**Skills Demonstrated:**
- Systems programming in Rust
- Real-time embedded development
- State estimation and sensor fusion
- Control algorithm design
- Aerospace software engineering

---

#### **Project: SAGE - Thermal Control System** (2024-Present)

**Role:** Control Systems Engineer

**Challenge:** Precise thermal control (±0.5°C) for microfluidic experiment in space

**Technologies:** C++, Python, NumPy, Matplotlib, PID control, MPC, thermal modeling

**Skills Demonstrated:**
- Control systems engineering
- Model predictive control
- System modeling and simulation
- Performance optimization
- Scientific computing

---

### ESA Robotics Workshop 2024

**Organization:** European Space Agency (ESA)  
**Location:** ESTEC, Noordwijk, Netherlands  
**Duration:** 1 week intensive workshop  
**Project:** ExoMy planetary rover ([esa-prl.github.io/ExoMy](https://esa-prl.github.io/ExoMy/))

**Challenge:** Implement ML-based terrain classification and autonomous navigation

**Certificate:** ESA Robotics Workshop Certificate 2024

**Technologies:** Python, scikit-learn, ROS, OpenCV, Random Forest, Raspberry Pi

**Skills Demonstrated:**
- Machine learning for robotics
- Computer vision in challenging environments
- ROS system integration
- Real-time embedded systems
- Space robotics domain knowledge

---



## 📄 Why Code is Private

Most repositories remain private due to **ETH Zurich's academic integrity policies**:

**What I CAN share:**
✅ Detailed project descriptions and documentation  
✅ Algorithm explanations and pseudocode  
✅ Architecture diagrams and system design  

**What I CANNOT share publicly:**
❌ Complete coursework solutions (ongoing academic use)  
❌ Competition code (until professor approval)  
❌ Proprietary research code (requires advisor permission)  

**Solution:** I'm happy to share code samples privately with potential employers upon request. Email me at **andrmeyer@ethz.ch** with:
- Company name and position
- Specific projects of interest
- I'll respond within 24 hours with access or code samples

---

## 🎯 What Sets Me Apart

### 1. Proven Competitive Ability
🥇 **1st place in DPOC competition** - Beat 30+ competitors by 2x  
→ Demonstrates algorithmic thinking and optimization skills

### 2. Breadth of Experience
- **Low-level:** Embedded C, microcontrollers, real-time systems
- **High-level:** Machine learning, computer vision, planning algorithms
- **Multi-paradigm:** OOP, functional programming, systems programming

### 3. Multiple Programming Languages
- **Python:** ML pipelines, scientific computing, ROS
- **C++/C:** Performance-critical code, embedded systems
- **Rust:** Modern systems programming, aerospace applications

### 4. Real-World Applications
- **Space:** ARIS rocket recovery, satellite thermal control
- **Robotics:** ANYmal distance estimation, ExoMy autonomy
- **Research:** Bachelor thesis in autonomous systems

### 5. Strong Fundamentals
- Algorithms & data structures
- System architecture and design
- Software engineering best practices
- Performance optimization

---

*Last updated: January 2026*
