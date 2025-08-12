# AMR ROS 2 Basics – Project 1

A single-file README containing **everything** you need to teach the first steps of Autonomous Mobile Robot (AMR) development with ROS 2 Humble on Ubuntu 22.04: installation, workspace layout, node code, DDS + QoS theory, ASCII visuals, and Git workflow.

***

## Table of Contents

1. Project Goals
2. Repository Layout
3. Prerequisites
4. Step-by-Step Installation Guide
5. Building the Workspace
6. Running Talker \& Listener Nodes
7. ROS 2 Architecture
8. DDS \& QoS Deep-Dive
9. Visual Diagrams (ASCII)
10. Git Commands Cheat-Sheet
11. Further Reading

***

## 1  Project Goals

* Install and verify ROS 2 Humble on **Ubuntu 22.04**.
* Create a Python publisher (talker) and subscriber (listener).
* Understand how **DDS** middleware and **QoS** policies make communication reliable and real-time.
* Push the finished workspace to GitHub.

***

## 2  Repository Layout

```
amr-ros2-basics/
├── amr_basics/
│   ├── __init__.py
│   ├── talker.py
│   └── listener.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md   ← you are here
```


***

## 3  Prerequisites

* Ubuntu 22.04 LTS (Jammy).
* Internet connection and sudo rights.
* Git \& GitHub account (PAT or SSH key).
* Familiarity with terminal basics.

***

## 4  Step-by-Step Installation Guide

> Commands summarised from the official ROS 2 Humble installation docs.[^1][^2]

```bash
# 1. System update & locale
sudo apt update && sudo apt upgrade -y
sudo apt install locales curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 2. Add ROS 2 repository & key
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list

# 3. Install ROS 2 Humble Desktop + dev tools
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools

# 4. Source the environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

# 5. Verify
ros2 --version     # should print something like: ros2 0.18.x
```


***

## 5  Building the Workspace

```bash
# Create a workspace
mkdir -p ~/ros2basic_ws/src
cd ~/ros2basic_ws/src

# Clone or copy this repository here
git clone https://github.com/Dvat162518/amr-ros2-basics.git
cd amr-ros2-basics

# Build
cd ~/ros2basic_ws
colcon build --packages-select amr_basics
source install/setup.bash
```


***

## 6  Running Talker \& Listener Nodes

Open two terminals (both `source install/setup.bash`):

```bash
# Terminal 1
ros2 run amr_basics talker

# Terminal 2
ros2 run amr_basics listener
```

Useful introspection:

```bash
ros2 node list
ros2 topic list
ros2 topic info /amr_topic --verbose
```


***

## 7  ROS 2 Architecture (High-Level)

| Layer | Responsibility |
| :-- | :-- |
| Application (talker / listener scripts) | Your business logic |
| **rclpy** | Python client library API |
| **RMW** (ROS Middleware) | Abstraction layer |
| **DDS implementation** (Cyclone DDS, Fast DDS, etc.) | Discovery, serialization, transport |
| RTPS over UDP/TCP | Network |

Key points:

* **No master** – DDS discovery is peer-to-peer.
* **Nodes** are independent processes; failures do not crash the graph.
* **Topics** carry typed messages; publishers \& subscribers match by **name + compatible QoS**.

***

## 8  DDS \& QoS Deep-Dive

ROS 2 builds on DDS because it provides:[^3]

* **Scalable discovery** with no central broker.
* **Real-time options** (bounded latency, deterministic scheduling).
* **QoS policies** to tune reliability, durability, and resource usage.


### Essential QoS Policies

| Policy | Options | Typical AMR usage |
| :-- | :-- | :-- |
| **Reliability** | BEST_EFFORT, RELIABLE | Sensors → BEST_EFFORT; Commands → RELIABLE |
| **Durability** | VOLATILE, TRANSIENT_LOCAL | Static parameters → TRANSIENT_LOCAL |
| **History** \& **Depth** | KEEP_LAST *N*, KEEP_ALL | KEEP_LAST 10 is ROS default |
| **Deadline / Lifespan** | time-bound | Monitor missed ticks |

**Compatibility rule:** a subscription only receives data if every policy it sets is **no stricter** than the publisher’s. Use:

```bash
ros2 topic info /amr_topic --verbose
```

to inspect policy matching.

## How DDS and QoS Optimize Node Communication
### DDS Architecture in ROS2
#### 1. Discovery Process

Nodes automatically discover each other on the network

No central master required (distributed system)

Uses DDSI-RTPS protocol for real-time communication

Periodic advertisements maintain connectivity

#### 2. Communication Layers

```
ROS2 Application Layer
    ↓
ROS2 Client Library (rclpy/rclcpp)
    ↓
ROS Middleware (RMW)
    ↓
DDS Implementation (Fast-DDS, Cyclone DDS, etc.)
    ↓
Network Transport (UDP/TCP)
```
### QoS Policies Explained
#### 1. Reliability

BEST_EFFORT: Like UDP - fast but may lose messages

RELIABLE: Like TCP - guarantees delivery

#### 2. Durability

VOLATILE: Only for active subscribers

TRANSIENT_LOCAL: Keeps last message for late-joining subscribers

#### 3. History

KEEP_LAST: Maintains only the most recent N messages

KEEP_ALL: Maintains all messages (memory permitting)

#### 4. Deadline

Sets maximum time between message publications

Triggers callback if deadline is missed
```
Visual Communication Flow
text
┌─────────────────┐                    ┌─────────────────┐
│   Talker Node   │                    │  Listener Node  │
│                 │                    │                 │
│  ┌──────────┐   │    DDS Network     │   ┌──────────┐  │
│  │Publisher │   │◄──────────────────►│   │Subscriber│  │
│  │QoS: REL  │   │                    │   │QoS: REL  │  │
│  └──────────┘   │                    │   └──────────┘  │
│                 │                    │                 │
│  Message:       │     ┌─────────┐    │                 │
│  "Robot OK"     │────►│DDS Topic│────┤                 │
│                 │     │"amr_topic"   │                 │
└─────────────────┘     └─────────┘    └─────────────────┘

QoS Matching Process:
1. Publisher advertises topic with QoS profile
2. Subscriber joins with compatible QoS profile  
3. DDS validates QoS compatibility
4. If compatible → Connection established
5. If incompatible → Connection rejected
```

***

## 9  Visual Diagrams (ASCII)

### 9.1  Stack Overview

```
App Nodes (talker / listener)
        │
      rclpy
        │
       RMW
        │
   DDS (Fast DDS / Cyclone DDS …)
        │
     RTPS UDP
```


### 9.2  Message Flow

```
┌────────────┐ publish() ───────► ┌────────────┐
│ Talker Node│                   │Listener Node│
│Publisher   │                   │ Subscriber  │
└────┬───────┘                   └────┬────────┘
     │ DDS DataWriter                 │ DDS DataReader
     └───────────►  DDS / RTPS  ◄─────┘
```


### 9.3  QoS Landscape

```
Reliability: BEST_EFFORT ──────────── RELIABLE
Durability : VOLATILE ──────────────── TRANSIENT_LOCAL
History    : KEEP_LAST(depth=N) ────── KEEP_ALL
```

```
ROS2 Node Communication Architecture
text
                    ROS2 NETWORK (Domain ID: 0)
    ┌─────────────────────────────────────────────────────────────┐
    │                                                             │
    │  ┌─────────────────┐         ┌─────────────────┐           │
    │  │   Talker Node   │         │  Listener Node  │           │
    │  │   ┌─────────┐   │         │   ┌─────────┐   │           │
    │  │   │Publisher│   │         │   │Subscribe│   │           │
    │  │   │         │   │         │   │         │   │           │
    │  │   │QoS:     │   │         │   │QoS:     │   │           │
    │  │   │RELIABLE │   │         │   │RELIABLE │   │           │
    │  │   │TRANSIENT│   │         │   │TRANSIENT│   │           │
    │  │   └─────────┘   │         │   └─────────┘   │           │
    │  └─────────────────┘         └─────────────────┘           │
    │           │                           ▲                    │
    │           │                           │                    │
    │           ▼                           │                    │
    │  ┌─────────────────────────────────────────────────────┐   │
    │  │              DDS MIDDLEWARE                         │   │
    │  │  ┌─────────────────────────────────────────────┐    │   │
    │  │  │            Topic: "amr_topic"               │    │   │
    │  │  │          Message Type: String               │    │   │
    │  │  │                                             │    │   │
    │  │  │  QoS Profile Matching:                      │    │   │
    │  │  │  ✓ Reliability: RELIABLE ↔ RELIABLE         │    │   │
    │  │  │  ✓ Durability: TRANSIENT ↔ TRANSIENT        │    │   │
    │  │  │  ✓ History: KEEP_LAST(10) ↔ KEEP_LAST(10)   │    │   │
    │  │  └─────────────────────────────────────────────┘    │   │
    │  └─────────────────────────────────────────────────────┘   │
    │                           │                                │
    │                           ▼                                │
    │  ┌─────────────────────────────────────────────────────┐   │
    │  │            NETWORK TRANSPORT                        │   │
    │  │              (UDP/TCP)                              │   │
    │  └─────────────────────────────────────────────────────┘   │
    └─────────────────────────────────────────────────────────────┘

    Message Flow Timeline:
    1. Talker creates message: "AMR Message 0: Robot status OK"
    2. DDS serializes message with QoS metadata
    3. DDSI-RTPS protocol transmits over network
    4. DDS receives and validates QoS compatibility  
    5. Message delivered to Listener callback
    6. Listener processes: "Received: AMR Message 0: Robot status OK"
```
## Expected Results
```
Terminal Output - Talker

[INFO] [1723371542.123456789] [amr_talker]: AMR Talker Node Started
[INFO] [1723371543.123456789] [amr_talker]: Publishing: "AMR Message 0: Robot status OK"
[INFO] [1723371544.123456789] [amr_talker]: Publishing: "AMR Message 1: Robot status OK"
[INFO] [1723371545.123456789] [amr_talker]: Publishing: "AMR Message 2: Robot status OK"
Terminal Output - Listener
```
```
[INFO] [1723371543.123456789] [amr_listener]: AMR Listener Node Started
[INFO] [1723371543.234567890] [amr_listener]: Received: "AMR Message 0: Robot status OK"
[INFO] [1723371544.234567890] [amr_listener]: Received: "AMR Message 1: Robot status OK"
[INFO] [1723371545.234567890] [amr_listener]: Received: "AMR Message 2: Robot status OK"
```
