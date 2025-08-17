#import "../common.typ": *

= ROS2 Foundations
---

#include "subparts/turtlesim.typ"

---

== What is ROS?

The *Robot Operating System* (*ROS*) is an open-source, flexible framework for writing robot software. It is not an operating system in the traditional sense but rather a collection of software libraries, tools, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

*ROS* provides functionalities typically expected from an *OS*, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. 

#goal[Its primary goal is to support code reuse in robotics research and development.]

---

=== Key Characteristics of ROS

==== Modular Architecture

*ROS* is built on a distributed computing model, allowing developers to create modular software packages that can easily communicate with each other. This approach enables:

- Flexible robot software design
- Easy integration of different components
- Reusability of code across different robotic projects

---

==== Communication Infrastructure

At its core, *ROS* provides a robust communication infrastructure that allows different parts of a robotic system to exchange information:

- Publish-subscribe messaging model
- Service-client communication
- Action servers for long-running tasks
- Support for multiple programming languages (primarily C++ and Python)

---

==== Extensive Toolset

*ROS* offers a comprehensive set of tools for various aspects of robotics development:

- Simulation environments (like *Gazebo*)
- Visualization tools (like *RViz*)
- Debugging and logging utilities
- Hardware abstraction layers
- Package management system

---

=== Applications and Usage

*ROS* is widely adopted in:

- Academic research institutions
- Robotics laboratories
- Industrial robotics
- Autonomous vehicle development
- Research and development in various domains (manufacturing, healthcare, aerospace)

---

#warning[Limitations of *ROS1*]

While *ROS1* achieved significant success, the evolving robotics landscape highlighted several limitations in its design, including:

/ Multi-robot systems: Struggled with decentralized communication and discovery in large or dynamic networks.
/ Real-time control: Lacked native support for real-time performance requirements.
/ Production environments: Offered limited security features, making it less suitable for commercial deployments.
/ Resource-constrained platforms: Proved resource-heavy for embedded systems.
/ Network reliability: Depended on a single master node, introducing a single point of failure.


---

#idea[*ROS2* Overview]

*ROS2* was designed from scratch to overcome the limitations of *ROS1*, delivering a more robust, secure, and adaptable platform. It supports a wide range of applications, from research prototypes to industrial and commercial systems.

/ Enhanced Multi-Robot Support: Enables decentralized communication and discovery, addressing *ROS1*'s challenges with dynamic, large-scale robot networks.
/ Real-Time Capabilities: Provides native support for real-time control, making it suitable for time-critical robotic applications.
/ Improved Security: Incorporates robust security features to meet the needs of commercial and industrial deployments.
/ Resource Efficiency: Optimized for resource-constrained platforms, such as embedded systems, unlike the heavier *ROS1*.
/ Network Reliability: Eliminates the single point of failure by removing dependency on a single master node, improving system resilience.


---

#set table(
  stroke: (x, y) => if x == 0 or y == 0 {
    (bottom: 0.7pt + black)
  },
  gutter: 0.2em,
  fill: (x, y) =>
    if x == 0 or y == 0 { yellow.lighten(70%) },
  inset: (right: 1.5em),
)

#align(center)[
#table(
  columns: 4,
[*Distro*], [*Logo*], [*Release*], [*EOL*],
[Kilted Kaiju], [#align(center)[#image("../imgs/logos/kilted-small.png", width:1.5cm)]], [23 mai 2025], [December 2026],
[Jazzy Jalisco], [#align(center)[#image("../imgs/logos/jazzy-small.png", width:1.5cm)]], [23 mai 2024], [May 2029],
[Iron Irwini], [#align(center)[#image("../imgs/logos/iron-small.png", width:1.5cm)]], [23 mai 2023], [November 2024],
[Humble Hawksbill], [#align(center)[#image("../imgs/logos/humble-small.png", width:1.5cm)]], [23 mai 2022], [mai 2027],
[Galactic Geochelone], [#align(center)[#image("../imgs/logos/galactic-small.png", width:1.5cm)]], [23 mai 2021], [9 december 2022],
[Foxy Fitzroy], [#align(center)[#image("../imgs/logos/foxy-small.png", width:1.5cm)]], [5 juin 2020], [20 june 2023],
[Eloquent Elusor], [#align(center)[#image("../imgs/logos/eloquent-small.png", width:1.5cm)]], [22 novembre 2019], [novembre 2020],
[Dashing Diademata], [#align(center)[#image("../imgs/logos/dashing-small.png", width:1.5cm)]], [31 mai 2019], [mai 2021],
[Crystal Clemmys], [#align(center)[#image("../imgs/logos/crystal-small.png", width:1.5cm)]], [14 december 2018], [december 2019],
[Bouncy Bolson], [#align(center)[#image("../imgs/logos/bouncy-small.png", width:1.5cm)]], [2 july 2018], [july 2019],
[Ardent Apalone], [#align(center)[#image("../imgs/logos/ardent-small.png", width:1.5cm)]], [8 december 2017], [december 2018]
)]
---

#notify[*Introducing ROS2 Humble Hawksbill*]

*ROS2* *Humble Hawksbill*, the eighth major release of *ROS2*, was officially launched on May 23, 2022. It marks a key milestone in the *ROS2* ecosystem, enhancing previous versions with new features and improvements.

---

- *Humble Hawksbill* is an LTS release, offering stability and extended maintenance for developers.
- Support includes bug fixes and security patches until May 2027.
- Its long support period suits commercial products, extended research projects, and educational use.
- Primarily targets *Ubuntu 22.04 Jammy Jellyfish* (`amd64` and `arm64` architectures).
- Also compatible with *Windows 10*.


---

*Improvements in Humble*

- ROSBag Enhancements 
- Performance and Stability Gains
- Enhanced Documentation
- Developer Ergonomics
- Gazebo Fortress Integration
- FogROS2
- Foxglove Studio
