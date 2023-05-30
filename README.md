# Awesome-Robot-Learning

[![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

This repo contains a curative list of **robot learning** (mainly for manipulation) resources, inspired by [Awesome-Implicit-NeRF-Robotics](https://github.com/zubair-irshad/Awesome-Implicit-NeRF-Robotics). <br> 

**Motivation:** Robot learning, especially robot manipulation skills learning, is receiving more and more attention, but since there are numerous subdivisions of robot learning and a dazzling array of approaches, this repo lists some of the researchers active in the field and the simulation environments used to test their algorithms to save researchers time in searching and focusing on their own algorithms. Related research papers are beyond the scope of this repo.  <br>

Please feel free to send me [pull requests](https://github.com/RayYoh/Awesome-Robot-Learning/blob/master/how-to-RP.md) or [email](mailto:rayyohhust@gmail.com) to add items! <br> 

If you find this repo useful, please consider STARing this list and feel free to share this list with others!

---
## Overview

- [Awesome-Robot-Learning](#awesome-robot-learning)
  - [Overview](#overview)
  - [Related Awesome Lists](#related-awesome-lists)
  - [Laboratories](#laboratories)
  - [Active Researchers](#active-researchers)
  - [Benchmarks](#benchmarks)
    - [MuJoCo-based](#mujoco-based)
    - [PyBullet-based](#pybullet-based)
    - [Issac-based](#issac-based)
    - [Others](#others)
  - [Datasets](#datasets)

---
## Related Awesome Lists
* [Awesome Robotics (Kiloreux)](https://github.com/kiloreux/awesome-robotics)
* [Awesome Robotics (ahundt)](https://github.com/ahundt/awesome-robotics)
* [Awesome Robotic Tooling](https://github.com/protontypes/awesome-robotic-tooling)
* [Awesome Robotics Libraries](https://github.com/jslee02/awesome-robotics-libraries)
* [Awesome Reinforcement Learning](https://github.com/aikorea/awesome-rl/)
* [Awesome Robot Descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions)
* [Awesome NVIDIA Isaac Gym](https://github.com/wangcongrobot/awesome-isaac-gym)
* [Awesome RL Envs](https://github.com/clvrai/awesome-rl-envs)

---
## Laboratories
* [Berkeley Robot Learning Lab](https://rll.berkeley.edu/research.html)
* [Berkeley Robotic AI & Learning Lab](http://rail.eecs.berkeley.edu/index.html)
* [CMU Robotics Institute](https://www.ri.cmu.edu/)
* [Stanford Vision and Learning Lab (SVL)](https://svl.stanford.edu/)
* [UT Austin Robot Perception and Learning Lab](https://rpl.cs.utexas.edu/) [[Github]](https://github.com/UT-Austin-RPL)
* [TU Darmstadt Intelligent Autonomous Systems: Machine Learning for Intelligent Autonomous Robots](https://www.ias.informatik.tu-darmstadt.de/#IAS)
* [ETH Robotic Systems Lab](https://rsl.ethz.ch/)
---
## Active Researchers
* [Pieter Abbeel](https://i3.cs.berkeley.edu/) UC Berkeley
* [Sergey Levine](https://people.eecs.berkeley.edu/~svlevine/) UC Berkeley
* [Jan Peters](https://www.ias.informatik.tu-darmstadt.de/Member/JanPeters) TU Darmstadt
* [Sethu Vijayakumar](https://homepages.inf.ed.ac.uk/svijayak/) University of Edinburgh
* [Huaping Liu](https://sites.google.com/site/thuliuhuaping) Tsinghua University
* [Andy Zeng](https://andyzeng.github.io/) [[Github]](https://github.com/andyzeng) Google Brain
* [Yuke Zhu](https://www.cs.utexas.edu/~yukez/) [[Github]](https://github.com/yukezhu) University of Texas at Austin
* [Cewu Lu](https://www.mvig.org/) Shanghai Jiaotong University
* [Huazhe Xu](http://hxu.rocks/) Tsinghua University
* [Edward Johns](https://www.robot-learning.uk/) Imperial College London
* [Hao Dong](https://zsdonghao.github.io/) Peking University
* [Yunzhu Li](https://yunzhuli.github.io/) [[Github]](https://github.com/yunzhuli) UIUC
* [Yang Gao](http://people.iiis.tsinghua.edu.cn/~gaoyang/yang-gao.weebly.com/index.html) Tsinghua University
* [Xiaolong Wang](https://xiaolonw.github.io/index.html) UC San Diego
* [Nicklas Hansen](https://nicklashansen.github.io/) UC San Diego
* [Wenyu Liang](https://www.liangwenyu.com/welcome) A star
* [Abhinav Valada](https://scholar.google.com/citations?hl=en&user=LcARjz0AAAAJ&view_op=list_works&sortby=pubdate) University of Freiburg
* [Dorsa Sadigh](https://iliad.stanford.edu/people/) Stanford
---
## Benchmarks
### MuJoCo-based
* [dm_control: DeepMind Infrastructure for Physics-Based Simulation](https://github.com/deepmind/dm_control)
* [robosuite: A Modular Simulation Framework and Benchmark for Robot Learning](https://robosuite.ai/)
* [Meta-World: A Benchmark and Evaluation for Multi-Task and Meta Reinforcement Learning](https://meta-world.github.io/)
* [Gymnasium-Robotics](https://github.com/Farama-Foundation/Gymnasium-Robotics)
* [RoboPianist: A Benchmark for High-Dimensional Robot Control](https://kzakka.com/robopianist/)
* [IKEA Furniture Assembly Environment](https://clvrai.github.io/furniture/)
* [Divide-and-Conquer Reinforcement Learning](https://github.com/dibyaghosh/dnc) Catching and Lobbing
* [DoorGym](https://github.com/PSVL/DoorGym)
* [RoboHive](https://github.com/vikashplus/robohive)
* [dm_robotics: Libraries, tools, and tasks created and used for Robotics research at DeepMind](https://github.com/deepmind/dm_robotics)

### PyBullet-based
* [PyBullet Gymperium](https://github.com/benelot/pybullet-gym)
* [panda-gym: Set of robotic environments based on PyBullet physics engine and gymnasium](https://github.com/qgallouedec/panda-gym)
* [MiniTouch benchmark](https://github.com/ServiceNow/MiniTouch) It allows evaluation of models' performance on different manipulation tasks that can leverage cross-modal learning.
* [Calvin: A Benchmark for Language-conditioned Policy Learning for Long-horizon Robot Manipulation Tasks](http://calvin.cs.uni-freiburg.de/) 
* [Pybullet-implementation of the multi-goal robotics environment originally from Open AI Gym](https://github.com/IanYangChina/pybullet_multigoal_gym)
* [Toolkit for Vision-Guided Quadrupedal Locomotion Research ](https://github.com/Mehooz/vision4leg)

### Issac-based
* [Omniverse Isaac Gym Reinforcement Learning Environments for Isaac Sim](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs)
* [Isaac-ManipulaRL](https://github.com/cypypccpy/Isaac-ManipulaRL)
* [Omniverse Isaac Orbit](https://github.com/NVIDIA-Omniverse/Orbit) (Recommended)

### Others
* [RLBench:Robot Learning Benchmark](https://sites.google.com/view/rlbench)
* [Thrower and Goalie Robot Arms](https://github.com/muddasser27/Thrower_Goalie_RobotArms)
* [SoftGym](https://github.com/Xingyu-Lin/softgym)
* [VIMA-Bench: Benchmark for Multimodal Robot Learning](https://github.com/vimalabs/VIMABench)
---
## Datasets
* [D4RL: Datasets for Deep Data-Driven Reinforcement Learning](https://sites.google.com/view/d4rl/home) For offline RL.

