<div align="center">

# Awesome-Robot-Learning
[![Awesome](https://awesome.re/badge.svg)](https://awesome.re) [![license](https://img.shields.io/badge/License-MIT-green.svg?labelColor=gray)](LICENSE) 
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/StrapDown.js/graphs/commit-activity) 
[![PR's Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat)](http://makeapullrequest.com)

</div>

This repo contains a curative list of **robot learning** (mainly for manipulation, which is also considered as a subtask of **Embodied AI**) resources, inspired by [Awesome-Implicit-NeRF-Robotics](https://github.com/zubair-irshad/Awesome-Implicit-NeRF-Robotics). <br> 

**Motivation:** Robot learning, especially robot manipulation skills learning, is receiving more and more attention, but since there are numerous subdivisions of robot learning and a dazzling array of approaches, this repo lists some of the researchers active in the field and the simulation environments used to test their algorithms to save researchers time in searching and focusing on their own algorithms. Related research papers are beyond the scope of this repo.  <br>

Please feel free to send me [pull requests](https://github.com/RayYoh/Awesome-Robot-Learning/blob/master/how-to-RP.md) or [email](mailto:rayyohhust@gmail.com) to add items! <br> 

If you find this repo useful, please consider STARing this list and feel free to share this list with others!

---
## 🔥 News
* Update Awesome-Robotics-3D (2024/08/16).

---

## Overview
- [Awesome-Robot-Learning](#awesome-robot-learning)
  - [🔥 News](#-news)
  - [Overview](#overview)
  - [Related Paper](#related-paper)
    - [Surveys](#surveys)
  - [Related Awesome Lists](#related-awesome-lists)
  - [Laboratories](#laboratories)
  - [Active Researchers](#active-researchers)
  - [Benchmarks](#benchmarks)
    - [Issac-based](#issac-based)
    - [MuJoCo-based](#mujoco-based)
    - [SAPIEN-based](#sapien-based)
    - [PyBullet-based](#pybullet-based)
    - [Others](#others)
  - [Datasets](#datasets)
  - [:books: License](#books-license)

---
## Related Paper
### Surveys
| Paper                    |  Venue | Code/Project |                                  
|---------------------------------------------|:-------------:|:------------:|
|[Aligning Cyber Space with Physical World: A Comprehensive Survey on Embodied AI](https://arxiv.org/pdf/2407.06886)|arXiv 2024|[Code](https://github.com/HCPLab-SYSU/Embodied_AI_Paper_List)|
|[A Survey on Vision-Language-Action Models for Embodied AI](https://arxiv.org/pdf/2405.14093)|arXiv 2024|-|
|[Robot learning in the era of foundation models: A survey](https://arxiv.org/abs/2311.14379)|arXiv 2023|-|
|[What Foundation Models can Bring for Robot Learning in Manipulation : A Survey](https://arxiv.org/pdf/2404.18201)|arXiv 2024|-|
|[Toward general-purpose robots via foundation models: A survey and meta-analysis](https://arxiv.org/abs/2312.08782)|arXiv 2023|[Project](https://robotics-fm-survey.github.io/)|
|[Towards Generalist Robot Learning from Internet Video: A Survey](https://arxiv.org/abs/2404.19664)|arXiv 2024|-|
|[Learning by Watching: A Review of Video-based Learning Approaches for Robot Manipulation](https://arxiv.org/abs/2402.07127)|arXiv 2024|-|


---
## Related Awesome Lists
| Col.1 | Col.2 | Col.3 |
|:---------:|:---------:|:---------:|
| [Awesome Robotics (Kiloreux)](https://github.com/kiloreux/awesome-robotics) | [Awesome Robotics (ahundt)](https://github.com/ahundt/awesome-robotics) | [Awesome Robotic Tooling](https://github.com/protontypes/awesome-robotic-tooling) |
| [Awesome Robotics Libraries](https://github.com/jslee02/awesome-robotics-libraries) | [Awesome Reinforcement Learning](https://github.com/aikorea/awesome-rl/) | [Awesome Robot Descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions) |
| [Awesome NVIDIA Isaac Gym](https://github.com/wangcongrobot/awesome-isaac-gym) | [Awesome RL Envs](https://github.com/clvrai/awesome-rl-envs) | [Awesome-Robotics-3D](https://github.com/zubair-irshad/Awesome-Robotics-3D) |

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
| Name                                                         | Institution            | Name                                                         | Institution                  |
| ------------------------------------------------------------ | ---------------------- | ------------------------------------------------------------ | ---------------------------- |
| [Pieter Abbeel](https://i3.cs.berkeley.edu/)                 | UC Berkeley            | [Sergey Levine](https://people.eecs.berkeley.edu/~svlevine/) | UC Berkeley                  |
| [Jan Peters](https://www.ias.informatik.tu-darmstadt.de/Member/JanPeters) | TU Darmstadt           | [Sethu Vijayakumar](https://homepages.inf.ed.ac.uk/svijayak/) | University of Edinburgh      |
| [Huaping Liu](https://sites.google.com/site/thuliuhuaping)   | Tsinghua University    | [Andy Zeng](https://andyzeng.github.io/) [[Github]](https://github.com/andyzeng) | Google Brain                 |
| [Yuke Zhu](https://www.cs.utexas.edu/~yukez/) [[Github]](https://github.com/yukezhu) | UT-Austin              | [Cewu Lu](https://www.mvig.org/)                             | Shanghai Jiaotong University |
| [Huazhe Xu](http://hxu.rocks/)                               | Tsinghua University    | [Edward Johns](https://www.robot-learning.uk/)               | Imperial College London      |
| [Hao Dong](https://zsdonghao.github.io/)                     | Peking University      | [Yunzhu Li](https://yunzhuli.github.io/) [[Github]](https://github.com/yunzhuli) | UIUC                         |
| [Yang Gao](http://people.iiis.tsinghua.edu.cn/~gaoyang/yang-gao.weebly.com/index.html) | Tsinghua University    | [Xiaolong Wang](https://xiaolonw.github.io/index.html)       | UC San Diego                 |
| [Nicklas Hansen](https://nicklashansen.github.io/)           | UC San Diego           | [Wenyu Liang](https://www.liangwenyu.com/welcome)            | A star                       |
| [Abhinav Valada](https://scholar.google.com/citations?hl=en&user=LcARjz0AAAAJ&view_op=list_works&sortby=pubdate) | University of Freiburg | [Dorsa Sadigh](https://iliad.stanford.edu/people/)           | Stanford                     |
| [Hao Su](https://cseweb.ucsd.edu/~haosu/) | UC San Diego | [He Wang](https://scholar.google.com/citations?user=roCAWkoAAAAJ&hl=en)           | Peking University                     |
| [Siyuan Huang](https://siyuanhuang.com/) | BIGAI | -           | -                     |

---
## Benchmarks
### Issac-based
* [Omniverse Isaac Orbit](https://github.com/NVIDIA-Omniverse/Orbit) Based on Issac Sim.
* [Omniverse Isaac Gym Reinforcement Learning Environments for Isaac Sim](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs)
* [OmniGibson](https://github.com/StanfordVL/OmniGibson) a platform for accelerating Embodied AI research built upon NVIDIA's Omniverse platform.
* [ARNOLD](https://github.com/arnold-benchmark/arnold) A Benchmark for Language-Grounded Task Learning With Continuous States in Realistic 3D Scenes
* [Isaac-ManipulaRL](https://github.com/cypypccpy/Isaac-ManipulaRL)
* [Unitree Go2 Omniverse](https://github.com/abizovnuralem/go2_omniverse)
* [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)

### MuJoCo-based
* [dm_control: DeepMind Infrastructure for Physics-Based Simulation](https://github.com/deepmind/dm_control)
* [dm_robotics: Libraries, tools, and tasks created and used for Robotics research at DeepMind](https://github.com/deepmind/dm_robotics)
* [DMControl Generalization Benchmark 2](https://github.com/aalmuzairee/dmcgb2)
* [robosuite: A Modular Simulation Framework and Benchmark for Robot Learning](https://robosuite.ai/)
* [RoboCasa: Large-Scale Simulation of Everyday Tasks for Generalist Robots](https://github.com/robocasa/robocasa) Based on robosuite.
* [Meta-World: A Benchmark and Evaluation for Multi-Task and Meta Reinforcement Learning](https://meta-world.github.io/)
* [Gymnasium-Robotics](https://github.com/Farama-Foundation/Gymnasium-Robotics)
* [RoboPianist: A Benchmark for High-Dimensional Robot Control](https://kzakka.com/robopianist/)
* [IKEA Furniture Assembly Environment](https://clvrai.github.io/furniture/)
* [Divide-and-Conquer Reinforcement Learning](https://github.com/dibyaghosh/dnc) Catching and Lobbing
* [DoorGym](https://github.com/PSVL/DoorGym)
* [RoboHive](https://github.com/vikashplus/robohive)
* [mujoco_menagerie: High-quality models for MuJoCo](https://github.com/google-deepmind/mujoco_menagerie)

### SAPIEN-based
* [ManiSkill3](https://github.com/haosulab/ManiSkill)
* [SimplerEnv: Simulated Manipulation Policy Evaluation Environments for Real Robot Setups](https://github.com/simpler-env/SimplerEnv) Currently based on ManiSkill2

### PyBullet-based
* [PyBullet Gymperium](https://github.com/benelot/pybullet-gym)
* [panda-gym: Set of robotic environments based on PyBullet physics engine and gymnasium](https://github.com/qgallouedec/panda-gym)
* [MiniTouch benchmark](https://github.com/ServiceNow/MiniTouch) It allows evaluation of models' performance on different manipulation tasks that can leverage cross-modal learning.
* [Calvin: A Benchmark for Language-conditioned Policy Learning for Long-horizon Robot Manipulation Tasks](http://calvin.cs.uni-freiburg.de/) 
* [Pybullet-implementation of the multi-goal robotics environment originally from Open AI Gym](https://github.com/IanYangChina/pybullet_multigoal_gym)
* [Toolkit for Vision-Guided Quadrupedal Locomotion Research ](https://github.com/Mehooz/vision4leg)

### Others
* [Genesis](https://github.com/Genesis-Embodied-AI/Genesis) A generative and simulated physical realm for general-purpose embodied-AI learning.
* [RLBench:Robot Learning Benchmark](https://sites.google.com/view/rlbench)
* [Thrower and Goalie Robot Arms](https://github.com/muddasser27/Thrower_Goalie_RobotArms)
* [SoftGym](https://github.com/Xingyu-Lin/softgym)
* [VIMA-Bench: Benchmark for Multimodal Robot Learning](https://github.com/vimalabs/VIMABench)
* [Colosseum: A Benchmark for Evaluating Generalization for Robotic Manipulation](https://github.com/robot-colosseum/robot-colosseum) Based on RLBench
* [LeRobot: State-of-the-art Machine Learning for real-world robotics](https://github.com/huggingface/lerobot) From HuggingFace.
* [HumanoidBench: Simulated Humanoid Benchmark for Whole-Body Locomotion and Manipulation](https://github.com/carlosferrazza/humanoid-bench)

---
## Datasets
* [Open X-Embodiment: Robotic Learning Datasets and RT-X Models](https://robotics-transformer-x.github.io/)
* [D4RL: Datasets for Deep Data-Driven Reinforcement Learning](https://sites.google.com/view/d4rl/home) For offline RL.
* [RH20T: A Comprehensive Robotic Dataset for Learning Diverse Skills in One-Shot](https://rh20t.github.io/) Currently the largest single robotic dataset for manipulation. It contains over **110K robot episodes**, **110K corresponding human demonstrations**, over **50 million frames** and over **140 tasks**. 

## :books: License
This repository is released under the [MIT license](LICENSE).