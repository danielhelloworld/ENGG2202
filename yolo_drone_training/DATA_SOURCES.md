# 数据来源与归因

当前实际使用的来源如下，已统一为RGB输入和单类别drone；鸟、有人飞机等已确认非无人机画面可作为空标签负样本。原作者的多类别标注不代表本项目也训练多类别。

| 来源 | 官方页面 | 使用与隔离 |
|---|---|---|
| Halmstad / DroneDataSet | https://github.com/DroneDetectionThesis/Drone-detection-dataset 、https://github.com/graether1/DroneDataSet | CC0；只使用V_可见光视频，按统一视频ID划分。引用Svanström等，A Dataset for Multi-Sensor Drone Detection，2021 |
| Mendeley Drone Dataset | https://data.mendeley.com/datasets/zcsj2g2m4c/4 | 继承原V2归因（CC BY 4.0）；因帧名和重包身份含混，仅训练及历史回归用途 |
| AerialObjects | 继承V2导出的AerialObjects清单与归因 | 可能与Mendeley重包重叠，按同一来源家族处理；不进入新验证/测试 |
| AOD-4 | https://data.mendeley.com/datasets/cd5z895tr2/1 | CC BY 4.0；Soni、Shah、Joshi，2024；保留图片来源信息，视频按YouTube来源组隔离，统一类别，排除明显非实拍图 |
| DUT-Anti-UAV | https://github.com/wangdongdut/DUT-Anti-UAV | Zhao等，Vision-based Anti-UAV Detection and Tracking，IEEE TITS 2022；官方检测图片及框，仅训练，原官方train/val/test均不当作本项目独立评估划分 |
| Anti-UAV300 | https://github.com/ZhaoJ9014/Anti-UAV | Jiang等，Anti-UAV: A Large-Scale Benchmark for Vision-Based UAV Tracking，IEEE TMM 2022；只取RGB及RGB框，沿用经审计的拍摄组隔离 |
| UAV-CB | https://github.com/hye999/UAV-CB | 只取可见光与对应RGB标签，整来源训练用途 |
| Zagreb RGB | https://zenodo.org/records/15632958 | CC BY 4.0；Ivanković、Stipičević、Stipetić、Vuličević、Bilić、Radišić，Visual drone detection dataset - not augmented，2025，DOI 10.5281/zenodo.15632958。整来源仅验证。原作者已640×640拉伸；包内Roboflow元数据写Private，公开Zenodo记录标明CC BY 4.0，归因按公开发布记录保存 |
| USC MCL RGB | https://mcl.usc.edu/mcl-drone-dataset/ 、https://huggingface.co/datasets/uscmcl/MCL_drone_dataset | University of Southern California，2019；Wang等，Towards Visible and Thermal Drone Monitoring with Convolutional Neural Networks。只取实拍RGB，整来源仅测试；像素框从左上角xywh转换为YOLO归一化坐标 |

上述来源中的图片来源识别、去重、抽帧、类别映射及划分是本项目的整理，不是原作者提供的统一划分。未获授权申请的Drone-vs-Bird与YOLOBirDrone未并入。

## USC使用声明

Copyright (c) 2019, University of Southern California.

Permission is hereby granted, free of charge, to any person obtaining a copy of the database and associated documentation files (the “USC DRONE DATASET”), to deal in the database without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, and/or sell copies of the USC DRONE DATASET, and to permit persons to whom the dataset is furnished to do so, provided that the above copyright notice(s) and this paragraph and the following two paragraphs appear in all copies of the USC DRONE DATASET and in supporting documentation.

IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN CALIFORNIA BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THE USC DRONE DATASET, EVEN IF THE UNIVERSITY OF SOUTHERN CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE UNIVERSITY OF SOUTHERN CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT. THE USC DRONE DATASET PROVIDED HEREUNDER IS ON AN “AS IS” BASIS, AND THE UNIVERSITY OF SOUTHERN CALIFORNIA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
