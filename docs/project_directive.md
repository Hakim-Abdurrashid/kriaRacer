# Real-Time FPGA-Accelerated Autonomous Racing

An F1TENTH autonomous racing platform utilizing hardware-accelerated deep learning and sensor fusion on an AMD Kria System-on-Module (SOM).

## Main Directive

Deploy a quantized **YOLO** computer vision pipeline on an **AMD Kria SOM** using the **Vitis AI** toolchain. The system offloads neural network inference to a Deep Learning Processing Unit (DPU) synthesized in the **Programmable Logic (PL)**, while the **Processing System (PS)** handles the operating system, ROS 2 control stack, and real-time sensor processing (Intel RealSense T265 & LiDAR) for high-speed autonomous navigation in a F1Tenth track-bounded environment.
