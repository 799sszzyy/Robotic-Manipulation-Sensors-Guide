# Sensors for Robotic Manipulation: Implementation Guide

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This repository contains the official implementation code and configuration guides for the survey paper: **"Sensors for Robotic Manipulation: Principles, Classification, and Applications—A Comprehensive Survey"**.

## 📖 About
While the paper provides a comprehensive theoretical framework and hardware selection guide, this repository serves as the practical **Appendix** for researchers and engineers. It includes minimal, ready-to-use boilerplate code for integrating multi-modal sensors (Vision, Force/Torque, Tactile, IMU) into a robotic manipulation system.

## 🗂️ Repository Structure

*   `/drivers`: Standalone interface code for common commercial sensors (Intel RealSense, ATI F/T, XELA Tactile).
*   `/ros2_integration`: Example ROS2 launch files, custom message definitions, and a basic multi-modal synchronization node using `message_filters`.
*   `/algorithms`: Implementations of core integration math (Hand-eye calibration, EKF fusion).

## 📝 Citation
If you find this code or our survey paper useful in your research, please consider citing:

```bibtex
@article{shen2025sensors,
  title={Sensors for Robotic Manipulation: Principles, Classification, and Applications—A Comprehensive Survey},
  author={Shen, Nick and Wang, Brian},
  journal={arXiv preprint},
  year={2025}
}
