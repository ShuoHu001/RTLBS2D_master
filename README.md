# 2D Ray Tracing and Localization Algorithms

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

This repository implements **2D Ray Tracing** (SBR+RT method) and **NLOS Localization Algorithms** with four hybrid positioning modes, powered by Ceres Solver for nonlinear optimization.

## 📌 Features
### 🌐 2D Ray Tracing
- **Specular Beam Reflection + Ray Tracing (SBR+RT)** 
- Multi-bounce reflections simulation
- Obstacle occlusion detection
- Channel parameter generation

### 📍 Hybrid Localization Algorithms
| Mode | Measurements | NLOS Support |
|------|--------------|--------------|
| AOA  | Angle of Arrival | ✔️ |
| TOA  | Time of Arrival  | ✔️ |
| AOA/TOA | Hybrid Angular/Temporal | ✔️ |
| AOA/TDOA | Angular/Time Difference | ✔️ |

## 🛠 Technical Highlights
- **Ceres Solver** integration for robust nonlinear least squares optimization
- Path validation through geometric constraints
- Multi-mode positioning error compensation
- Cross-platform support (Windows/Linux/macOS)

## 📥 Installation
### Dependencies (Recommand windows)
```bash
# Ubuntu
sudo apt-get install libceres-dev libeigen3-dev

# macOS
brew install ceres-solver eigen

# windows
vcpkg install ceres:x64-windows
```

## 🚀 Usage
# ray tracing
```cpp
System rtSystem;
rtSystem.Setup(MODE_RT);
rtSystem.Render();
```

# location based service

```cpp
System lbsSystem;
lbsSystem.Setup(MODE_LBS);
```

@software{RTLBS2025,
  author = {Shuo Hu},
  title = {2D NLOS Localization Framework},
  year = {2025},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/ShuoHu-Scorpio/repo}}
}
