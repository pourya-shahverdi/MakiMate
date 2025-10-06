# 🤖 MakiMate

**MakiMate** is a cross-platform **ROS 2 Jazzy** codebase designed for the **Maki robot**, enabling consistent development across Linux, Windows, and Raspberry Pi through Docker. It provides a unified environment for developing, testing, and deploying robot software — no matter what machine you’re on.

## 🧭 Overview

MakiMate allows every team member to:
- Develop and test **ROS 2** code locally without manual setup
- Build images compatible with **Raspberry Pi** and **desktop systems**
- Push and pull the same environment from **GitHub Container Registry (GHCR)**
- Contribute modular components such as packages, interfaces, and hardware drivers

## 🧩 Project Goals

- 🧠 **Cross-platform** development through Docker
- ⚙️ **Unified ROS 2 environment** for all team members
- 🚀 **Continuous Integration/Deployment (CI/CD)** ready
- 🧍‍♂️ **Collaborative codebase** for modular robot features

## 🏗️ Project Structure

```
MakiMate/
├── src/
│   └── makimate_bringup/
│       ├── launch/
│       ├── makimate_bringup/
│       ├── resource/
│       ├── setup.py
│       ├── setup.cfg
│       └── package.xml
│   ├── interfaces/
│   ├── hw/
│   ├── docker/
│   │   └── robot/
│   │       └── Dockerfile
│   ├── docs/
│   │   └── SETUP.md
│   └── README.md
```

## ⚙️ Getting Started

Follow the detailed step-by-step guide in the **[docs/SETUP.md](docs/SETUP.md)** file. That guide will walk you through:
1. Installing Docker and Git
2. Cloning this repository
3. Building the Docker image
4. Running the MakiMate environment
5. Launching and testing ROS 2 nodes

## 🧠 Quick Start (for experienced users)

If you already have Docker and Buildx configured:

```bash
# Clone the repository
git clone https://github.com/pourya-shahverdi/MakiMate.git
cd MakiMate

# Build the development image
docker buildx build . \
  -f docker/robot/Dockerfile \
  --platform linux/amd64 \
  -t makimate:dev \
  --load

# Run the container
docker run --rm -it makimate:dev
```

If everything is configured correctly, you should see logs like:

```
[INFO] [hello_node]: Hello from MakiMate!
```

### 🧰 Common Commands

**Rebuild the workspace**

```bash
docker buildx build . -f docker/robot/Dockerfile -t makimate:dev --load
```

**Run a single node inside the container**

```bash
docker run -it --entrypoint bash makimate:dev
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
ros2 run makimate_bringup hello
```

**Launch the main bringup**

```bash
docker run --rm -it makimate:dev
```

### 🧩 Development Workflow

1. Create a new branch for your feature or fix:

```bash
git checkout -b feature/new-module
```

2. Modify or add packages in `src/` or `interfaces/`.
3. Test inside Docker to ensure everything builds and launches.
4. Commit and push:

```bash
git add .
git commit -m "Add new ROS node for XYZ"
git push origin feature/new-module
```

5. Create a Pull Request on GitHub.

## 🐳 Why Docker?

Without Docker, setting up ROS 2 with all dependencies can be inconsistent across systems. With Docker:
- Everyone uses the same environment
- The build runs identically on your PC, laptop, or Raspberry Pi
- No dependency conflicts
- No manual ROS installation

## 🔖 Versioning

We follow semantic versioning:
- `latest` → The most recent stable image
- `v0.1.0`, `v0.2.0`, ... → Tagged versions for release builds

Images are stored at:
`ghcr.io/pourya-shahverdi/makimate`

## 🧑‍💻 Contributing

Contributions are welcome! To keep everything clean and functional:
- Work inside Docker so your code matches the deployed environment
- Test your build before pushing
- Keep commits small and meaningful
- Follow ROS 2 Python package structure for new nodes

## 🧩 Maintainers

**Maintainer**: Pourya Shahverdi  
**Lab**: In Real-Life Intelligent Robotics Lab (IRL²), Oakland University  
**Focus**: ROS 2 for social and affective human-robot interaction
