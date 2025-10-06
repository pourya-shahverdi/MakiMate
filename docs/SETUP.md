# 🧠 MakiMate Setup Guide

Welcome to the **MakiMate** project!  
This guide walks you through setting up a **cross-platform ROS 2 development environment** using Docker.  
By following these steps, you’ll be able to develop, build, and test MakiMate’s robot software from any machine (Windows, macOS, or Linux) — without worrying about hardware or OS differences.

## 🚀 Overview

MakiMate uses:
- **ROS 2 Jazzy** as the robotics middleware  
- **Docker** for isolated, reproducible builds  
- **GitHub** for version control and collaboration  

With this setup, everyone on the team can:
- Run the same ROS environment  
- Develop and test locally  
- Push code that builds identically on every system  

## 🧩 1. Prerequisites

Before starting, install the following:

| Tool | Description | Link |
|------|--------------|------|
| **Docker Desktop** | Container engine for your OS | [https://www.docker.com/get-started/](https://www.docker.com/get-started/) |
| **Git** | Version control for cloning and pushing repositories | [https://git-scm.com/downloads](https://git-scm.com/downloads) |
| **VS Code (optional)** | Recommended editor with Docker and ROS plugins | [https://code.visualstudio.com/](https://code.visualstudio.com/) |

Once Docker is installed, make sure it’s running and that **Buildx** is enabled.  
You can test with:

```bash
docker buildx version
```

If you see version info printed, you’re good to go!

## 🧠 2. Clone the Repository

Clone the MakiMate repository to your local machine:

```bash
git clone https://github.com/pourya-shahverdi/MakiMate.git
cd MakiMate
```

## ⚙️ 3. Build the Docker Image

We use a multi-architecture build so that our image works both on desktop (x86_64) and Raspberry Pi 5 (ARM64).

To build and push to GitHub Container Registry (GHCR):

```bash
docker buildx build . \
  -f docker/robot/Dockerfile \
  --platform linux/amd64,linux/arm64 \
  -t ghcr.io/pourya-shahverdi/makimate:latest \
  -t ghcr.io/pourya-shahverdi/makimate:v0.1.0 \
  --push
```

If you just want to test locally (faster build, no push):

```bash
docker buildx build . \
  -f docker/robot/Dockerfile \
  --platform linux/amd64 \
  -t makimate:dev \
  --load
```

## 🧩 4. Run the Container

Once built, you can run the container directly to test that ROS 2 is working:

```bash
docker run --rm -it makimate:dev
```

You should see logs like:

```
[INFO] [hello_node]: Hello from MakiMate!
```

If you want to explore inside the container:

```bash
docker run -it --entrypoint bash makimate:dev
```

Then run ROS manually inside:

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
ros2 launch makimate_bringup bringup.launch.py
```

## 🧰 5. Repository Structure

```
MakiMate/
<<<<<<< HEAD
├── src/                  # Source packages (e.g. makimate_bringup)
│   └── makimate_bringup/
│       ├── launch/
│       ├── makimate_bringup/
│       ├── resource/
│       ├── setup.py
│       ├── setup.cfg
│       └── package.xml
│
├── docker/
│   └── robot/
│       └── Dockerfile    # Main build recipe
│
├── hw/                   # Hardware-related code
├── interfaces/           # ROS interfaces (msg, srv, etc.)
├── docs/
│   └── SETUP.md          # This file
│
├── README.md             # Project overview
└── LICENSE
=======
├── LICENSE                     # Project license file
├── README.md                   # Project overview and introduction
├── deploy                      # Deployment-related scripts or configurations
├── docker/                     # Docker-related files for containerized builds
│   ├── base/                   # Base Docker image configurations
│   │   └── Dockerfile          # Dockerfile for the base image
│   └── robot/                  # Robot-specific Docker configurations
│       ├── Dockerfile          # Main Dockerfile for the robot environment
│       └── entrypoint.sh       # Entrypoint script for the robot container
├── docs/                       # Documentation files
│   └── SETUP.md                # Setup guide for the project
├── hw/                         # Hardware-related code
├── interfaces/                 # ROS 2 interfaces (msg, srv, etc.)
└── src/                        # Source code for ROS 2 packages
    └── makimate_bringup/       # Main ROS 2 package for MakiMate
        ├── launch/             # Launch files for ROS 2 nodes
        │   └── bringup.launch.py  # Main launch file for starting nodes
        ├── makimate_bringup/   # Python package directory
        │   ├── __init__.py     # Python package initialization
        │   └── hello.py        # Example ROS 2 node script
        ├── package.xml         # ROS 2 package manifest
        ├── resource/           # Resource files for the package
        │   └── makimate_bringup  # Resource directory for makimate_bringup
        ├── setup.cfg           # Configuration for Python package
        └── setup.py            # Build script for the ROS 2 package
>>>>>>> origin/main
```

## 🪄 6. Common Commands

**Rebuild the Docker image after making code changes**

```bash
docker buildx build . -f docker/robot/Dockerfile -t makimate:dev --load
```

**Run your package manually in the container**

```bash
docker run -it --entrypoint bash makimate:dev
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
ros2 run makimate_bringup hello
```

**Push code updates to GitHub**

```bash
git add .
git commit -m "Update MakiMate bringup"
git push
```

## 💡 7. Troubleshooting

### “Package not found” error

If ROS can’t find your package when running `ros2 launch`, ensure:
- You built the workspace (`colcon build --merge-install`)
- You sourced both setups:

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
```

### Slow Docker builds

Use `--load` instead of `--push` when testing locally.  
`--push` performs a full multi-arch build and upload, which can take several minutes.

## 🧠 8. Key Concept: Cross-Platform Development

This entire setup ensures:
- Everyone can develop on any OS
- The same image runs on desktop and Raspberry Pi
- No one needs to manually install ROS or dependencies

When you push code to GitHub, it’s automatically shareable and portable — our build system ensures consistency.

## 🫶 9. Need Help?

<<<<<<< HEAD
If something doesn’t work or seems confusing, open a thread in the #dev-setup channel on Discord or tag @pourya.  
We’ll help you troubleshoot!
=======
If something doesn’t work or seems confusing, reach out to Pourya on Discord @pourya9698 .
We’ll help you troubleshoot!
>>>>>>> origin/main
