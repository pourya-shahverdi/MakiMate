# 🧠 MakiMate Setup Guide

Welcome to **MakiMate**, a cross-platform **ROS 2 Jazzy** codebase for the Maki robot! This guide will help you set up a consistent development environment using **Docker** on **Windows**, **macOS**, or **Linux**, enabling you to develop, test, and deploy robot software without worrying about OS or hardware differences.

---

## 🚀 Overview

MakiMate leverages:
- **ROS 2 Jazzy**: Robotics middleware for nodes, topics, and services.
- **Docker**: Ensures isolated, reproducible build environments.
- **GitHub**: Facilitates version control and team collaboration.
- **VS Code** (optional): Recommended editor with ROS 2 and Docker support.

**Benefits**:
- ✅ Run the same ROS 2 environment across all systems.
- ✅ Develop and test locally with ease.
- ✅ Share code that builds identically for everyone.

---

## 🪟 Setup Instructions

Follow the steps below based on your operating system. Windows users will use WSL 2 (Windows Subsystem for Linux) to run Docker and ROS 2.

### 🪟 A) Windows Setup (One-Time)

For **Windows 10 (21H2+)** or **Windows 11**, complete these steps to prepare your system.

#### 1️⃣ Install WSL 2 + Ubuntu
1. Open **“Turn Windows features on or off”** via Windows Search and enable:
   - ✅ **Windows Subsystem for Linux**
   - ✅ **Virtual Machine Platform**
2. Reboot your computer if prompted.
3. Open **Microsoft Store**, search for **Ubuntu 24.04 LTS**, and install it.
4. Launch Ubuntu from the Start Menu to set up a username and password.
5. In **PowerShell (Admin)**, set WSL 2 as the default:
   ```powershell
   wsl --set-default-version 2
   ```
   💡 This creates a lightweight Linux environment for running Docker and ROS 2.

#### 2️⃣ Install Docker Desktop
1. Download and install **Docker Desktop for Windows**: [https://www.docker.com/get-started/](https://www.docker.com/get-started/).
2. Open Docker Desktop → **Settings** → **Resources** → **WSL Integration**.
3. Enable:
   - ✅ **Enable integration with my default WSL distro**.
   - ✅ Toggle on **Ubuntu-24.04**.
4. Click **Apply & Restart**.

#### 3️⃣ Install VS Code (Optional)
1. Download **VS Code**: [https://code.visualstudio.com/](https://code.visualstudio.com/).
2. Install these extensions:
   - **Dev Containers**
   - **Remote – WSL**
   - **Docker**
   - **ROS** (optional, for ROS 2 development support)
3. Open Ubuntu (Start Menu → Ubuntu), navigate to your project folder, and run `code .` to launch VS Code in WSL (look for “WSL: Ubuntu-24.04” in the bottom-left corner).

---

### 🐧 B) Linux or WSL (Ubuntu) Setup

These steps apply to **Ubuntu 22.04/24.04** (or WSL Ubuntu on Windows). macOS users should skip to the macOS-specific Docker installation below.

#### 4️⃣ Install Basic Packages and Git
1. Open your Ubuntu terminal (or WSL Ubuntu on Windows).
2. Install essential tools:
   ```bash
   sudo apt update
   sudo apt install -y git build-essential curl python3-pip
   ```

#### 5️⃣ (Optional) Set Up GitHub SSH Authentication
1. Generate an SSH key:
   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com" -f ~/.ssh/id_ed25519 -N ""
   ```
2. Display the public key:
   ```bash
   cat ~/.ssh/id_ed25519.pub
   ```
3. Copy the output, go to **GitHub** → **Settings** → **SSH and GPG keys** → **New SSH key**, and paste it.

---

### 🐳 C) Install Docker

#### For Ubuntu (22.04/24.04) or WSL Ubuntu
1. Remove any old Docker versions:
   ```bash
   sudo apt remove docker docker-engine docker.io containerd runc
   ```
2. Install Docker:
   ```bash
   sudo apt update
   sudo apt install -y ca-certificates curl gnupg
   sudo install -m 0755 -d /etc/apt/keyrings
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
   echo \
     "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
     https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
     sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   sudo apt update
   sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   ```
3. Add your user to the Docker group:
   ```bash
   sudo usermod -aG docker $USER
   newgrp docker
   ```
4. Test Docker and Buildx:
   ```bash
   docker run hello-world
   docker buildx version
   ```
   💡 If Buildx is missing, set it up:
   ```bash
   docker buildx create --name mybuilder --use
   docker buildx inspect --bootstrap
   ```
   🔁 Restart your computer or log out/in after adding yourself to the Docker group.

#### For macOS
1. Install **Docker Desktop for macOS**: [https://www.docker.com/get-started/](https://www.docker.com/get-started/).
2. Start Docker Desktop (ensure the 🐳 icon appears in the menu bar).
3. Test Docker:
   ```bash
   docker run hello-world
   ```

---

### 🧠 D) Clone the Repository
1. Clone the MakiMate repository:
   ```bash
   git clone https://github.com/pourya-shahverdi/MakiMate.git
   cd MakiMate
   ```
   Or, if you set up SSH:
   ```bash
   git clone git@github.com:pourya-shahverdi/MakiMate.git
   cd MakiMate
   ```

---

### ⚙️ E) Build the Docker Image
MakiMate uses **Docker Buildx** to create images for both desktop (x86_64) and Raspberry Pi 5 (ARM64).

#### Option 1: Local Build (Recommended for Beginners)
Build a local image for your system (faster):
```bash
docker buildx build . \
  -f docker/robot/Dockerfile \
  --platform linux/amd64 \
  -t makimate:dev \
  --load
```

#### Option 2: Multi-Architecture Build (for Desktop + Raspberry Pi)
Build and push to GitHub Container Registry (GHCR) for both architectures:
```bash
docker login ghcr.io
docker buildx build . \
  -f docker/robot/Dockerfile \
  --platform linux/amd64,linux/arm64 \
  -t ghcr.io/pourya-shahverdi/makimate:latest \
  -t ghcr.io/pourya-shahverdi/makimate:v0.1.0 \
  --push
```
💡 This takes longer due to multi-architecture compilation.

---

### 🧩 F) Run the Container
1. Run the container to verify ROS 2 is working:
   ```bash
   docker run --rm -it makimate:dev
   ```
   **Expected Output**:
   ```
   [INFO] [hello_node]: Hello from MakiMate!
   ```

2. Explore inside the container:
   ```bash
   docker run -it --entrypoint bash makimate:dev
   ```
3. Inside the container, run ROS 2 commands:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source /ws/install/setup.bash
   ros2 launch makimate_bringup bringup.launch.py
   ```
4. Exit the container: `exit` or `Ctrl+D`.

---

## 🏗️ Project Structure

The MakiMate repository is organized as follows:

```
MakiMate/
├── LICENSE                     # Project license file
├── README.md                   # Project overview and introduction
├── deploy/                     # Deployment-related scripts or configurations
├── docker/                     # Docker configuration files
│   ├── base/                   # Base Docker image configurations
│   │   └── Dockerfile          # Dockerfile for the base image
│   └── robot/                  # Robot-specific Docker configurations
│       ├── Dockerfile          # Main Dockerfile for the robot environment
│       └── entrypoint.sh       # Entrypoint script for the robot container
├── docs/                       # Documentation files
│   └── SETUP.md                # This setup guide
├── hw/                         # Hardware-related code
├── interfaces/                 # ROS 2 interfaces (msg, srv, etc.)
└── src/                        # Source code for ROS 2 packages
    └── makimate_bringup/       # Main ROS 2 package
        ├── launch/             # Launch files for ROS 2 nodes
        │   └── bringup.launch.py  # Main launch file
        ├── makimate_bringup/   # Python package directory
        │   ├── __init__.py     # Python package initialization
        │   └── hello.py        # Example ROS 2 node script
        ├── package.xml         # ROS 2 package manifest
        ├── resource/           # Resource files
        │   └── makimate_bringup  # Resource directory
        ├── setup.cfg           # Python package configuration
        └── setup.py            # Build script for the package
```

💡 The `__init__.py` file ensures Python and ROS 2 recognize the `makimate_bringup` package.

---

## 🧰 Common Commands

| Action | Command | Run Where |
|--------|---------|-----------|
| Rebuild Docker image | `docker buildx build . -f docker/robot/Dockerfile -t makimate:dev --load` | Host |
| Run container interactively | `docker run -it --entrypoint bash makimate:dev` | Host |
| Source ROS 2 environment | `source /opt/ros/jazzy/setup.bash` | Inside container |
| Build workspace | `colcon build --merge-install` | Inside container |
| Run a single node | `ros2 run makimate_bringup hello` | Inside container |
| Launch all nodes | `ros2 launch makimate_bringup bringup.launch.py` | Inside container |
| Push code to GitHub | `git add . && git commit -m "Update" && git push` | Host |

---

## 💡 Troubleshooting

1. **Permission Denied for Docker**:
   ```bash
   sudo usermod -aG docker $USER
   newgrp docker
   ```
   🔁 Log out and back in or restart your computer.

2. **Docker Not Running**:
   ```bash
   sudo systemctl start docker
   ```

3. **“Package not found” in ROS 2**:
   Ensure you’ve built and sourced the workspace:
   ```bash
   colcon build --merge-install
   source /opt/ros/jazzy/setup.bash
   source /ws/install/setup.bash
   ```

4. **Slow Docker Builds**:
   Use `--load` instead of `--push` for local testing to avoid multi-architecture builds.

5. **No Space Left on Device**:
   Clean up unused Docker images:
   ```bash
   docker system prune -af
   ```

6. **Buildx Missing**:
   Set up Buildx:
   ```bash
   docker buildx create --name mybuilder --use
   docker buildx inspect --bootstrap
   ```

---

## 🧠 Key Concepts

| Term | Description |
|------|-------------|
| **Docker Image** | A blueprint containing ROS 2, dependencies, and project code. |
| **Container** | A running instance of a Docker image. |
| **Buildx** | Docker tool for building multi-architecture images (e.g., for desktop and Raspberry Pi). |
| **Launch File** | ROS 2 script to start one or more nodes (e.g., `bringup.launch.py`). |
| **colcon** | ROS 2 build system for compiling packages. |

---

## 🧩 Cross-Platform Development

This setup ensures:
- ✅ Consistent ROS 2 environment across Windows, macOS, Linux, and Raspberry Pi.
- ✅ No manual ROS 2 or dependency installation required.
- ✅ Code pushed to GitHub is portable and builds identically for all team members.

---

## 🫶 Need Help?

If you encounter issues or have questions:
- Open a thread in the **#dev-setup** channel on Discord.
- Tag **@pourya9698** on Discord for direct assistance.

---

## 🎉 You’re Done!

You now have a fully functional, cross-platform ROS 2 Jazzy development environment for MakiMate. Start coding and building awesome robot features! 🤖