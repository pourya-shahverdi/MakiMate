# 🧠 MakiMate Setup Guide for Linux

Welcome to **MakiMate**, a cross-platform **ROS 2 Jazzy** codebase for the Maki robot! This guide walks you through setting up a development environment on **Ubuntu 22.04/24.04** using **Docker**. By following these steps, you’ll be able to develop, test, and deploy robot software in a consistent ROS 2 environment.

---

## 🚀 Overview

MakiMate uses:
- **ROS 2 Jazzy**: Robotics middleware for nodes, topics, and services.
- **Docker**: Ensures isolated, reproducible build environments.
- **GitHub**: Facilitates version control and team collaboration.
- **VS Code** (optional): Recommended editor with ROS 2 and Docker support.

**Benefits**:
- ✅ Run the same ROS 2 environment as your team.
- ✅ Develop and test locally with ease.
- ✅ Share code that builds identically across systems.

---

## 🐧 Linux Setup (One-Time)

### 1️⃣ Install Docker
1. Remove Old Docker Versions:
   ```bash
   sudo apt remove -y docker docker-engine docker.io containerd runc docker-compose-plugin
   sudo snap remove docker 2>/dev/null || true   ```
2. Install Prerequisites and Add Docker Repository:
   ```bash
   sudo apt update
   sudo apt install -y ca-certificates curl gnupg

   # Create the keyrings directory if it doesn't exist
   sudo install -m 0755 -d /etc/apt/keyrings

   # Add Docker’s GPG key (correct path and permissions)
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg   | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
   sudo chmod a+r /etc/apt/keyrings/docker.gpg

   # Add the Docker APT repository (Jammy=22.04, Noble=24.04)
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

   sudo apt update
   ```
3. Install Docker Engine and Plugins:
   ```bash
   sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   ```
4. Enable and Start Docker:
   ```bash
   sudo systemctl enable --now docker
   sudo usermod -aG docker $USER
   newgrp docker    # refresh group membership in the current shell
   ```
5. Verify Installation:
   ```bash
   docker run hello-world
   docker buildx version
   ```
*If Buildx is missing, initialize it manually:
```bash
docker buildx create --name mybuilder --use
docker buildx inspect --bootstrap
```
### ⚙️ Troubleshooting

#### 🔹 GPG Key Error (NO_PUBKEY / InRelease not signed)
```bash
ls -l /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
sudo apt update
```

#### 🔹 “No Installation Candidate” for docker-ce
Check that the repository entry matches your Ubuntu version:
```bash
cat /etc/apt/sources.list.d/docker.list
apt-cache policy docker-ce | sed -n '1,120p'
```

#### 🔹 Permission Denied for /var/run/docker.sock
```bash
groups          # should include 'docker'
newgrp docker   # or log out and back in
sudo systemctl status docker --no-pager
```

#### 🔹 Raspberry Pi (ARM64) Tip
Ensure you build and run ARM64 images:
```bash
docker run --platform=linux/arm64 -it alpine uname -m   # should print: aarch64
docker buildx build --platform=linux/arm64 -t your/image:tag .
```

### 2️⃣ Install Git and Basic Packages
```bash
sudo apt update
sudo apt install -y git build-essential curl python3-pip
```

### 3️⃣ Install VS Code (Optional)
1. Download **VS Code**: [https://code.visualstudio.com/](https://code.visualstudio.com/).
2. Install extensions:
   - **Dev Containers**
   - **Docker**
   - **ROS** (optional, for ROS 2 support)
3. Run `code .` in your project folder to open VS Code.

### 4️⃣ (Optional) Set Up GitHub SSH Authentication
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

## 🧠 Clone the Repository
```bash
git clone https://github.com/pourya-shahverdi/MakiMate.git
cd MakiMate
```
Or, if using SSH:
```bash
git clone git@github.com:pourya-shahverdi/MakiMate.git
cd MakiMate
```

---

## ⚙️ Build the Docker Image
MakiMate uses **Docker Buildx** to create images for desktop (x86_64) and Raspberry Pi 5 (ARM64).

### Option 1: Local Build (Recommended for Beginners)
```bash
docker buildx build . \
  -f docker/robot/Dockerfile \
  --platform linux/amd64 \
  -t makimate:dev \
  --load
```

### Option 2: Multi-Architecture Build (for Desktop + Raspberry Pi)
```bash
docker login ghcr.io
docker buildx build . \
  -f docker/robot/Dockerfile \
  --platform linux/amd64,linux/arm64 \
  -t ghcr.io/pourya-shahverdi/makimate:latest \
  -t ghcr.io/pourya-shahverdi/makimate:v0.1.0 \
  --push
```

---

## 🧩 Run the Container
1. Run the container:
   ```bash
   docker run --rm -it makimate:dev
   ```
   **Expected Output**:
   ```
   [INFO] [hello_node]: Hello from MakiMate!
   ```

2. Explore inside:
   ```bash
   docker run -it --entrypoint bash makimate:dev
   ```
3. Inside the container:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source /ws/install/setup.bash
   ros2 launch makimate_bringup bringup.launch.py
   ```
4. Exit: `exit` or `Ctrl+D`.

---

## 🏗️ Project Structure

```
MakiMate/
├── LICENSE                     # Project license file
├── README.md                   # Project overview
├── deploy/                     # Deployment scripts/configurations
├── docker/                     # Docker configurations
│   ├── base/                   # Base Docker image
│   │   └── Dockerfile
│   └── robot/                  # Robot-specific configurations
│       ├── Dockerfile
│       └── entrypoint.sh
├── docs/                       # Documentation
│   └── SETUP.md
├── hw/                         # Hardware-related code
├── interfaces/                 # ROS 2 interfaces (msg, srv, etc.)
└── src/                        # ROS 2 packages
    └── makimate_bringup/
        ├── launch/             # Launch files
        │   └── bringup.launch.py
        ├── makimate_bringup/   # Python package
        │   ├── __init__.py
        │   └── hello.py
        ├── package.xml         # ROS 2 package manifest
        ├── resource/           # Resource files
        │   └── makimate_bringup
        ├── setup.cfg           # Python package config
        └── setup.py            # Build script
```

---

## 🧰 Common Commands

| Action | Command | Run Where |
|--------|---------|-----------|
| Rebuild image | `docker buildx build . -f docker/robot/Dockerfile -t makimate:dev --load` | Host |
| Run container | `docker run -it --entrypoint bash makimate:dev` | Host |
| Source ROS 2 | `source /opt/ros/jazzy/setup.bash` | Container |
| Build workspace | `colcon build --merge-install` | Container |
| Run node | `ros2 run makimate_bringup hello` | Container |
| Launch nodes | `ros2 launch makimate_bringup bringup.launch.py` | Container |
| Push code | `git add . && git commit -m "Update" && git push` | Host |

---

## 💡 Troubleshooting

1. **Docker Not Running**:
   ```bash
   sudo systemctl start docker
   ```

2. **Permission Denied**:
   ```bash
   sudo usermod -aG docker $USER
   newgrp docker
   ```

3. **“Package not found” in ROS 2**:
   ```bash
   colcon build --merge-install
   source /opt/ros/jazzy/setup.bash
   source /ws/install/setup.bash
   ```

4. **Buildx Missing**:
   ```bash
   docker buildx create --name mybuilder --use
   docker buildx inspect --bootstrap
   ```

5. **No Space Left**:
   ```bash
   docker system prune -af
   ```

---

## 🧠 Cross-Platform Development
This setup ensures:
- ✅ Consistent ROS 2 environment across systems.
- ✅ Same image runs on desktop and Raspberry Pi.
- ✅ No manual ROS 2 installation required.

---

## 🫶 Need Help?
Reach out on Discord (@pourya9698) or open a thread in the **#dev-setup** channel.

---

## 🎉 You’re Done!
You now have a fully functional ROS 2 Jazzy environment for MakiMate on Linux. Happy coding! 🤖
