# hiwonder_ros2

ROS 2 workspace and Docker environment for Hiwonder ROS 2 development.

The repository provides a reproducible development environment using Docker for both **native Linux** and **Windows (WSL)** systems.

---

# Installation

## 1. Install Docker

### Linux

Follow the Docker installation instructions in:

```
docker/docker_install.md
```

or refer to the official Docker documentation.

---

### Windows (via WSL)

Docker Desktop is **not required and has not been tested with this setup**.
Instead, Docker is installed directly inside WSL.

#### 1. Install WSL

Open a PowerShell terminal and run:

```bash
wsl --install
```

---

#### 2. List available distributions

```bash
wsl.exe --list --online
```

---

#### 3. Install Ubuntu (recommended)

```bash
wsl.exe --install Ubuntu-24.04
```

---

#### 4. Install Docker inside WSL

Enter the WSL environment and install Docker following the **Linux installation instructions** described in:

```
docker/docker_install.md
```

---

# 2. Setup the Repository

Clone the repository:

```bash
git clone --recurse-submodules https://github.com/KevinEppacher/hiwonder_ros2.git
```

Checkout the ROS 2 Jazzy branch:

```bash
git checkout jazzy
```

---

# 3. Start the Docker Environment

Navigate to the Docker configuration directory:

```bash
cd /path/to/repo/hiwonder_ros2/docker
```

## Native Linux

```bash
docker compose up -d --build ros2_manipulation
```

## Windows (WSL)

```bash
docker compose up -d --build ros2_manipulation_wsl
```

---

# 4. Enter the Container

```bash
docker exec -it <container_name> bash
```

Example:

```bash
docker exec -it ros2_manipulation_container bash
```

---

# 5. Build the ROS 2 Workspace

Inside the container:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

---

# Notes

* The Docker setup supports both **native Linux** and **WSL environments**.
* GUI tools such as **RViz** are supported via X11/WSLg.
* ROS packages are mounted into the container and built inside the workspace.

---

# To Be Added

* Navigation container usage
* Launch instructions
* Development workflow
* CI/CD integration


## Docker Containers:
The hiwonder_ros2 folder contains general hiwonder packages, which are mounted to every container. Container specific packages are located within the same repspective folders.

- **Manipulation Container**:

The docker manipulation container contains MoveIt2 specific package installs.
- **Navigation Container**:

The docker navigation container contains Nav2 specific package installs.


