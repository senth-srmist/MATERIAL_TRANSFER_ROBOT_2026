# Material Transfer Robot - Docker Setup

## Prerequisites

> **Note:** Skip step 2 and 3 if your device does not have NVIDIA graphics.

### 1. Check Docker Version

```bash
docker --version
# Must be >= 20.x
```

### 2. Install NVIDIA Container Toolkit

```bash
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L "https://nvidia.github.io/libnvidia-container/${distribution}/libnvidia-container.list" -o /tmp/nvidia-container.list
sed -i 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' /tmp/nvidia-container.list
sudo mv /tmp/nvidia-container.list /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 3. Install ZED udev rules on the host

```bash
sudo mv CODE/99-slabs.rules /etc/udev/rules.d/99-slabs.rules
sudo chmod 644 /etc/udev/rules.d/99-slabs.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Build Dev Container

> **Note:** The base image is pre-built and hosted on GitHub Container Registry
> (`ghcr.io/tejasmk-tkp/material-transfer-robot-base-{amd64|arm64}:latest`).
> It will be pulled automatically during build — no manual setup needed.

```bash
# amd64 (Laptop)
TARGETARCH=amd64 docker compose build

# arm64 (Jetson)
TARGETARCH=arm64 docker-compose build
```

## Start and Run the Container

```bash
xhost +local:docker
```

### ROS2 Communication Modes

| Mode    | Description           | Network | Use Case                             |
| ------- | --------------------- | ------- | ------------------------------------ |
| local   | Localhost only        | No      | Single-device operation              |
| network | Full cross-device DDS | Yes     | Multi-device ROS2 (high bandwidth)   |
| viz     | Throttled remote rviz | Yes     | Remote visualization (low bandwidth) |

### Local Mode (Default)

```bash
docker compose up -d

# arm64 (Jetson)
docker-compose -f docker-compose.yml -f docker-compose.arm64.yml up -d
```

### Network Mode

```bash
ROS_MODE=network CYCLONE_INTERFACE_IP=<YOUR_IP> docker compose up -d

# arm64 (Jetson)
ROS_MODE=network CYCLONE_INTERFACE_IP=<YOUR_IP> docker-compose -f docker-compose.yml -f docker-compose.arm64.yml up -d
```

### Viz Mode (Remote RViz)

Run rviz on your laptop with minimal WiFi load. Robot stays local. A bridge forwards selected topics at throttled rates.

**On the Robot (Jetson):**

```bash
CYCLONE_INTERFACE_IP=<YOUR_IP> docker-compose -f docker-compose.yml -f docker-compose.arm64.yml -f docker-compose.viz.yml up -d material-transfer-robot viz-bridge
```

**On the Laptop:**

```bash
CYCLONE_INTERFACE_IP=<YOUR_IP> docker compose -f docker-compose.yml -f docker-compose.viz.yml up viz-laptop
```

RViz auto-launches. Default config: `/workspace/rviz/viz.rviz`

**Custom rviz config:**

```bash
CYCLONE_INTERFACE_IP=<YOUR_IP> RVIZ_CONFIG=/workspace/rviz/nav.rviz docker compose -f docker-compose.yml -f docker-compose.viz.yml up viz-laptop
```

**Customizing topics:** Edit `viz_bridge_config.yaml`, then `docker-compose restart viz-bridge`.

**Stopping:**

```bash
# Laptop
docker compose -f docker-compose.yml -f docker-compose.viz.yml stop viz-laptop

# Robot
docker-compose -f docker-compose.yml -f docker-compose.arm64.yml -f docker-compose.viz.yml down
```

### Access Container

```bash
docker exec -it material-transfer-robot-container bash
```

### Stop Container

```bash
docker compose down
```
