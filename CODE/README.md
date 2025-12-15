# Material Transfer Robot - Docker Setup

## Prerequisites

### 1. Check Docker Version
```bash
docker --version
# Must be >= 20.x
```

### 2. Install NVIDIA Container Toolkit
```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 3. Enable X11 Display Access
```bash
xhost +local:docker
```

## Build & Run

### Build Base Image
```bash
docker build -f Dockerfile.base -t material-transfer-robot-base:latest .
```

### Build Production Image
```bash
docker compose build
```

### Start Container
```bash
docker compose up -d
```

### Access Container
```bash
docker exec -it material-transfer-robot-container bash
```

### Stop Container
```bash
docker compose down
```

## Rebuild After Updates

**Base Image:**
```bash
docker build -f Dockerfile.base -t material-transfer-robot-base:latest .
```

**Production Image:**
```bash
docker compose build
```
