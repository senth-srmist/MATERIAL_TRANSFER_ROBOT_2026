# Material Transfer Robot - Docker Setup

## Prerequisites

> **Note:** Skip step 2 and 3 if your device doesn't have NVIDIA graphics. Any ZED Camera related packages will not work in this case.

### 1. Check Docker Version
```bash
docker --version
# Must be >= 20.x
```

### 2. Install NVIDIA Container Toolkit
```bash
# Set distribution variable
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
echo "Detected: $distribution"

# Add GPG key
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Download list file
curl -s -L "https://nvidia.github.io/libnvidia-container/${distribution}/libnvidia-container.list" -o /tmp/nvidia-container.list

# Add signed-by to the file
sed -i 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' /tmp/nvidia-container.list

# Move to apt sources
sudo mv /tmp/nvidia-container.list /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Now update and install
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker``
```

### 3. Install ZED udev rules on the host

Run this on your **host**:

```bash
# Download and install ZED udev rules
sudo mv CODE/99-slabs.rules /etc/udev/rules.d/99-slabs.rules
sudo chmod 644 /etc/udev/rules.d/99-slabs.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 4. Configure CycloneDDS Network Interface

CycloneDDS requires each device to specify its network interface IP for ROS2 cross-device communication. All devices must be connected to the same network (the Jetson AP(myrobo) network: `192.168.50.x`).

#### Step 1: Identify your IP on the AP network

```bash
ip addr show
```

Look for the interface connected to the `192.168.50.x` subnet:
- **Jetson (AP host):** `wlan1` with IP `192.168.50.1`
- **Other devices:** Look for `192.168.50.x` IP (e.g., `192.168.50.97`)

> **Important:** Make sure you're connected to the Jetson AP network(myrobo), not your home/office network.

#### Step 2: Add the environment variable to your shell

For **bash** (`~/.bashrc`):
```bash
echo 'export CYCLONE_INTERFACE_IP=<YOUR_IP>' >> ~/.bashrc
source ~/.bashrc
```

For **zsh** (`~/.zshrc`):
```bash
echo 'export CYCLONE_INTERFACE_IP=<YOUR_IP>' >> ~/.zshrc
source ~/.zshrc
```

#### Step 3: Verify the environment variable

```bash
echo $CYCLONE_INTERFACE_IP
# Should print your IP
```

## Build the Container

### Build Base Image
```bash
docker build -f Dockerfile.base -t material-transfer-robot-base:latest .
```

### Build Production Image
```bash
# On amd64 devices
docker compose build

# On arm64(referring to jetson nano in this case) devices
docker-compose -f docker-compose.yml -f docker-compose.arm64.yml build
```

## Start and Run the Container

### Enable X11 Display Access
```bash
xhost +local:docker
```

### Start Container
```bash
# On amd64 devices
docker compose up -d

# On arm64(referring to jetson nano in this case) devices
docker-compose -f docker-compose.yml -f docker-compose.arm64.yml up -d
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
