# Floor-Level Material Transfer Mobile Robot

This repository contains the software for the **development of an autonomous floor-level material transfer mobile robot**.
The system is designed to operate within a **single indoor floor environment**, autonomously transporting materials between designated pickup and drop-off points (such as room entrances), while safely coexisting with human activity.

The project aims to establish a **validated core platform for indoor autonomous logistics**, integrating navigation, task handling, and human-aware operation. It serves as a foundation for scalable deployment across larger indoor environments such as offices, academic buildings, and hospitals. 

---

## Repository Setup

### 1. Clone the repository

```bash
git clone https://github.com/senth-srmist/MATERIAL_TRANSFER_ROBOT_2026.git
cd MATERIAL_TRANSFER_ROBOT_2026
```

---

### 2. Initialize and update top-level submodules

This repository uses Git submodules for managing external dependencies.

```bash
git submodule update --init --recursive
```

---

## ZED Wrapper Setup

The `zed_wrapper` submodule requires **platform-specific versioning**.
Incorrect versions may lead to build or runtime failures.

### Navigate to the ZED wrapper

```bash
cd CODE/ros_ws/src/zed_wrapper
```

---

### Update submodules inside `zed_wrapper`

```bash
git submodule update --init --recursive
```

---

## ZED Wrapper Version Selection

### Jetson (ARM64)

For Jetson-based systems, the ZED wrapper **must** be set to version **humble-v4.0.8** for compatibility.

```bash
git checkout humble-v4.0.8
```

---

### Laptop / Desktop (x86_64)

For x86_64 systems (laptops or desktops), use **humble-v5.1.0**.

```bash
git checkout humble-v5.1.0
```

---

### Verify the selected version

```bash
git describe --tags
```

---

## Updating the Repository

Whenever pulling new changes, ensure submodules are kept in sync:

```bash
git pull
git submodule update --init --recursive
```

---
