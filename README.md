## Host Setup

This dev container ships several configurations. After cloning, open the folder in
VS Code, run **Dev Containers: Reopen in Container**, and pick the configuration that
matches your machine from the list. Your choice is remembered per machine, so you only
pick once.

| Your machine | Pick this config |
|---|---|
| Linux with an NVIDIA GPU | `ROS2 Humble (Linux + NVIDIA, Wayland host)` |
| Linux with integrated graphics | `ROS2 Humble (Ubuntu, no GPU)` |
| Windows with integrated graphics | `ROS2 Humble (Windows + WSLg)` |

All hosts need [VS Code](https://code.visualstudio.com/) with the
**Dev Containers** extension (`ms-vscode-remote.remote-containers`).

---

### Linux with an NVIDIA GPU (Podman)

This path uses Podman with CDI GPU injection and runs GUI apps over X11/XWayland.

**Supported distros:** Fedora 40+ (incl. 44), RHEL 9 / Rocky Linux 9 / AlmaLinux 9,
openSUSE Tumbleweed — anything shipping **Podman 4+**. (Other distros work too if they
provide Podman and the NVIDIA Container Toolkit.)

**Prerequisites**

1. Install Podman:
   ```bash
   sudo dnf install -y podman          # Fedora / RHEL family
   ```
2. Install the NVIDIA proprietary driver for your GPU (e.g. via RPM Fusion on Fedora),
   then confirm `nvidia-smi` works on the host.
3. Install the NVIDIA Container Toolkit and generate the CDI spec:
   ```bash
   sudo dnf install -y nvidia-container-toolkit
   sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
   ```
   Re-run the `cdi generate` command after any NVIDIA driver update.
4. Verify CDI and GPU access:
   ```bash
   nvidia-ctk cdi list                 # should show nvidia.com/gpu=all
   podman run --rm --device nvidia.com/gpu=all \
     docker.io/nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
   ```
5. Make sure `xhost` is available (the container authorizes X automatically via it):
   ```bash
   sudo dnf install -y xorg-x11-server-utils
   ```
6. Point VS Code at Podman — add to your VS Code `settings.json`:
   ```json
   "dev.containers.dockerPath": "podman"
   ```

**Notes**

- rviz renders on your **integrated GPU** (hardware-accelerated XWayland). The NVIDIA
  card stays available for CUDA / compute. This is intentional and avoids
  NVIDIA-on-Wayland rendering issues.
- A Wayland desktop session is assumed (the default on modern GNOME/KDE). An X11
  session also works.

---

### Linux with integrated graphics (Docker)

No discrete GPU required. GUI apps render on the integrated GPU (or fall back to
software rendering on a headless/VM host).

**Supported distros:** Ubuntu 22.04 / 24.04, Debian 12, Linux Mint 21+, Pop!_OS 22.04 —
any Debian/Ubuntu-family host with **Docker Engine**. (Also works on Fedora or others
running Docker; if you prefer rootless Podman, see the note below.)

**Prerequisites**

1. Install Docker Engine and add yourself to the `docker` group:
   ```bash
   sudo apt-get install -y docker.io
   sudo usermod -aG docker "$USER"     # log out / back in afterwards
   ```
2. Make sure `xhost` is installed (used to authorize the X connection):
   ```bash
   sudo apt-get install -y x11-xserver-utils
   ```
3. For hardware-accelerated rendering, confirm `/dev/dri` exists (it does on any host
   with integrated graphics + Mesa drivers). To use it, uncomment the `/dev/dri` device
   line in `ubuntu-nogpu/devcontainer.json`. Leave it commented on a headless/VM host.

**Notes**

- Expect `llvmpipe` (CPU) rendering only on hosts with no GPU at all; it works but is
  slower for large scenes.
- **Using rootless Podman instead of Docker?** Add
  `"--userns=keep-id:uid=0,gid=0"` to `runArgs` in `ubuntu-nogpu/devcontainer.json` so
  socket permissions line up.

---

### Windows with integrated graphics (WSLg)

GUI apps reach the Windows desktop through WSLg, which ships with WSL2.

**Supported versions:** Windows 11 (any), or Windows 10 22H2 with an up-to-date WSL2.
(WSLg requires a recent WSL2 — older Windows 10 builds are not supported.)

**Prerequisites**

1. Install / update WSL2 (this includes WSLg):
   ```powershell
   wsl --install
   wsl --update
   ```
2. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) and enable
   the WSL2 backend: **Settings → General → Use the WSL 2 based engine**.
3. Install your normal Windows GPU driver (Intel/AMD) — this provides GPU support inside
   WSL automatically; no extra container flags are needed.
4. Install VS Code with the **Dev Containers** and **WSL** extensions.

**Notes**

- For the smoothest experience, clone the repository into your WSL2 filesystem (e.g.
  `\\wsl$\Ubuntu\home\<you>\...`) rather than a Windows path, then open it in VS Code.
- No `xhost` step is needed — WSLg handles X authorization for you.

---

### Verifying it works

Inside the container terminal:

```bash
printenv ROS_DISTRO          # -> humble
glxinfo | grep "OpenGL renderer"   # shows your GPU (or llvmpipe if software)
glxgears                     # a window with spinning gears should appear
rviz2                        # the rviz window should open
```

If `glxgears` renders but `rviz2` fails to create a window, confirm
`QT_QPA_PLATFORM=xcb` is set in your config — rviz's renderer requires X11, not Wayland.

---

## Husky/A200 Simulation

Use the ROS 2 Humble Clearpath simulator stack for Husky/A200. The old `husky_gazebo` tutorial is ROS 1 (`roslaunch`); this repository uses `clearpath_gz` and an A200 `robot.yaml`.

Checkout this branch before testing:

```bash
git checkout feature/husky-simulation
```

The Dev Container installs the verified simulation packages:

- `ignition-fortress`
- `ros-humble-clearpath-simulator`
- `ros-humble-teleop-twist-keyboard`

After changing `.devcontainer/Dockerfile`, rebuild the Dev Container before testing from a clean environment.

The default robot config is stored in the repo at:

```bash
config/husky/a200_default.yaml
```

Start Gazebo and RViz from the first Dev Container terminal:

```bash
bash scripts/husky_sim.sh
```

The helper copies `config/husky/a200_default.yaml` to `~/clearpath/robot.yaml`, then launches `clearpath_gz` with RViz enabled. Pass normal launch arguments after the script name, for example:

```bash
RVIZ=false bash scripts/husky_sim.sh
bash scripts/husky_sim.sh x:=1.0 y:=0.0 yaw:=1.57
```

Drive the robot from a second Dev Container terminal:

```bash
bash scripts/husky_teleop.sh
```

The simulated Husky uses the `/a200_0000` namespace. Check teleop output with:

```bash
ros2 topic echo /a200_0000/cmd_vel
```

Expected result: Gazebo shows the Husky/A200 model, RViz displays the robot state, and keyboard teleop moves the robot in simulation.

IMU is not implemented in this branch yet. This branch only provides the Husky/A200 Gazebo base simulation, RViz visualization, and `/a200_0000/cmd_vel` teleop control path for future IMU integration.
