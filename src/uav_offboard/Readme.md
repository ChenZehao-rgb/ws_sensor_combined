# 开发环境安装 Ubuntu 22.04 + ROS 2 Humble + PX4 1.16

## 1. ROS 2 安装

使用小鱼一键安装：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

## 2. 安装相关库和依赖

安装 `pip3`：

```bash
sudo apt install python3-pip
```

安装 `git`：

```bash
sudo apt install git
```

安装 `colcon` 常用扩展：

```bash
sudo apt install python3-colcon-common-extensions
```

安装 ROS 2 Humble 桌面版与自动补全：

```bash
sudo apt install ros-humble-desktop python3-argcomplete
```

安装 ROS 开发工具：

```bash
sudo apt install ros-dev-tools
```

## 3. 安装 PX4 固件

克隆 PX4 仓库：

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

重新获取子模块：

```bash
cd PX4-Autopilot
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
```

说明：

- 默认会安装 Gazebo Harmonic 8。
- 如果没有出现需要 `reboot` 的提示，说明安装可能有问题；这时可以尝试切换 PX4 固件版本后重新安装。

切换 PX4 固件版本：

```bash
cd PX4-Autopilot
make clean
make distclean
git fetch origin release/1.15
git checkout release/1.14
git branch
make submodulesclean
bash ./Tools/setup/ubuntu.sh
sudo reboot
```

## 4. 安装 Micro-XRCE-DDS-Agent

克隆仓库：

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

## 5. 运行仿真

说明：在虚拟机下，需要关闭加速 3D 图形。

首先启动 ROS Agent：

```bash
MicroXRCEAgent udp4 -p 8888
```

启动 PX4 SITL 仿真：

```bash
make px4_sitl gz_x500
```

该命令会自动启动 PX4 Client。

如果不调用 Gazebo GUI，可以使用：

```bash
HEADLESS=1 make px4_sitl gz_x500
```

## 6. ROS Bag 包查看

安装 PlotJuggler：

```bash
sudo apt install ros-humble-plotjuggler-ros
```

在 ROS 2 中启动，需要先 source 编译环境：

```bash
source install/setup.bash
```

然后运行：

```bash
ros2 run plotjuggler plotjuggler
```

打开记录下的 YAML 文件即可。

---

# 无人机 Offboard 控制

## 1. Ruckig 库安装

克隆仓库：

```bash
git clone https://github.com/pantor/ruckig.git
```

编译：

```bash
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
```

如果提示缺少 `format` 头文件，可安装 GCC 13：

```bash
sudo apt update
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-13 g++-13
```

重新编译 Ruckig：

```bash
cd ~/ruckig
rm -rf build && mkdir build && cd build

cmake .. \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-13 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-13 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=20 \
  -DCMAKE_CXX_STANDARD_REQUIRED=ON \
  -DCMAKE_CXX_EXTENSIONS=OFF

make
sudo make install
```

## 2. ROS 2 项目环境

### 2.1 代码跳转问题

#### 方法一：合并 `compile_commands.json`

重新构建，在每个包下生成 `compile_commands.json`：

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

合并所有 `compile_commands.json` 到一个总文件中：

```bash
mkdir -p tools
touch tools/merge_compile_commands.py
```

`tools/merge_compile_commands.py` 内容如下：

```python
#!/usr/bin/env python3
import json
import glob
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = ROOT / "build"
OUT_FILE = ROOT / "compile_commands.json"


def main():
    entries = []
    seen = set()
    for fpath in glob.glob(str(BUILD_DIR / "*/compile_commands.json")):
        try:
            with open(fpath, "r") as f:
                data = json.load(f)
        except Exception:
            continue

        for e in data:
            key = (e.get("file"), e.get("directory"))
            if key in seen:
                continue
            seen.add(key)
            entries.append(e)

    if not entries:
        print("No compile_commands.json files found under build/* . Did you build with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON?")
        return 1

    OUT_FILE.write_text(json.dumps(entries, indent=2))
    print(f"Wrote {OUT_FILE} with {len(entries)} entries")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

执行合并脚本：

```bash
python3 tools/merge_compile_commands.py
```

在 `.vscode` 目录下新建 `settings.json`：

```json
{
  "files.associations": {
    "chrono": "cpp"
  },
  "C_Cpp.default.compileCommands": "${workspaceFolder}/compile_commands.json"
}
```

#### 方法二：让 VS Code 自动生成环境配置

删除 `.vscode` 目录下的所有文件，然后重启 VS Code。

## 3. uORB Bridged to ROS 2（uORB 参数使用指南）

### 3.1 `msg` 版本需一致

需要将 `px4_msgs` 克隆到 ROS 2 工作空间下，并保证 `px4_msgs` 版本与飞控版本一致。可将飞控目录下 `msg` 目录中的所有文件复制并替换到 `px4_msgs/msg` 下。

### 3.2 常用 Topic 实例

#### 3.2.1 `/fmu/in/vehicle_command`

**ROS 2 消息类型**

`px4_msgs::msg::VehicleCommand`（ROS 2 → PX4，写入 uORB `vehicle_command`）

**使用实例**：

```cpp
publish_vehicle_command(
    VehicleCommand::VEHICLE_CMD_DO_SET_MODE, // MAV_CMD_DO_SET_MODE
    1.0f,   // param1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    6.0f    // param2: PX4 main_mode = 6 (OFFBOARD)
);
```

**参数语义**（以 `MAV_CMD_DO_SET_MODE` 为例）：

- **Command**：`VEHICLE_CMD_DO_SET_MODE`（MAV_CMD = 176）
- **param1 = base_mode**：`MAV_MODE_FLAG` 位掩码 — [CMD 说明](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE)
- **param2 = custom_mode**：飞控自定义模式，在 PX4 中对应 `main_mode` — [MAV_MODE_FLAG](https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG)
- **param3 = custom_submode**：可选，PX4 某些主模式下才会使用 — [custom param](https://github.com/mavlink/MAVSDK/blob/ce6b7186d837b1ab5e9b23bb9be72aec28899630/src/mavsdk/core/px4_custom_mode.h)

说明：以上为 MAVLink 官方定义；标准模式另有 `MAV_CMD_DO_SET_STANDARD_MODE`，而自定义模式必须使用 `MAV_CMD_DO_SET_MODE`。参考：[mavlink.io](https://mavlink.io/en/services/standard_modes.html)

#### 3.2.2 `/fmu/out/home_position`

在 PX4 1.14、1.15 版本中，该 Topic 默认不会发给 ROS，需要在以下路径加入配置：

```text
~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```

添加内容：

```yaml
- topic: /fmu/out/home_position
  type: px4_msgs::msg::HomePosition
  rate_limit: 5.
```

## 4. 仿真运行

### 4.1 准备工作

先按本节第 1 部分安装 Ruckig 库环境（底层的轨迹规划库）。

将 `px4_msgs` 库 clone 到 `src` 目录下，并切换到 1.16 分支：

```bash
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs
git checkout release/1.16
```

编译工作空间：

```bash
cd /ws_sensor_combined
colcon build
```

### 4.2 启动终端

**Terminal 1 — Micro-XRCE-DDS Agent**

```bash
MicroXRCEAgent udp4 -p 8888
```

与实际飞控连接可以使用：

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

**Terminal 2 — PX4 SITL**

```bash
make px4_sitl gz_x500
```

**Terminal 3 — 规划器节点**

```bash
cd /ws_sensor_combined
source install/setup.bash
ros2 launch traj_offboard offboard_traj.launch.py
```

**Terminal 4 — 状态机**

```bash
cd /ws_sensor_combined
source install/setup.bash
ros2 launch uav_offboard_fsm uav_offboard_fsm.launch.py
```

**Terminal 5 — 按键控制**

```bash
cd /ws_sensor_combined
source install/setup.bash
ros2 run uav_offboard_fsm uav_keyboard_control_node
```

**Terminal 6 — 地面站**

打开地面站。

注意：

- 需打开地面站才能 ready to fly，才能正确切到 offboard 模式。
- 如果使用 Jetson，没有地面站的情况下，需要在 PX4 的 `pxh>` 终端里设置：

  ```bash
  param set NAV_DLL_ACT 0
  ```

---

# 实机部署（Jetson）

## 1. Fast DDS Discovery Server

启动 server：

```bash
fast-discovery-server -i 0 -l 192.168.4.2 -p 11811
```

## 2. Micro-XRCE-DDS Agent

启动 agent（串口连接飞控）：

```bash
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600 -c px4_participant.xml
```

## 3. RealSense 相机启动

```bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=1280x720x15 \
  rgb_camera.color_profile:=1280x720x15 \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true \
  enable_accel:=true \
  enable_gyro:=true \
  unite_imu_method:=1
```

## 4. 图像压缩节点

```bash
cd ros2_xmy
source fast<TAB>   # 自动补全 fast-dds 环境
ros2 run d435i_image_compressor image_compressor_node
```

## 5. 文件传输（scp / rsync）

通用语法：

```bash
scp -r ./myfolder user@host:/remote/path/
```

示例：

```bash
scp -r ws_sensor_combined/src/uav_offboard/uav_offboard_fsm/ neu@192.168.4.2:/home/neu/manipulator_ws/src/uav_offboard
scp -r ws_sensor_combined/src/uav_offboard/traj_offboard/ sia@10.225.115.210:/home/sia/ws_sensor_combined/src/uav_offboard
```

## 6. ROS Bag 修复

重建索引：

```bash
ros2 bag reindex /path/to/bag_folder
```

如果 bag 文件损坏，先用 sqlite 的 `.recover` 命令修复数据库：

```bash
sqlite3 online_traj_19700101_080654_0.db3 ".recover" | sqlite3 recovered.db3
```

然后再重建索引：

```bash
ros2 bag reindex /path/to/bag_folder -s sqlite3
```

stdbuf -oL ros2 topic bw /camera/camera/relay/color/image_raw | ts '[%Y-%m-%d %H:%M:%S]' | tee bw9.log

nohup ./netlog.sh > /dev/null 2>&1 &

