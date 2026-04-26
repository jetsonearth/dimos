# How to Integrate a New Robot with DimOS

This guide walks you through integrating any robot - humanoid, quadruped, drone, wheeled base, or manipulator arm - with DimOS. It's based on real integration experiences with the Unitree Go2/G1/B1, Galaxea R1 Pro, Booster K1, Deep Robotics M20, AgileX Piper, UFactory xArm, and others.

**Time estimate**: 2-5 weeks depending on your robot's interface quality.

**What you'll build**: A connection module (or adapter) that lets DimOS talk to your robot, plus blueprints that assemble it with perception, navigation, and agent capabilities.

> **Branch note**: These examples target the current `dev` branch APIs. Create integration PRs from `dev` and target `dev`; do not copy snippets from older feature branches without adapting them to the current module, blueprint, and ControlCoordinator APIs.

---

## Before You Start

### What you need from your robot vendor

| Item | Why | Priority |
|------|-----|----------|
| Python SDK or ROS 2 interface | Primary control channel | Required |
| Network access (ethernet/WiFi) | Connect your dev machine to the robot | Required |
| Camera stream (RTSP/ROS/WebSocket) | Visual perception | Required for agentic use |
| Odometry data (position + orientation) | Navigation and mapping | Required for navigation |
| LiDAR point cloud | Obstacle avoidance and SLAM | Required for navigation |
| IMU data | Localization accuracy | Recommended |
| URDF model file | Motion planning and collision checking | Required for arm planning |
| API documentation | Know what commands the robot accepts | Very helpful |
| SSH access to the robot's onboard computer | Debug, check topics, install software | Very helpful |

> **Tip**: If your robot doesn't provide odometry, LiDAR, or IMU natively, you can add external sensors (e.g., Livox MID-360 for LiDAR + IMU, Intel RealSense for depth). DimOS has built-in modules for these - see [External Sensors](#external-sensors-filling-the-gaps).

### What you need on your dev machine

- Ubuntu 22.04/24.04 (recommended) or macOS
- Python 3.12
- DimOS installed. For repo development, run `uv sync --all-extras --no-extra dds`. For package installs, use the extras your robot needs, e.g. `uv pip install 'dimos[base,unitree,manipulation]'`.
- Your robot's vendor SDK installed

---

## The Integration Flow

Every robot integration follows the same five phases, regardless of form factor:

```
Phase 1: Connect        Can you ping the robot? Can you see its API?
    |
Phase 2: Read Sensors   Can you receive camera frames, joint states, odometry?
    |
Phase 3: Send Commands  Can you make the robot move?
    |
Phase 4: Write Module   Wrap it all in a DimOS Module or Adapter
    |
Phase 5: Build Blueprint  Assemble modules into a runnable system
```

The sections below walk through each phase in detail.

---

## Phase 1: Connect to the Robot

This is always the first thing you do, and always where the first surprises happen.

### 1a. Establish network connectivity

Connect your dev machine to the robot (ethernet or WiFi) and figure out the IP addresses.

```bash
# If the robot's IP is unknown, try these:
ping 192.168.123.1      # Common Unitree default
ping 192.168.1.1        # Common default gateway

# If ping doesn't work, scan the subnet:
sudo arp-scan --interface=eth0 192.168.123.0/24

# Or watch for traffic:
sudo tcpdump -i eth0 -n | head -20
```

> **Real example (R1 Pro)**: The robot had no known IP on ethernet. We used `tcpdump` and `arp -a` to discover it, then manually assigned a static IP via netplan.

### 1b. Verify you can talk to the robot's API

Depending on how your robot exposes its interface:

**If your robot uses ROS 2:**
```bash
# Set up ROS 2 environment to match the robot
source /opt/ros/humble/setup.bash   # or jazzy
export ROS_DOMAIN_ID=0              # match your robot's domain ID

# List available topics
ros2 topic list --no-daemon

# You should see topics like:
#   /cmd_vel
#   /joint_states
#   /camera/image_raw
#   /odom
#   /scan or /points
```

**If your robot uses a Python SDK:**
```python
# Test basic SDK connectivity
from your_robot_sdk import RobotClient

robot = RobotClient("192.168.1.100")
robot.connect()
print(robot.get_status())   # Should return something
robot.disconnect()
```

**If your robot uses a custom protocol (UDP/TCP/WebSocket):**
```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Send a heartbeat / status request
sock.sendto(b'\x00\x01', ("192.168.1.100", 8000))
data, addr = sock.recvfrom(1024)
print(f"Got response: {data.hex()}")
```

### Common Phase 1 problems

| Problem | Solution |
|---------|----------|
| Can't ping the robot | Check cable, check subnet (robot might be on `192.168.123.x` not `192.168.1.x`) |
| ROS 2 topics not visible | Check `ROS_DOMAIN_ID`, check `ROS_LOCALHOST_ONLY` is not set to 1 on the robot |
| Topics visible but empty | DDS middleware mismatch - make sure both sides use the same RMW (FastDDS or CycloneDDS) |
| Topics visible from robot but not from laptop | FastDDS sending multicast on wrong interface - create a FastDDS XML profile to bind to the correct ethernet interface |

> **Real example (R1 Pro)**: Five separate network issues had to be solved: `ROS_LOCALHOST_ONLY=1` in the robot's bashrc, CycloneDDS/FastDDS EDP incompatibility, FastDDS wrong interface, `interfaceWhiteList` renamed in FastDDS 3.x, and a misleading discovery server. Each one silently broke topic visibility.

---

## Phase 2: Read Sensor Data

Once connected, verify you can receive data from the robot's sensors. Start with the camera (most visual feedback), then odometry, then LiDAR.

> **Note**: The commands below are for standalone verification - confirming your robot's sensors are actually publishing data. When you build your DimOS module (Phase 4), you won't write raw ROS nodes. DimOS has a **ROS Transport** layer that bridges ROS topics into DimOS streams automatically - see [2e. Bringing Sensor Data into DimOS](#2e-bringing-sensor-data-into-dimos-ros-transport).

### 2a. Camera

Most robots expose camera feeds in one of three ways:

**ROS 2 topic** (most common for ROS-based robots):
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

rclpy.init()
node = rclpy.create_node('camera_test')
def callback(msg):
    print(f"Got frame: {len(msg.data)} bytes")
node.create_subscription(CompressedImage, '/camera/image/compressed', callback, 10)
rclpy.spin_once(node, timeout_sec=5.0)
```

**RTSP stream** (common for industrial robots):
```python
import av

container = av.open("rtsp://192.168.1.100:8554/video1")
for frame in container.decode(video=0):
    img = frame.to_ndarray(format='bgr24')
    print(f"Got frame: {img.shape}")
    break
```

**WebSocket** (common for consumer robots):
```python
import asyncio, websockets

async def test():
    async with websockets.connect("ws://192.168.1.100:8080/video") as ws:
        data = await ws.recv()
        print(f"Got frame: {len(data)} bytes")

asyncio.run(test())
```

### 2b. Odometry

Odometry tells the robot where it is in the world. Check if your robot publishes it:

```bash
# ROS 2
ros2 topic echo /odom --once

# You want to see: position (x, y, z) and orientation (quaternion)
```

If your robot doesn't provide odometry, you have two options:
1. **External SLAM** - Add a LiDAR (e.g., MID-360) and use FAST-LIO2 (already in DimOS)
2. **Dead reckoning** - Integrate velocity commands over time (inaccurate but works as a fallback)

### 2c. LiDAR / Point Cloud

```bash
# ROS 2
ros2 topic echo /scan --once           # 2D laser scan
ros2 topic echo /points --once          # 3D point cloud
```

### 2d. Joint States (for arms and humanoids)

```bash
# ROS 2
ros2 topic echo /joint_states --once

# You want: name[], position[], velocity[], effort[]
```

### 2e. Bringing sensor data into DimOS (ROS Transport)

Once you've verified sensor data is flowing (above), you don't need to write raw `rclpy` nodes to use it in DimOS. The **ROS Transport** layer bridges ROS topics into DimOS streams with automatic message conversion.

In your blueprint, map stream names to ROS topics:

```python
from dimos.core.transport import ROSTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

yourmodel_ros = yourmodel_basic.transports({
    ("color_image", Image): ROSTransport("/camera/image/compressed", Image),
    ("odom", PoseStamped): ROSTransport("/odom", PoseStamped),
    ("lidar", PointCloud2): ROSTransport("/points", PointCloud2),
})
```

DimOS handles ROS 2 node creation, message type conversion (DimOS types to/from ROS types), and threading. Your module code and downstream consumers should still work with DimOS message types; the ROS details stay at the transport boundary.

> **Real example (Go2 ROS)**: The Go2's ROS blueprint maps four streams in one call:
> ```python
> unitree_go2_ros = unitree_go2.transports({
>     ("lidar", PointCloud2): ROSTransport("lidar", PointCloud2),
>     ("global_map", PointCloud2): ROSTransport("global_map", PointCloud2),
>     ("odom", PoseStamped): ROSTransport("odom", PoseStamped),
>     ("color_image", Image): ROSTransport("color_image", Image),
> })
> ```

---

## Phase 3: Send Motion Commands

Now make the robot move. **Start small** - send a tiny velocity for a short duration.

### Mobile bases (wheeled, legged, quadruped)

Most mobile robots accept velocity commands as a `Twist` message (linear + angular velocity):

**ROS 2:**
```bash
# Move forward slowly for 1 second
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

**Python SDK:**
```python
robot.move(linear_x=0.1, angular_z=0.0)
time.sleep(1.0)
robot.stop()
```

### Manipulator arms

Arms typically accept joint position or joint velocity commands:

```python
# Read current position first (so you know where you are)
current = robot.get_joint_positions()
print(f"Current joints: {current}")

# Move joint 0 by a tiny amount
target = list(current)
target[0] += 0.1  # radians
robot.set_joint_positions(target)
```

### Common Phase 3 problems

| Problem | Solution |
|---------|----------|
| Command sent but robot doesn't move | Check for "gates" or safety interlocks - some robots require multiple conditions to be satisfied before accepting commands |
| Robot moves erratically | Check units - your SDK might expect degrees, DimOS uses radians. Check if commands are velocity or position |
| Robot moves once then stops | Some robots need continuous commands at a minimum frequency (e.g., 20Hz). Add a control loop |
| Emergency stop needed | Know how to e-stop your robot BEFORE testing motion. Hardware e-stop button is best |

> **Real example (R1 Pro)**: The chassis had three hidden "gates" that all had to be unlocked simultaneously - a subscriber count gate, a braking mode flag, and an acceleration limit that defaulted to zero. This took multiple sessions of investigation, including binary disassembly of the control node.

> **Real example (M20)**: Velocity commands had to be normalized to [-1, 1] range for UDP mode, but sent as absolute m/s for DDS navigation mode. Two completely different code paths for the same robot.

---

## Phase 4: Write Your DimOS Module

Now wrap everything into a DimOS module. There are two patterns depending on your robot type:

### Pattern A: Connection Module

Use this for **whole robots** (quadrupeds, wheeled bases, humanoids with built-in locomotion). This is the pattern used by Go2, K1, and M20.

Create your module at `dimos/robot/yourvendor/yourmodel/connection.py`:

```python
"""YourRobot connection module."""

from threading import Event, Thread
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.perception import Camera


class YourRobotConfig(ModuleConfig):
    """Configuration for your robot."""

    ip: str = "192.168.1.100"
    camera_port: int = 8554


class YourRobotConnection(Module, Camera):
    """Connection module for YourRobot.

    Handles:
    - Camera streaming (RTSP/ROS/WebSocket)
    - Velocity commands (cmd_vel -> robot SDK)
    - Odometry publishing
    - Agent-callable skills
    """

    config: YourRobotConfig

    # -- Output streams (sensor data FROM the robot) --
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]

    # -- Input streams (commands TO the robot) --
    cmd_vel: In[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._sdk = None
        self._stop = Event()

    @rpc
    def start(self) -> None:
        """Connect to robot and start streaming."""
        super().start()
        self._stop.clear()

        # 1. Connect to the robot
        from your_robot_sdk import RobotClient

        self._sdk = RobotClient(self.config.ip)
        self._sdk.connect()

        # 2. Start camera thread
        self._camera_thread = Thread(
            target=self._stream_camera, daemon=True
        )
        self._camera_thread.start()

        # 3. Subscribe to cmd_vel input
        self._disposables.add(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))

    @rpc
    def stop(self) -> None:
        """Disconnect and clean up."""
        self._stop.set()
        if self._sdk:
            self._sdk.stop()
            self._sdk.disconnect()
            self._sdk = None
        if getattr(self, "_camera_thread", None) and self._camera_thread.is_alive():
            self._camera_thread.join(timeout=1.0)
        super().stop()

    def _stream_camera(self) -> None:
        """Background thread: receive camera frames and publish."""
        import av

        container = av.open(
            f"rtsp://{self.config.ip}:{self.config.camera_port}/video1"
        )
        for frame in container.decode(video=0):
            if self._stop.is_set():
                break
            img = frame.to_ndarray(format='rgb24')
            self.color_image.publish(Image.from_numpy(
                img,
                format=ImageFormat.RGB,
                frame_id="camera_optical",
            ))

    def _on_cmd_vel(self, twist: Twist) -> None:
        """Forward velocity commands to the robot."""
        if self._sdk:
            self._sdk.move(
                linear_x=twist.linear.x,
                linear_y=twist.linear.y,
                angular_z=twist.angular.z,
            )

    # -- Agent skills (callable by LLM) --

    @skill
    def walk(
        self, x: float, y: float = 0.0, yaw: float = 0.0
    ) -> str:
        """Walk in the specified direction.

        Args:
            x: Forward speed in m/s (positive = forward).
            y: Lateral speed in m/s (positive = left).
            yaw: Rotation speed in rad/s (positive = counter-clockwise).
        """
        self._on_cmd_vel(Twist(
            linear=Vector3(x, y, 0.0),
            angular=Vector3(0.0, 0.0, yaw),
        ))
        return f"Walking: x={x}, y={y}, yaw={yaw}"

    @skill
    def stop_moving(self) -> str:
        """Stop all motion."""
        self._on_cmd_vel(Twist())
        return "Stopped."
```

> **If your robot uses ROS 2 for sensors:** You don't need the `_stream_camera` thread above. Instead, declare your output streams as usual (`color_image: Out[Image]`, `odom: Out[PoseStamped]`, etc.) but skip the manual streaming code. In your blueprint, wire them with `ROSTransport`:
>
> ```python
> from dimos.core.transport import ROSTransport
>
> yourmodel = autoconnect(
>     YourRobotConnection.blueprint(ip="192.168.1.100"),
> ).transports({
>     ("color_image", Image): ROSTransport("/camera/image/compressed", Image),
>     ("odom", PoseStamped): ROSTransport("/odom", PoseStamped),
> })
> ```
>
> DimOS handles the ROS 2 subscription and message conversion automatically. Downstream modules see DimOS message types; the ROS details stay at the transport boundary. See the Go2 ROS blueprint and B1 connection for real examples.

### Pattern B: Hardware Adapter

Use this for **components** (manipulator arms, grippers, drive trains) that plug into the `ControlCoordinator`. This is the pattern used by xArm, Piper, and R1 Pro arms/chassis.

Create your adapter at `dimos/hardware/manipulators/yourarm/adapter.py`:

```python
"""YourArm hardware adapter - implements ManipulatorAdapter protocol."""

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorInfo,
)


class YourArmAdapter:
    """Implements ManipulatorAdapter via duck typing.

    No inheritance needed - just match the method signatures in spec.py.
    """

    def __init__(self, address: str, dof: int = 6) -> None:
        self._address = address
        self._dof = dof
        self._sdk = None
        self._mode = ControlMode.POSITION

    # -- Connection --

    def connect(self) -> bool:
        """Connect to hardware. Import SDK lazily."""
        try:
            from yourarm_sdk import YourArmSDK
            self._sdk = YourArmSDK(self._address)
            self._sdk.connect()
            return self._sdk.is_alive()
        except ImportError:
            print("ERROR: yourarm-sdk not installed")
            return False

    def disconnect(self) -> None:
        if self._sdk:
            self._sdk.disconnect()
            self._sdk = None

    def is_connected(self) -> bool:
        return self._sdk is not None and self._sdk.is_alive()

    # -- Info --

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="YourVendor", model="YourModel", dof=self._dof,
        )

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        return JointLimits(
            position_lower=[-math.pi] * self._dof,
            position_upper=[math.pi] * self._dof,
            velocity_max=[math.pi] * self._dof,
        )

    # -- Control mode --

    def set_control_mode(self, mode: ControlMode) -> bool:
        self._mode = mode
        # Call the vendor SDK here if mode switching is required.
        return True

    def get_control_mode(self) -> ControlMode:
        return self._mode

    # -- Read state (all values in SI units: radians, rad/s, Nm) --

    def read_joint_positions(self) -> list[float]:
        raw = self._sdk.get_joint_positions()
        return [math.radians(p) for p in raw[:self._dof]]  # if SDK uses degrees

    def read_joint_velocities(self) -> list[float]:
        return [0.0] * self._dof  # Return zeros if SDK doesn't provide this

    def read_joint_efforts(self) -> list[float]:
        return [0.0] * self._dof  # Return zeros if SDK doesn't provide this

    def read_state(self) -> dict[str, int]:
        return {"mode": 0, "state": 0}

    def read_error(self) -> tuple[int, str]:
        return (0, "")

    # -- Write commands --

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        sdk_positions = [math.degrees(p) for p in positions]
        return self._sdk.set_joint_positions(sdk_positions)

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        return False  # Return False if the SDK cannot stream joint velocities

    def write_stop(self) -> bool:
        return self._sdk.emergency_stop()

    def write_enable(self, enable: bool) -> bool:
        return self._sdk.enable_motors(enable)

    def read_enabled(self) -> bool:
        return self._sdk.motors_enabled() if self._sdk else False

    def write_clear_errors(self) -> bool:
        return self._sdk.clear_errors() if self._sdk else False

    # -- Optional (return None/False if not supported) --

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(
        self, pose: dict[str, float], velocity: float = 1.0
    ) -> bool:
        return False

    def read_gripper_position(self) -> float | None:
        return None

    def write_gripper_position(self, position: float) -> bool:
        return False

    def read_force_torque(self) -> list[float] | None:
        return None


# Registry hook - required for auto-discovery
def register(registry: "AdapterRegistry") -> None:
    registry.register("yourarm", YourArmAdapter)
```

### Which pattern should I use?

| Your robot is... | Use | Examples |
|-------------------|-----|----------|
| A whole robot with built-in locomotion | **Connection Module** | Go2, K1, M20 |
| A manipulator arm | **Hardware Adapter** (ManipulatorAdapter) | xArm, Piper, R1 Pro arms |
| A mobile base / drive train | **Hardware Adapter** (TwistBaseAdapter) | R1 Pro chassis |
| A humanoid with arms + legs | **Both** - adapters for arms/chassis, assembled by ControlCoordinator | R1 Pro |

> **Why two patterns?** Connection modules manage the whole robot lifecycle (connect, stream sensors, accept commands). Hardware adapters are simpler - they just implement a protocol interface and let the ControlCoordinator handle the 100Hz control loop, task arbitration, and multi-hardware coordination. If your robot has independently controllable subsystems (left arm, right arm, chassis), use adapters.

---

## Phase 5: Create Blueprints

Blueprints are declarative recipes that assemble modules into a runnable system. Every robot needs at least a "basic" blueprint.

### 5a. Create your robot directory

```
dimos/robot/
├── unitree/          # Unitree robots
├── booster/          # Booster robots
├── deep_robotics/    # Deep Robotics robots
└── yourvendor/       # Your vendor
    └── yourmodel/
        ├── __init__.py          # (can be empty)
        ├── connection.py        # Your connection module (Phase 4)
        └── blueprints/
            ├── __init__.py      # (can be empty)
            └── basic/
                └── yourmodel_basic.py
```

### 5b. Write a basic blueprint

For a **Connection Module** robot:

```python
"""Basic blueprint for YourRobot - camera + movement + visualization."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.yourvendor.yourmodel.connection import YourRobotConnection
from dimos.visualization.rerun.bridge import RerunBridgeModule

yourmodel_basic = autoconnect(
    YourRobotConnection.blueprint(ip="192.168.1.100"),
    RerunBridgeModule.blueprint(),
).global_config(
    n_workers=4,
    robot_model="yourmodel",
)
```

For a **Hardware Adapter** robot (e.g., arm with coordinator):

```python
"""Blueprint for YourArm with ControlCoordinator."""

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import JointState

arm_joints = make_joints("arm", 6)

coordinator_yourarm = ControlCoordinator.blueprint(
    tick_rate=100.0,
    publish_joint_state=True,
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=arm_joints,
            adapter_type="yourarm",          # Must match registry name
            address="192.168.1.100",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_arm",
            type="trajectory",
            joint_names=arm_joints,
            priority=10,
        ),
    ],
).transports({
    ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
})
```

### 5c. Add more blueprint variants

Once basic works, layer on capabilities:

```python
# Smart blueprint - adds navigation
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner

yourmodel_smart = autoconnect(
    yourmodel_basic,
    VoxelGridMapper.blueprint(),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
).global_config(n_workers=4, robot_model="yourmodel")


# Agentic blueprint - adds MCP tools + LLM client
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer

YOURMODEL_SYSTEM_PROMPT = """
You are controlling YourRobot. Use the available tools safely and briefly
explain physical actions before executing them.
"""

yourmodel_agentic = autoconnect(
    yourmodel_smart,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=YOURMODEL_SYSTEM_PROMPT),
    # Add robot-specific skill containers here if your connection module
    # does not already expose all required @skill methods.
).global_config(n_workers=8, robot_model="yourmodel")
```

### 5d. Register your blueprints

```bash
# This auto-generates dimos/robot/all_blueprints.py
uv run pytest dimos/robot/test_all_blueprints_generation.py

# Now you can run your robot:
dimos run yourmodel-basic
dimos run yourmodel-agentic
```

---

## Phase 6: Test on Hardware

Write simple test scripts to validate each layer before assembling the full system. The R1 Pro integration used this approach - five sequential tests, each building on the previous:

| Test | What it validates | Safe? |
|------|-------------------|-------|
| 1. Discovery | Can you see the robot's API/topics? | Yes (read-only) |
| 2. Sensor read | Can you receive camera/joint/odometry data? | Yes (read-only) |
| 3. Small motion | Does a tiny velocity command work? | Mostly safe |
| 4. Full motion | Do complex commands work correctly? | Use caution |
| 5. DimOS integration | Does the full module/adapter work through DimOS? | Use caution |

**Always test read-only operations first.**

Example test script:
```python
"""Test 1: Verify connectivity and sensor data."""

def main() -> bool:
    from your_robot_sdk import RobotClient

    robot = RobotClient("192.168.1.100")
    assert robot.connect(), "Failed to connect"

    # Read-only: check we can get data
    status = robot.get_status()
    print(f"Robot status: {status}")

    camera_frame = robot.get_camera_frame()
    print(f"Camera frame: {camera_frame.shape if camera_frame is not None else 'None'}")

    robot.disconnect()
    return True

if __name__ == "__main__":
    success = main()
    print(f"\n{'PASS' if success else 'FAIL'}")
```

---

## External Sensors: Filling the Gaps

If your robot doesn't provide LiDAR, odometry, or IMU, you can add external sensors. DimOS has built-in support for:

### Livox MID-360 (LiDAR + IMU)

The MID-360 is the most common external sensor we use. DimOS has a native C++ driver and FAST-LIO2 config ready to go.

Use either the raw MID-360 driver or the integrated FAST-LIO2 module. Do not add both to the same blueprint unless you intentionally want two independent Livox consumers.

Raw LiDAR + IMU:

```python
from dimos.hardware.sensors.lidar.livox.module import Mid360

# Add to your blueprint:
yourmodel_with_lidar = autoconnect(
    yourmodel_basic,
    Mid360.blueprint(host_ip="192.168.1.5", lidar_ip="192.168.1.155"),
    VoxelGridMapper.blueprint(),
    CostMapper.blueprint(),
)
```

FAST-LIO2 LiDAR-inertial SLAM:

```python
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2

yourmodel_with_slam = autoconnect(
    yourmodel_basic,
    FastLio2.blueprint(host_ip="192.168.1.5", lidar_ip="192.168.1.155"),
    VoxelGridMapper.blueprint(),
    CostMapper.blueprint(),
)
```

**What this gives you:**
- `Mid360`: `lidar: Out[PointCloud2]` at ~10Hz and `imu: Out[Imu]` at ~200Hz
- `FastLio2`: `lidar: Out[PointCloud2]`, `odometry: Out[nav_msgs.Odometry]`, and optional `global_map: Out[PointCloud2]`

> **Navigation note**: The current native navigation stack consumes `odom: PoseStamped`. FAST-LIO2 publishes full `nav_msgs.Odometry`, so add a small conversion module or publish a matching `odom: Out[PoseStamped]` from your robot connection if you want to feed `ReplanningAStarPlanner` directly.

### Intel RealSense (Depth camera)

For robots that need depth perception but don't have a depth camera. Used on arm setups for eye-in-hand grasping.

```python
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera

# Add to your blueprint:
yourmodel_with_realsense = autoconnect(
    yourmodel_basic,
    RealSenseCamera.blueprint(
        width=848,
        height=480,
        fps=15,
        enable_depth=True,
        base_frame_id="base_link",      # Parent frame in your TF tree
    ),
)
```

**What this gives you:**
- `Image` (color) at configured FPS
- `Image` (depth, aligned to color)
- `CameraInfo` for both color and depth
- Optional `PointCloud2` (set `enable_pointcloud=True`)

> **Real example (xArm grasping)**: The xArm manipulation blueprint mounts a RealSense on the end-effector with a calibrated transform for eye-in-hand grasping. See `dimos/manipulation/blueprints.py`.

### ZED Camera (Stereo depth + tracking)

For robots that need stereo depth and built-in visual-inertial tracking. Used on the Unitree G1 for chest-mounted perception.

```python
from dimos.hardware.sensors.camera.zed.camera import ZEDCamera

# Add to your blueprint:
yourmodel_with_zed = autoconnect(
    yourmodel_basic,
    ZEDCamera.blueprint(
        width=1280,
        height=720,
        fps=15,
        depth_mode="NEURAL",         # Or "PERFORMANCE" for speed
        enable_tracking=True,         # Visual-inertial odometry
        enable_imu_fusion=True,
        base_frame_id="base_link",
    ),
)
```

**What this gives you:**
- `Image` (color) at configured FPS
- `Image` (depth)
- `CameraInfo` for both color and depth
- Built-in visual-inertial odometry (from IMU + stereo fusion)
- Optional positional tracking with area memory

### Mounting considerations

When adding external sensors to a robot:
1. **Mount rigidly** - vibration causes noisy data
2. **Know the transform** - you need the sensor's position/orientation relative to the robot's base frame (the TF transform)
3. **Power** - most sensors need USB or ethernet + separate power
4. **Bandwidth** - LiDAR + cameras can saturate a USB hub. Use separate USB controllers or ethernet

---

## Form Factor Reference

While the integration flow is the same, different robot types have different emphasis areas:

### Quadrupeds (Go2, M20)

**What's unique:**
- Gait switching (walk, trot, stair-climb) - expose as agent skills
- Motion states (stand, sit, lie down) - expose as agent skills
- Usually have SDK-based velocity control, not ROS
- Often need heartbeat/keepalive messages

**Key interfaces:** `cmd_vel` (Twist), camera, odometry, LiDAR

### Humanoids (G1, R1 Pro)

**What's unique:**
- Multiple subsystems: arms, chassis/legs, torso, grippers
- Need ControlCoordinator to orchestrate all subsystems at 100Hz
- Joint-level control for arms (ManipulatorAdapter)
- Velocity control for locomotion (TwistBaseAdapter)
- May need URDF for arm motion planning

**Key interfaces:** Joint states (per arm), `cmd_vel` (chassis), multiple cameras

### Wheeled Bases (delivery robots, AMRs)

**What's unique:**
- Simplest form factor - just `cmd_vel` + sensors
- Navigation is the main capability
- Often have ROS 2 built in

**Key interfaces:** `cmd_vel` (Twist), LiDAR, odometry, cameras

### Manipulator Arms (xArm, Piper)

**What's unique:**
- Use Hardware Adapter pattern (not Connection Module)
- Need URDF for motion planning (Drake)
- Joint-level position/velocity control
- Gripper control
- Unit conversion is critical (degrees vs radians, mm vs m)

**Key interfaces:** Joint positions, joint velocities, gripper position, force/torque

### Drones

**What's unique:**
- MAVLink protocol (standard across many drones)
- 3D velocity commands (including altitude)
- GPS-based localization (outdoors)
- Flight modes and arming sequences

**Key interfaces:** 3D velocity, GPS position, altitude, battery

---

## File Checklist

When you're done, you should have created these files:

### For a Connection Module robot:

- [ ] `dimos/robot/yourvendor/yourmodel/connection.py` - Connection module
- [ ] `dimos/robot/yourvendor/yourmodel/blueprints/basic/yourmodel_basic.py` - Basic blueprint
- [ ] `pyproject.toml` - Add vendor SDK to optional dependencies (if needed)
- [ ] `scripts/yourmodel_test/` - Test scripts (recommended)

### For a Hardware Adapter robot:

- [ ] `dimos/hardware/manipulators/yourarm/adapter.py` - Adapter + `register()` hook
- [ ] `dimos/hardware/manipulators/yourarm/__init__.py` - Package init
- [ ] `dimos/robot/yourvendor/yourmodel/blueprints.py` - Coordinator blueprint
- [ ] `pyproject.toml` - Add vendor SDK to optional dependencies (if needed)

### For a humanoid (adapters + coordinator):

- [ ] `dimos/hardware/manipulators/yourmodel/adapter.py` - Arm adapter
- [ ] `dimos/hardware/drive_trains/yourmodel/adapter.py` - Chassis adapter
- [ ] `dimos/robot/yourvendor/yourmodel/blueprints.py` - Coordinator + arm + chassis blueprints
- [ ] `scripts/yourmodel_test/` - Test scripts (highly recommended)

### Verification:

- [ ] `uv run pytest dimos/robot/test_all_blueprints_generation.py` passes
- [ ] `dimos run yourmodel-basic` starts and shows camera feed
- [ ] Robot responds to movement commands

---

## Lessons from Real Integrations

These are hard-won lessons from integrating 10+ robots with DimOS.

### Network issues will eat most of your time

Every single integration spent significant time on connectivity. Budget for it.

- **Always use `--no-daemon`** when running `ros2 topic list` - the ROS 2 daemon is unreliable for cross-machine discovery
- **Pin your DDS middleware** - if the robot uses FastDDS, use FastDDS on your laptop too. Don't assume CycloneDDS will interop
- **Create a FastDDS XML profile** that binds to the correct network interface. Laptops with WiFi + ethernet + VPN confuse DDS multicast
- **Check `ROS_LOCALHOST_ONLY`** on the robot. Vendors sometimes ship with this set to 1

### Test incrementally

Don't try to build the full agentic blueprint on day one. Follow the phases:
1. Get basic connectivity working
2. Read one sensor (camera is easiest)
3. Send one command (make it move)
4. Then wrap in DimOS

### Units will bite you

DimOS uses SI units everywhere. Your robot's SDK probably doesn't.

| Quantity | DimOS | Common SDK units |
|----------|-------|------------------|
| Angles | radians | degrees |
| Distance | meters | millimeters |
| Velocity | m/s | mm/s, or normalized [-1, 1] |
| Angular velocity | rad/s | deg/s |
| Force | Newtons | grams, kg |
| Torque | Nm | mNm |

Convert at the adapter boundary. Never let non-SI units leak into DimOS.

### Document your "gates"

Many robots have hidden prerequisites for accepting commands. The R1 Pro had three gates for chassis control. The M20 required a specific gait mode for navigation. Document these in a README in your test scripts directory - the next person will thank you.

### Sensor dropout is a real problem

When running cameras + LiDAR + control loop simultaneously, sensor streams can drop. Common causes:
- OS socket buffer overflow (increase with `sysctl net.core.rmem_max`)
- DDS receive thread starvation under high-frequency control traffic
- GIL contention in Python when decoding large frames on the spin thread

Solutions that have worked:
- Move `bytes(msg.data)` copy off the spin thread into a dedicated worker
- Use separate `rclpy.Context` for sensor subscriptions (isolated DDS participant)
- Use `queue.Queue(maxsize=1)` for latest-frame semantics

---

## Quick Reference: Existing Integrations

| Robot | Type | Location | Connection | Good reference for... |
|-------|------|----------|------------|----------------------|
| Unitree Go2 | Quadruped | `dimos/robot/unitree/go2/` | WebRTC SDK | Connection Module pattern, skills |
| Unitree G1 | Humanoid | `dimos/robot/unitree/g1/` | WebRTC + SDK | Humanoid with sim support |
| Booster K1 | Humanoid | `dimos/robot/booster/k1/` (branch: `miguel/booster`) | RPC/WebSocket | Simple Connection Module |
| Deep Robotics M20 | Quadruped | `dimos/robot/deep_robotics/m20/` (branch: `afik/feat/m20`) | UDP + ROS 2 | Dual-mode connection, protocol parsing |
| Galaxea R1 Pro | Humanoid | `dimos/hardware/*/r1pro/` (branch) | ROS 2 | Adapter pattern, multi-subsystem, detailed troubleshooting |
| UFactory xArm | Arm | `dimos/hardware/manipulators/xarm/` | TCP/IP SDK | Arm adapter, motion planning |
| AgileX Piper | Arm | `dimos/hardware/manipulators/piper/` | CAN bus | Arm adapter, gripper |

> The Go2 on `main` is the most complete reference. The R1 Pro README at `scripts/r1pro_test/README.md` (on branch `task/mustafa/r1pro-dual-arm-testing`) is the best troubleshooting reference.
