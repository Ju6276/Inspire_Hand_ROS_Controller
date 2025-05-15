# Inspire Dexterous Hand Controller - ROS Noetic

This package provides a ROS Noetic-compatible interface for controlling the **Inspire Dexterous Hand**, built on top of the official `inspire_hand` package. Unlike the original service-based approach, this version uses **ROS topics** for all control and feedback, enabling non-blocking, high-frequency communication and real-time responsiveness.

---

## ✨ Features

- 🛠️ Full control of the Inspire hand via ROS topics (angle, speed, and force)
- 📡 Real-time feedback including tactile sensors, joint angles, and force/current values
- ⚡ More efficient than service-based architecture
- 🔄 Multiple launch modes for flexible deployment
- ✅ Compatible with ROS Noetic & Ubuntu 20.04

---

## 🚀 Launch Modes

Use the following launch modes to start the system:

| Mode | Description                                                        |
|------|--------------------------------------------------------------------|
| 0    | Launches `hand_modbus_control` (basic Modbus communication node)  |
| 1    | Launches `hand_modbus_control` + `hand_control_client_modbus`     |
| 2    | Launches `hand_modbus_control` + `inspire_hand_modbus_topic.py`   |
| 3    | Launches `hand_modbus_control` with both service and topic APIs   |

```bash
roslaunch inspire_hand_modbus control.launch mode:=<mode_id>
```

---

## 🛰️ Topic Interface

### ✅ Control Topics (Subscribers)

| Topic Name        | Message Type                      | Description                           |
|------------------|-----------------------------------|---------------------------------------|
| `/set_angle_data`| `inspire_hand_modbus/set_angle_1` | Set joint angles (encoder units)      |
| `/set_speed_data`| `inspire_hand_modbus/set_speed_1` | Set joint movement speeds             |
| `/set_force_data`| `inspire_hand_modbus/set_force_1` | Set joint force/current values        |

#### 🔧 Example: Set Angles

```bash
rostopic pub -1 /set_angle_data inspire_hand_modbus/set_angle_1 "{finger_ids: [1, 2, 3, 4, 5, 6], angles: [1000, 1000, 1000, 1000, 1000, 1000]}"
```

#### 🔧 Example: Set Speed

```bash
rostopic pub -1 /set_speed_data inspire_hand_modbus/set_speed_1 "{finger_ids: [1, 2, 3, 4, 5, 6], speeds: [1000, 1000, 1000, 1000, 1000, 1000]}"
```

#### 🔧 Example: Set Force

```bash
rostopic pub -1 /set_force_data inspire_hand_modbus/set_force_1 "{finger_ids: [1, 2, 3, 4, 5, 6], forces: [1000, 1000, 1000, 1000, 1000, 1000]}"
```

---

### 📤 Feedback Topics (Publishers)

| Topic Name      | Message Type              | Description                           |
|----------------|---------------------------|---------------------------------------|
| `/touch_data`  | `std_msgs/Int32MultiArray`| Binary tactile data from each finger  |
| `/force_data`  | `std_msgs/Int32MultiArray`| Measured joint force values           |
| `/angle_data`  | `std_msgs/Int32MultiArray`| Current joint angles (encoder values) |

#### 📡 Example: View Feedback

```bash
rostopic echo /touch_data
rostopic echo /force_data
rostopic echo /angle_data
```

---

## 🧾 Custom Message Definitions

### `set_angle_1.msg`

```msg
int32[] finger_ids  
int32[] angles
```

### `set_speed_1.msg`

```msg
int32[] finger_ids  
int32[] speeds
```

### `set_force_1.msg`

```msg
int32[] finger_ids  
int32[] forces
```

---

## 📂 Directory Structure

```
inspire_hand_modbus/
├── launch/
│   └── control.launch
├── msg/
│   ├── set_angle_1.msg
│   ├── set_speed_1.msg
│   └── set_force_1.msg
├── scripts/
│   └── inspire_hand_modbus_topic.py
├── src/
│   ├── hand_modbus_control.cpp
│   └── hand_control_client_modbus.cpp
├── CMakeLists.txt
└── package.xml
```

---

## 🔧 Dependencies

Ensure the following dependencies are installed:

- ROS Noetic
- `libmodbus-dev` (for Modbus RTU communication)
- Standard ROS packages (`rospy`, `std_msgs`, etc.)
- Python 3
- Official Inspire Hand Modbus driver

---

## 📞 Contact

For questions, bug reports, or contributions, feel free to open an issue or contact the maintainer via GitHub.

---

## 📜 License

This project is distributed under the MIT License.
