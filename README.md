# Lucy Control Panel Package
*REST API backend for managing robot joints via URDF files in ROS 2*

---

## 📌 Overview

The Lucy Control Panel Package is a ROS 2 package that provides a REST API backend for the [Lucy Control Panel](https://github.com/Sentience-Robotics/lucy_control_panel) project. This package enables remote management and control of robot joints through a web-based interface by exposing robot capabilities via RESTful endpoints.

The package integrates with ROS 2's robot description framework, parsing URDF (Unified Robot Description Format) files to dynamically generate API endpoints for each controllable joint. This allows for real-time monitoring and control of robot actuators through HTTP requests, making it easy to integrate with web applications, mobile apps, or other external systems.

---

## 🚀 Features

- **REST API Server**: Embedded HTTP server providing RESTful endpoints for robot control
- **URDF Integration**: Automatic parsing of robot URDF files to discover available joints and capabilities
- **Joint Control**: Real-time position, velocity, and effort control for robot joints
- **Status Monitoring**: Live feedback on joint states, positions, and health status
- **ROS 2 Native**: Full integration with ROS 2 ecosystem including topics, services, and parameters
- **Web Interface Compatible**: Designed to work seamlessly with the Lucy Control Panel frontend
- **Configurable**: Flexible configuration options for different robot platforms and setups

---

## 🛠️ Installation & Usage

### Prerequisites

- ROS 2 (Humble or later)
- Python 3.8+
- Valid URDF file for your robot

### Installation

This setup uses the [lucy_devtools](https://github.com/Sentience-Robotics/lucy_devtools) repository to build the package.

```bash
# Clone the repository into your ROS 2 workspace
cd ~/lucy_/src
git clone https://github.com/Sentience-Robotics/lucy_control_panel_package.git

# Enter into the devtools ROS 2 workspace
Env launch
cd ~/lucy_ros2_ws

# Build the package
Make -b

# Source the workspace
source install/setup.bash
```

### Quick Start

```bash
# Launch the REST API server
ros2 run lucy_control_panel_package lucy_api_server

# In another terminal, launch the node with the following commands:
ros2 lifecycle get /lucy_api_server && \
ros2 lifecycle set /lucy_api_server configure && \
ros2 lifecycle set /lucy_api_server activate

# You can check the status of the node with:
ros2 lifecycle get /lucy_api_server
ros2 param get /lucy_api_server server_port
ros2 param get /lucy_api_server server_host

# The API will be available at http://localhost:8080
# Use with the [Lucy Control Panel](https://github.com/Sentience-Robotics/lucy_control_panel) frontend for full functionality
```

---

## 📖 Documentation

For more details on API endpoints, configuration options, and integration guides, visit the [Sentience Robotics documentation](https://docs.sentience-robotics.fr).

---

## 🔗 Related Projects

- **[Lucy Control Panel](https://github.com/Sentience-Robotics/lucy_control_panel)** - Frontend web interface for robot control
- **[InMoov Project](https://inmoov.fr/)** - Open-source humanoid robot platform

---

## 📜 Code of Conduct

We value the participation of each member of our community and are committed to ensuring that every interaction is respectful and productive. To foster a positive environment, we ask you to read and adhere to our [Code of Conduct](CODE_OF_CONDUCT.md).

By participating in this project, you agree to uphold this code in all your interactions, both online and offline. Let's work together to maintain a welcoming and inclusive community for everyone.

If you encounter any issues or have questions regarding the Code of Conduct, please contact us at [contact@sentience-robotics.fr](mailto:contact@sentience-robotics.fr).

Thank you for being a part of our community!

---

## 🤝 Contributing

To find out more on how you can contribute to the project, please check our [CONTRIBUTING.md](CONTRIBUTING.md)

---

## 📜 License

This project is licensed under the **GNU GPL V3 License**. See the [LICENSE](LICENSE) file for details.

---

## 🙌 Acknowledgments

- 🎉 [InMoov Project](https://inmoov.fr/) – Original design by Gael Langevin
- 🎉 **ROS 2 Community** – For the excellent robotics framework
- 🎉 **All contributors** to the InMoov and ROS communities

---

## 📬 Contact

- 📧 Email: [contact@sentience-robotics.fr](mailto:contact@sentience-robotics.fr)
- 🌍 GitHub Organization: [Sentience Robotics](https://github.com/sentience-robotics)

---

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-34aec5?logo=ros)](https://docs.ros.org/en/humble/)
[![Contributor Covenant](https://img.shields.io/badge/Contributor%20Covenant-2.0-4baaaa.svg)](CODE_OF_CONDUCT.md)
