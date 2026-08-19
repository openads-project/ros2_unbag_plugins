# ros2_unbag_plugins

<p align="center">
  <a href="https://openads-project.github.io"><img src="https://img.shields.io/badge/OpenADS-45ccc6"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/openads-project/ros2_unbag_plugins/releases/latest"><img src="https://img.shields.io/github/v/release/openads-project/ros2_unbag_plugins"/></a>
  <a href="https://github.com/openads-project/ros2_unbag_plugins/blob/main/LICENSE"><img src="https://img.shields.io/github/license/openads-project/ros2_unbag_plugins"/></a>
  <br>
  <a href="https://github.com/openads-project/ros2_unbag_plugins/actions/workflows/docker-ros.yml"><img src="https://github.com/openads-project/ros2_unbag_plugins/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/ros2_unbag_plugins/actions/workflows/compose-oci.yml"><img src="https://github.com/openads-project/ros2_unbag_plugins/actions/workflows/compose-oci.yml/badge.svg"/></a>
  <a href="https://openads-project.github.io/ros2_unbag_plugins"><img src="https://github.com/openads-project/ros2_unbag_plugins/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/ros2_unbag_plugins/actions/workflows/consistency.yml"><img src="https://github.com/openads-project/ros2_unbag_plugins/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

**Plugins for ros2_unbag topic export with the OpenADS interfaces**

<img src="https://raw.githubusercontent.com/ika-rwth-aachen/ros2_unbag/refs/heads/main/qt_resources/assets/badge.svg" align="right" height="120" alt="ros2_unbag_plugins logo">
<p align="justify">
This repository extends <a href="https://github.com/ika-rwth-aachen/ros2_unbag">ros2_unbag</a> — a high-performance ROS 2 CLI/GUI for exporting topics from <code>.db3</code> and <code>.mcap</code> bag files — with routines and processors for OpenADS data. It supports exporting <code>perception_msgs/msg/ObjectList</code> messages in the HEXAMOTION format and Cloudini-compressed point clouds, as well as processors for tasks such as object-list transformations, Cloudini point-cloud decompression, and point-field colormaps. The repository also produces a Docker image with <code>ros2_unbag</code> and the required OpenADS ROS 2 interfaces preinstalled; alternatively, the plugins can be installed into an existing <code>ros2_unbag</code> environment. For natively supported topics, see the upstream <a href="https://github.com/ika-rwth-aachen/ros2_unbag/blob/main/docs/EXPORT_ROUTINES.md">Export Routines</a> documentation; see <a href="#-documentation">Documentation</a> for the OpenADS-specific additions.
</p>

<p align="center">
  <strong>🚀 <a href="#-quick-start">Quick Start</a></strong> • <strong>💻 <a href="#-development">Development</a></strong> • <strong>📝 <a href="#-documentation">Documentation</a></strong>
</p>


> [!IMPORTANT]
> This repository is part of [***OpenADS***](https://github.com/openads-project), the *Open Automated Driving Systems* project. *OpenADS* and its modules have been initiated and are currently being maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/de/).


## 🚀 Quick Start

### Dockerized Setup
1. Launch the [`docker/compose/docker-compose.yml`](docker/compose/docker-compose.yml) setup. This starts the image with the preinstalled OpenADS interfaces, routines, and processors, mounting a local bag directory into the container:

    ```bash
    cd docker/compose
    xhost +local: # allow GUI forwarding from containers
    BAG_DIR=/path/to/your/bags docker compose up
    ```

2. Use `ros2 unbag` as normal in the GUI, or attach to the running container to use the CLI:

    ```bash
    docker exec -it ros2-unbag bash
    ros2 unbag my_bag.mcap --export /topic:format
    ```

3. Stop the demo, remove its containers, and revoke GUI access:

    ```bash
    docker compose down
    xhost -local:
    ```
### Direct Installation
Docker is optional. If `ros2_unbag` and the required OpenADS ROS 2 interfaces are already installed in your environment, install the plugins directly from this checkout:

```bash
for routine in plugins/routines/*.py; do ros2 unbag --install-routine "$routine"; done
for processor in plugins/processors/*.py; do ros2 unbag --install-processor "$processor"; done
```

## 💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/openads-project/ros2_unbag_plugins.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/openads-project/openads-dev-environment) submodule containing development environment configuration.
    ```bash
    cd ros2_unbag_plugins
    git submodule update --init --recursive
    ```
1. Open the repository in [Visual Studio Code](https://code.visualstudio.com).
    ```bash
    code .
    ```
1. Install the recommended VS Code extensions.
    > *Ctrl+Shift+P / Extensions: Show Recommended Extensions / Install Workspace Recommended Extensions (Cloud Download Icon)*
1. Reopen the repository in a [Dev Container](https://code.visualstudio.com/docs/devcontainers/containers).
    > *Ctrl+Shift+P / Dev Containers: Rebuild and Reopen in Container*

### Add Routines & Processors

Place new Python-based export routines in [`plugins/routines/`](plugins/routines) and new processors in [`plugins/processors/`](plugins/processors). They are installed automatically during the image build via `ros2 unbag --install-routine` and `ros2 unbag --install-processor`, see [`docker/custom.sh`](docker/custom.sh).


## 📝 Documentation

Natively supported topics are documented in the upstream [Export Routines](https://github.com/ika-rwth-aachen/ros2_unbag/blob/main/docs/EXPORT_ROUTINES.md) reference. The OpenADS-specific routines and processors added by this repository are documented in [`docs/IMPLEMENTATION.md`](docs/IMPLEMENTATION.md).

## ⚖️ Licensing

The source code in this repository is licensed under Apache-2.0, see [LICENSE](LICENSE). Container images provided by this repository may contain third-party software shipped with their own license terms.

## 🙏 Acknowledgements

Development and maintenance of this repository are supported by the following projects. We acknowledge the funding of the respective institutions.

| Project | Funding Institution | Grant Number |
| --- | --- | --- |
| 4-CAD | 🇩🇪 Deutsche Forschungsgemeinschaft (DFG, German Research Foundation) | 503852364 |

<p>
  <img src="https://www.fz-juelich.de/en/jsc/images/newsletter/dfg-logo/@@images/image-400-5f654fc8bb836a0c010a9410208a0d9c.png" height=70>
</p>
