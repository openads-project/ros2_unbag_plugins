# ros2 unbag

This repository provides a dockerized wrapper around [ros2_unbag](https://github.com/ika-rwth-aachen/ros2_unbag), a high-performance ROS 2 CLI/GUI tool for exporting topics from `.db3` or `.mcap` bag files.

## Features

- Installs `ros2_unbag` via PyPI
- Runs in a ROS 2 Docker environment using [docker-ros](https://github.com/ika-rwth-aachen/docker-ros)
- Supports custom export routines in [`custom/routines/`](custom/routines) and custom processors in [`custom/processors/`](custom/processors)

## Usage

1. Modify the `docker-compose.yml` to mount your desired folder.

2. Run the container via docker compose:

   ```bash
   docker-compose up
   ```

3. Use `ros2 unbag` as normal (CLI or GUI).
   Example:

   ```bash
   ros2 unbag my_bag.mcap --export /topic:format
   ```

## Custom Routines & Processors

Place your Python-based export routines in `custom/routines/` and your custom processors in `custom/processors/`. They are installed automatically during the image build via `ros2 unbag --install-routine` and `ros2 unbag --install-processor`.
