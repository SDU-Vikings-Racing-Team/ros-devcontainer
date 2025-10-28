# ROS 2 Devcontainer

This is the official development environment for ROS 2 Humble used by the SDU Vikings Driverless Team. It runs in a Docker container via VS Code to ensure a consistent setup across Linux, macOS, and Windows. The purpose of this standardization is to eliminate the classic “works on my machine” problem.

## What You Need

Before you get started, you will need to install a few things on your computer.

### Docker

Docker runs the container that has all our ROS 2 tools and dependencies.

**Linux**:
```bash
# Install Docker Engine
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add your user to docker group (so you do not need sudo)
sudo usermod -aG docker $USER

# Log out and back in, then verify
docker --version
```

Or follow the official guide: https://docs.docker.com/engine/install/

**macOS or Windows**:
Download and install Docker Desktop:
- macOS: https://docs.docker.com/desktop/install/mac-install/
- Windows: https://docs.docker.com/desktop/install/windows-install/

After installing, verify it works:
```bash
docker --version
docker compose version
```

### Visual Studio Code

We use VS Code to connect to the container and do our development inside it.

Download from: https://code.visualstudio.com/download

### VS Code Extensions

You will need the Dev Containers extension. Install it from:
https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers

Or search for "Dev Containers" in VS Code's extension marketplace.

That is it for requirements! The container itself comes with everything else (ROS 2, Python packages, C++ tools, etc.).

## Getting Started

### Clone the Repository

First, grab the code:

```bash
git clone https://github.com/SDU-Vikings-Racing-Team/ros-devcontainer.git
cd ros-devcontainer
```

### Open in VS Code

Open the project folder:

```bash
code .
```

VS Code will detect the `.devcontainer` folder and show you a notification in the bottom right corner asking if you want to "Reopen in Container". Click that button.

Alternatively, you can:
1. Press `F1` (or `Ctrl+Shift+P` on Linux/Windows, `Cmd+Shift+P` on macOS)
2. Type "Dev Containers: Reopen in Container"
3. Press Enter

### First Launch

The first time you open the container, it will take a few minutes because it needs to:
1. Figure out your environment (do you have X11 for GUI apps? Are we in CI?)
2. Build the Docker image with ROS 2 and all the tools
3. Set up the workspace structure
4. Link any packages you have in the `src/` folder
5. Install dependencies
6. Build everything

You will see all this happening in the VS Code terminal. Once it is done, open a new Bash terminal and you are ready to go!

## Your Development Environment

When you are inside the container, here is what you get.

### The Workspace

Everything lives under `/home/rosdev/ros_ws/`:

```
/home/rosdev/ros_ws/
├── src/
│   ├── host_packages/        # Your packages (symlinked from ../src on host)
│   └── thirdparty_packages/  # External dependencies
├── build/                    # Build artifacts
├── install/                  # Installed packages
├── log/                      # Build and test logs
├── bags/                     # ROS bag recordings
└── scripts/                  # Utility scripts
```

Your packages that live in the `src/` folder on your host machine are automatically linked into `src/host_packages/` in the container. This means you can edit files on your host with any editor you like, and the changes show up immediately in the container.

### Environment Already Set Up

The container automatically sets up ROS 2 for you. When you open a new terminal, it does the following:
- Sources the ROS 2 Humble environment
- Sources your workspace overlay
- Sets up helpful aliases (more on these below)
- Puts you in the workspace directory

## Workflow

### Building Your Code

We have some convenient aliases to make building easier:

```bash
# Build everything
cb

# Build just one package
cbp my_package_name

# Build a package and all its dependencies
cbu my_package_name
```

These are shortcuts for `colcon build` with our standard arguments already configured (symlink install, compile commands for IDE, release mode, etc.).

### Running Your Code

After building, run your nodes like normal:

```bash
# Run a node
ros2 run my_package my_node

# Launch a launch file
ros2 launch my_package my_launch_file.py
```

The environment is already sourced, so your packages are immediately available.

### Testing

```bash
# Run all tests
ct

# Test a specific package
ctp my_package_name

# Show test results
ctr
```

### Starting Fresh

If you need to clean everything and rebuild from scratch:

```bash
clean_ws  # Removes build/, install/, and log/
cb        # Build everything again
```

## Working with Packages

### Creating a New Package

Create packages in the `src/` folder on your host machine (not inside the container):

```bash
# On your host computer, in the project root
cd src
ros2 pkg create --build-type ament_cmake my_new_package
```

The next time you open the container (or run the setup script), it will automatically get linked and you can build it.

If you want it available immediately without restarting:

```bash
# Inside the container
bash /home/rosdev/ros_ws/scripts/setup.sh
```

### Adding External Dependencies

We use a `packages.repos` file to manage third party packages. This uses the VCS tool format.

1. Edit `.devcontainer/packages.repos`:

```yaml
repositories:
  some_dependency:
    type: git
    url: https://github.com/org/some_dependency.git
    version: humble
```

2. Import the packages:

```bash
cd /home/rosdev/ros_ws/src/thirdparty_packages
vcs import < /home/rosdev/ros_ws/packages.repos
```

3. Install dependencies and build:

```bash
rosdep_install  # Alias for rosdep install
cb              # Build everything
```

## Using Foxglove Studio

The container comes with Foxglove Bridge for visualizing data.

Start the bridge:

```bash
foxglove
```

This starts a WebSocket server on port 8765. Then open Foxglove Studio (either the desktop app or https://studio.foxglove.dev) and connect to:

```
ws://localhost:8765
```

> **Note:** You will need to sign in with a Foxglove account to access Foxglove Studio. If you do not have one, you can create it for free on their website.

You can customize the port in `.devcontainer/config/defaults.env` if needed.

## Useful Commands

Here is a quick reference of the aliases and commands you will use most often.

### Building and Testing

```bash
cb              # Build all packages
cbp <pkg>       # Build specific package
cbu <pkg>       # Build package and its dependencies
ct              # Test all packages
ctp <pkg>       # Test specific package
ctr             # Show test results
clean_ws        # Remove build artifacts
source_ws       # Re-source the workspace
```

### ROS 2 Commands

```bash
# See what is running
ros2 node list
ros2 topic list

# Monitor a topic
ros2 topic echo /my_topic

# Check topic info
ros2 topic info /my_topic

# See available services
ros2 service list
```

## GUI Applications

The container automatically detects if you have X11 available on your host. If you do, GUI applications like RViz2 will work out of the box:

```bash
rviz2
```

On Linux, this should just work. On macOS or Windows, you might need to set up an X server (like XQuartz or VcXsrv).

## Common Issues

### Container will not start

First, check Docker is running:

```bash
docker ps
```

If that fails, start Docker Desktop or the Docker daemon.

Try regenerating the environment setup:

```bash
bash .devcontainer/host-setup.sh
```

Then rebuild:

```bash
docker compose -f .devcontainer/docker-compose.yml build --no-cache
```

### Permission denied errors

This usually means your user ID does not match what the container expects (1000).

Check your ID:

```bash
id
```

If you are not `1000:1000`, edit `.devcontainer/docker-compose.yml` and change:

```yaml
services:
  ros2-dev:
    build:
      args:
        DEV_USER_ID: <your-uid>
        DEV_GROUP_ID: <your-gid>
```

Then rebuild the container.

### ROS 2 commands not found

The environment should auto source, but if it does not:

```bash
source_ws
```

Or manually:

```bash
source /opt/ros/humble/setup.bash
source /home/rosdev/ros_ws/install/setup.bash
```

### Package not found after adding it

For packages in `src/`:

```bash
bash /home/rosdev/ros_ws/scripts/setup.sh
```

For packages in `packages.repos`:

```bash
cd /home/rosdev/ros_ws/src/thirdparty_packages
vcs import < /home/rosdev/ros_ws/packages.repos
rosdep_install
cb
```

### Build fails after pulling updates

Sometimes the build cache gets stale. Clean and rebuild:

```bash
clean_ws
cb
```

If that does not work, completely reset the container:

```bash
# On host
docker compose -f .devcontainer/docker-compose.yml down -v
docker compose -f .devcontainer/docker-compose.yml up -d
```

## Configuration

You can customize the environment by editing `.devcontainer/config/defaults.env`:

```bash
# ROS configuration
ROS_DISTRO=humble
ROS_DOMAIN_ID=7

# Build configuration  
BUILD_TYPE=Release

# Foxglove
FOXGLOVE_PORT=8765
```

After changing configuration, rebuild the container for changes to take effect.

## Working without VS Code

If you prefer to work without VS Code, you can still use the container:

```bash
# Generate environment specific config
bash .devcontainer/host-setup.sh

# Start the container
docker compose -f .devcontainer/docker-compose.yml \
               -f .devcontainer/docker-compose.override.yml \
               up -d

# Enter the container
docker compose -f .devcontainer/docker-compose.yml \
               -f .devcontainer/docker-compose.override.yml \
               exec ros2-dev bash

# You are now inside the container
cd /home/rosdev/ros_ws
```

## Updating the Container

When the devcontainer configuration gets updated (new dependencies, ROS packages, etc.), you will need to rebuild.

Using VS Code:
1. Press `F1`
2. Type "Dev Containers: Rebuild Container"
3. Press Enter

Using Docker Compose:
```bash
docker compose -f .devcontainer/docker-compose.yml build --no-cache
docker compose -f .devcontainer/docker-compose.yml up -d
```

## Tips and Tricks

### Parallel Builds

Builds already use all your CPU cores by default. But if you want to limit it:

```bash
colcon build --parallel-workers 4
```

### Build Only Changed Packages

```bash
colcon build --packages-up-to my_package
```

This rebuilds `my_package` and anything it depends on, but skips unrelated packages.

### Keep Bash History

Your bash history persists between container sessions. It is stored in your home directory on the host, so you will not lose your command history when rebuilding.

### Multiple Terminals

You can open multiple terminals in VS Code (Terminal → New Terminal), and they will all be inside the container. Each one is a separate bash session, so you can run multiple nodes simultaneously.

## Getting Help

If you run into issues:

1. Check the Common Issues section above
2. Look at the container logs: `docker compose logs`
3. Search our GitHub issues to see if others have encountered the same problem
4. Ask in the team chat with a brief description of your issue and relevant logs

If you have identified a bug, please report it as a GitHub issue using the Bug template in this repository. Include details about how the bug occurred and any error messages you encountered.

## Contributing

Found a bug or want to add a feature? Contributions are welcome!

1. Create a branch from `main` (use `feature/feature_name` or `fix/bug_description` format)
2. Make your changes
3. Test that the devcontainer still builds and works correctly
4. Update the README or other documentation if your changes affect usage
5. Open a pull request with a clear description of your changes

The CI workflow will automatically test that the container builds correctly.
