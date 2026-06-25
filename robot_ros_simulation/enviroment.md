# HexNav Environment Setup Notes

This file summarizes only the environment-level changes needed to build, export,
and launch the HexNav ROS 2 package reliably from this workspace. It does not
cover robot geometry, URDF joint placement, or IK math changes.

## Workspace Target

- Workspace root: `/Users/vitormhenrique/dev/vitor/Robot/robot_ros`
- ROS package: `HexNav_description`
- Default ROS/Pixi environment: `jazzy`
- Package share path after build: `install/HexNav_description/share/HexNav_description`

## Pixi Environment

The workspace is driven through Pixi instead of relying on a globally sourced ROS
installation. Commands should target the local manifest explicitly:

```sh
pixi run -e jazzy --manifest-path pixi.toml <command>
```

The Just helpers encode this as:

```just
pixi := "pixi run -e " + ros_env + " --manifest-path " + manifest
```

That avoids accidentally using another `PIXI_PROJECT_MANIFEST` or another Pixi
project from the shell environment.

## Shell And Activation Fixes

- The `justfile` uses zsh as its recipe shell:

  ```just
  set shell := ["zsh", "-cu"]
  ```

- On Unix/macOS, Pixi activates the colcon overlay with `install/setup.sh`:

  ```toml
  [target.unix.activation]
  scripts = ["install/setup.sh"]
  ```

- `install/setup.sh` was chosen over `install/setup.bash` because Pixi may run
  activation from zsh on macOS, while `setup.bash` depends on Bash-specific
  behavior such as `BASH_SOURCE`. The POSIX `setup.sh` self-locates correctly
  and works from zsh.

- Recipes that need Bash-specific syntax still call Bash explicitly inside Pixi.
  For example, `move-leg` uses a Bash array and therefore runs with
  `{{pixi}} bash -lc ...` while the outer Just shell remains zsh.

## ROS Jazzy Dependencies Added

The Jazzy Pixi environment includes the ROS runtime pieces needed for display,
joint testing, and ros2_control:

```toml
ros-jazzy-desktop = "*"
ros-jazzy-ros-gz = "*"
gz-sim8 = "*"
ros-jazzy-joint-state-publisher-gui = "*"
ros-jazzy-ros2-control = "*"
ros-jazzy-ros2-controllers = "*"
filelock = "*"
```

The package manifest also declares the runtime dependencies that ROS tooling
expects, including `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`,
`xacro`, `controller_manager`, `ros2_control`, `ros2_controllers`,
`joint_state_broadcaster`, `forward_command_controller`, and `mock_components`.

## Package Export And Installed Assets

The package exports as an `ament_cmake` package through `package.xml`:

```xml
<export>
  <build_type>ament_cmake</build_type>
</export>
```

The CMake install step exports the assets required at launch time into the
package share directory:

```cmake
install(DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME}
)
```

This is what lets `ros2 launch HexNav_description display.launch.py` find the
xacro, meshes, RViz config, and controller config from the installed overlay.

## Build And Launch Helpers

The important Just commands are:

```sh
just install      # solve/install the selected Pixi environment
just build        # colcon build --symlink-install inside Pixi
just rebuild      # remove build/install/log and rebuild
just launch       # launch display.launch.py from the installed package
just check-urdf   # expand installed xacro and parse it with check_urdf
just controllers  # list ros2_control controllers
```

The Unix build task uses `colcon build --symlink-install`, which keeps installed
package assets linked to the source tree and makes iteration faster.

## Runtime Notes

- macOS may print Fast DDS thread-affinity warnings such as `Protocol family not
  supported`. These are noisy but were not the root cause of the build or launch
  failures.
- If controllers report that they are already loaded, an old launch may still be
  running. Stop stale ROS processes or restart the ROS daemon before relaunching.
- After changing environment dependencies, run `just install` before rebuilding.
- After changing launch, URDF, mesh, RViz, or controller assets, run `just build`
  so the installed package share tree is current.

## Known-Good Verification

The environment-level setup is considered healthy when these pass:

```sh
just info
just build
just check-urdf
just launch
just controllers
```

Expected controller state after launch:

```text
joint_state_broadcaster active
position_controller     active
velocity_controller     active
```