# DWM3000 ROS

This is the software suite that supports the UWB-based interrobot ranging system used in flight tests for the paper **Observability-Aware Control for Cooperatively Localizing Quadrotor UAVs**.

It contains four components

- `dwm3000_esp`: The ESP32 firmware
  - This is a heavily cut-down version of kk9six's [ESP32 UWB DW3000](https://github.com/kk9six/dw3000)
  - Only anchor-tag mode is supported, distance-matrix is removed
  - Ranging measurements are sent over the serial connection as nanopb messages

- `dwm3000_ros`: The ROS node
  - This is the ROS node that decodes the incoming serial messages from the ESP32 and broadcasts it out again as ros messages

- `dwm3000_msgs`: The ROS message definitions

- `proto`: nanopb message definitions

## Dependencies

### dwm3000_ros

Depends on the FSC Lab's serial bridge package `fsc_serial`, which acts as a thin wrapper over Boost::asio.

### dwm3000_esp

The requisite NConcepts Dw3000 driver package is built-in in the `lib` subdirectory.

Vscode's platformio `platformio.platformio-ide` IDE is used to build the embedded software and upload the binaries to the ESP32 board.
Install it from the extensions store.

## Building

### dwm3000_ros

Put this package inside a ROS workspace, e.g. `catkin_ws/src`, then run

``` bash
catkin build
```

at the workspace root.

### dwm3000_esp

Open the `dwm3000_esp` folder as a platformio project, then use the built-in build/upload buttons.

