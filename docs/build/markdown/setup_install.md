# Software installation and setup instructions

This page guides users through the full installation and setup procedure of the framework on all the hardware components outlined in the [Lab architecture](hardware_architecture.md) section of the documentation.

## Command PC setup

As the Command PC runs a standalone Python package, the setup is straightforward via pip:

1. **Download the f1tenth_r2 framework from GitHub**:
   : ```bash
     https://github.com/AIMotionLab-SZTAKI/AIMotionLab-F1TENTH.git
     ```
2. **Create a virtual environment and activate it**:
   > ```bash
   > cd AIMotionLab-F1TENTH
   > python3 -m venv venv
   > source venv/bin/activate
   > ```
3. **Install the Python package as an editable**:
   > ```bash
   > pip install -e .
   > ```

## MoCap server setup

1. **Motive setup**: In order to use the MoCap-based state estimator fot the vehicle the followign steps are necessary:
   - Define a RigidBody in Motive corresponding to the marker configuration on the vehicle
   - Make sure that the ID of the RigidBody mathces the ID of the vehicle
   - Retrieve the IP address of the MoCap server machine
2. **Crazymocap setup**:
   - Install the crazymocap Python package from GitHub:
   ```bash
   git clone https://github.com/AIMotionLab-SZTAKI/crazymocap.git
   cd crazymocap
   pip install -e .
   ```

   - To stream MoCap data to the vehicle, the RadioStreamer class can be used as follows:

   ```python
   from crazymocap.radio_streamer import RadioStreamer
   import traceback

   streamer = RadioStreamer(devid=0, ip='mocap_server_ip', object_name='car_ID')
   try:
      while True:
         streamer.send_pose()
   except Exception as e:
      print(f"Exception: {e!r}. Trackeback:")
      print(traceback.format_exc())
      streamer.close()
   ```

   #### NOTE
   This script is only capable of streaming the pose data of a single vehicle defined by the object_name parameter.
   For multiple vehicles, multiple instances of the RadioStreamer class should be created, each with a different object_name parameter.
   Furthermore, one radio is only capable of handling a single connection, so multi-vehicle configurations require multiple Crazyradio dongles.
   Use the devid parameter to differentiate between the Crazyradio dongles.

## F1TENTH vehicle setup

1. **Prerequisites**:
   - A fully built F1TENTH vehicle
   - A working installation of ROS2 Foxy

   #### NOTE
   The official F1TENTH onboard software stack is not required as the framework uses its own custom onboard software.
2. **Configure the platform**: This can be most conveniently done by plugging a monitor and keyboard into the onboard computer, but ssh is also an option.
   - Connect the F1TENTH vehicle to the local network. Be sure to check the IP address of the vehicle as it will be required later.
   - Configure `udev` rules for the VESC motor controller:

     Open `/etc/udev/rules.d/99-vesc.rules` as the root user and copy and paste the following rule for the VESC into the file:
     ```text
     KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"
     ```

     Finally, trigger (activate) the rule by running:
     ```bash
     sudo udevadm control --reload-rules
     sudo udevadm trigger
     ```
   - Install the Python packages required for building the software environment of the onboard stack:
     ```bash
     sudo apt install python3-pip
     sudo apt install build-essential libssl-dev libffi-dev python-dev
     sudo apt install numpy pyyaml gpytorch casadi scipy matplotlib
     ```
   - Configure USB permissions for the Crazyradio as described [here](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/)

   #### NOTE
   > USB configuration and permissions can be checked by listing the connected USB devices:
   ```bash
   usb-devices
   ```
