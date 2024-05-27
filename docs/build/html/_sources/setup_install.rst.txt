Software installation and setup instructions
==================================================
This page guides users through the full installation and setup procedure of the framework on all the hardware components outlined in the :doc:`hardware_architecture` section of the documentation.

Command PC setup
----------------

As the Command PC runs a standalone Python package, the setup is straightforward via pip:

1. **Download the f1tenth_r2 framework from GitHub**:   
    .. code-block:: bash

        git clone https://github.com/AIMotionLab-SZTAKI/f1tenth_r2

2. **Create a virtual environment and activate it**:

    .. code-block:: bash

        cd f1tenth_r2
        python3 -m venv venv
        source venv/bin/activate

3. **Install the Python package as an editable**:

    .. code-block:: bash

        pip install -e .
        
F1TENTH vehicle
---------------

1. **Prerequisites**:

   - A fully built F1TENTH vehicle
   - A working installation of ROS2 Foxy

   .. note::
      The official F1TENTH onboard software stack is not required as the framework uses its own custom onboard software.

2. **Configure the platform**: This can be most conveniently done by plugging a monitor and keyboard into the onboard computer, but can also be done via ssh.
   - Connect the F1TENTH vehicle to the local network. Be sure to check the IP address of the vehicle as it will be required later.
   - Configure ``udev`` rules for the VESC motor controller:
   
     Open ``/etc/udev/rules.d/99-vesc.rules`` as the root user and copy and paste the following rule for the VESC into the file:

     .. code-block:: text

        KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"

     Finally, trigger (activate) the rule by running:

     .. code-block:: bash

        $ sudo udevadm control --reload-rules
        $ sudo udevadm trigger

   - Install the Python packages required for building the software environment of the onboard stack:

     .. code-block:: bash

        $ sudo apt install python3-pip
        $ sudo apt install build-essential libssl-dev libffi-dev python-dev
        $ sudo apt install numpy pyyaml gpytorch casadi scipy matplotlib

   - Configure USB permissions for the Crazyradio as described `here <https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/>`_

   .. note::
      USB configuration and permissions can be checked by listing the connected USB devices:

     .. code-block:: bash

        $ usb-devices
