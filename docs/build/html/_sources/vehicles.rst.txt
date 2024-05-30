Working with the vehicles
==========================

Adding a new vehicle to the framework
--------------------------------------
To add a new vehicle to the framework, first setup an F1TENTH platform as described in the :doc:`Hardware documentation <hardware_architecture>`.
Then, retrieve the following propoerties of the vehicle:
- IP address
- Username
- Password

With the above information, create a login config file in the `configs` folder. The file should be named `car_ID_login.yaml`. The structure of the file should follow the provided template titled `Template_login.yaml`.
After the login config, create a new parameter file in the `configs` folder. The file should be named `car_ID_params.yaml`. The structure of the file should follow the provided template titled `Template_params.yaml`.
Note that these parameters will be used as default, when the vehicle is launched. The parameters can be changed during runtime using the Python API.

If the configs are ready and the vehicle is connected to the network, the onboard stack can be installed with the following script:

.. code-block:: python

   from aimotion_f1tenth_utils.install import install_onboard_stack

   install_onboard_stack('car_ID')

Note that the installation might take a couple of minutes.

Launching the vehicles
----------------------

1. **Connect the two plugs of the battery**:

   .. image:: images/plug1.png
      :alt: plug1

   .. image:: images/plug.png
      :alt: plug2

2. **Turn on the vehicle with the switch located on the power board**:

   .. image:: images/switch.png
      :alt: switch

   The red LED will light up. The onboard computer of the vehicle should connect to the same network that the OptiTrack server and the control PC use. Help for the configuration is available `here <https://f1tenth.org/build.html>`_.

3. **Connect to the vehicle via SSH and launch the onboard software stack**:
The username, IP address, and password information can be found in the ``car_ID_login.yaml`` file in the configs folder.
After the SSH connection is established with the vehicle, run the following commands to launch the vehicle:

   .. code-block:: bash

      cd aimotion_f1tenth_system
      source startup_framework.sh
