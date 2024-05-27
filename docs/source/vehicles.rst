Working with the vehicles
==========================

This page details how to start up the vehicles and launch their onboard software stack.

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
The SSH connection can be established by the provided startup script:

   .. code-block:: bash

      $ ./lauch_vehicle.sh -u vehicle_username -i vehicle_ip_address -p vehicle_password
