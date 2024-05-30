Control the vehicles by the Python API
=========================================
This page outlines the TCP-based communication protocol and the Python API that can be used for controlling the vehicles.

Communication protocol
----------------------
The onboard stack of the F1TENTH vehicles runs a TCP server that listens for incoming connections on port 8000.
The server expects to recieve packets of the following format:

- 4 bytes: The length prefix that indicates the length of the payload. The prefix can be encoded as

    .. code-block:: python

        struct.pack('I', len(payload))

- N bytes: The payload, which is a serialized Pytohn dictonary which can be constructed as

    .. code-block:: python

        payload = {'key1': value1, 'key2': value2, ...}
        payload = pickle.dumps(payload)
    
The server will respond with a serialized Python dictionary that contains the response to the request. The standard format of a request sent by the client is:

    .. code-block:: python

        request = {'command': 'command_name', "arg1": value1, "arg2": value2, ...}

where the `command_name` is the name of the command that the client wants to execute. 
The full list of available commands can be found in the automatically generated code documentation of the `F1Client` class of the API.
The server will respond with a dictionary that contains the key `status` with a `boolean` value which indicates the status of the command execution. 
If the command failed, the value of the `status` key will be `False` and the dictionary will contain an additional key `error` that contains the error message.
The comminication interface is implemented in the :doc:`modules/aimotion_f1tenth_utils.communicaton` module.

The Python API
--------------
The Python API of the `aimotion_f1tenth_utils` package provides a high-level interface for controlling the vehicles. 
It automatically handles the communication with the vehicle (i.e serializes the messages, retrives and handles the responses) 
and provides a set of functions that can be used to control the vehicle. The automatically generated documentation of the API can be found :doc:`modules/aimotion_f1tenth_utils`

Examples
--------
Finally, the following examples present the main functionalities of the API. The examples assume that the vehicles are running and listening for incoming connections on port 8000.
The scripts can also be found in the `examples` directory of the package.

Installing the onboard stack of the vehicles
********************************************
.. literalinclude:: ../../examples/install_onboard_stack.py
   :language: python
   :linenos:

Manual control by the keyboard
*******************************
.. literalinclude:: ../../examples/keyboard_control.py
   :language: python
   :linenos:

Execute a presaved trajectory
******************************
.. literalinclude:: ../../examples/example_trajectory_execution.py
   :language: python
   :linenos: