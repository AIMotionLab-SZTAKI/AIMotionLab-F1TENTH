# aimotion_f1tenth_utils.communicaton package

## Submodules

## aimotion_f1tenth_utils.communicaton.TCPClient module

### *class* aimotion_f1tenth_utils.communicaton.TCPClient.TCPClient

Bases: `object`

#### close()

Close the connection to the server

#### connect(host, port)

Establish connection to the TCP server with the given host and port

* **Parameters:**
  * **host** (*str*) – IP address of the server
  * **port** (*int*) – Port of the server
* **Returns:**
  Status of the connection
* **Return type:**
  bool

#### send(message)

Function that sends a message to the server and returns the recieved response

* **Parameters:**
  **message** (*dict*) – Message to be sent to the server. The message should be encoded in a dictinary format

## aimotion_f1tenth_utils.communicaton.TCPServer module

### *class* aimotion_f1tenth_utils.communicaton.TCPServer.TCPServer(host, port, message_callback=None)

Bases: `object`

#### close_connection(con_ID)

Function that closes the connection with the given ID

* **Parameters:**
  **con_ID** (*int*) – ID of the connection to be closed

#### handle_connection(client_socket, con_ID)

Function that handles the connection with a single connected client This function is called in a thread for each connected client

* **Parameters:**
  * **client_socket** (*socket*) – Socket object of the connected client
  * **con_ID** (*int*) – ID of the connection

#### start()

Start the TCP server, waits for connections and establishes communication channels with the clients

#### stop()

Function that stops the server forces all the connections to close

## Module contents
