# aimotion_f1tenth_utils.communicaton package

## Submodules

## aimotion_f1tenth_utils.communicaton.TCPClient module

### *class* aimotion_f1tenth_utils.communicaton.TCPClient.TCPClient

Bases: `object`

#### close()

#### connect(host, port)

#### send(message)

Function that sends a message to the server and returns the response

## aimotion_f1tenth_utils.communicaton.TCPServer module

### *class* aimotion_f1tenth_utils.communicaton.TCPServer.TCPServer(host, port, message_callback=None)

Bases: `object`

#### close_connection(con_ID)

Function that closes the connection with the given ID

* **Parameters:**
  **con_ID** – ID of the connection to be closed

#### handle_connection(client_socket, con_ID)

Function that handles the connection with a single connected client!

#### start()

#### stop()

Function that stops the server forces all the connections to close

## Module contents
