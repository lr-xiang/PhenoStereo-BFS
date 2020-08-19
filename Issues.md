## Bug List

### 10/09/2018:

socketServer:  
1. When cut connection on the client (host cpu) side, received `read header err code:asio.misc:2
`. But the server (Jetson) side "section" didn't destroy. ( FIXED 10/09 )
2. After 1, if connect again, the commands sent from client (host) were not received. (FIXED 10/09)
