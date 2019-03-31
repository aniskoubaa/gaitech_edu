from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket # pip install git+https://github.com/dpallot/simple-websocket-server.git


# Load an color image in grayscale


websocket_prot = 9090

clients = []

class SimpleWebSocketHub(WebSocket):

    def handleMessage(self):
        for client in clients:
          if client != self:
             client.sendMessage(self.data)

        
    def handleConnected(self):
        print(self.address, 'connected')
        clients.append(self)
        print("connected clients", len(clients))
         
        

    def handleError(self, error):
        print (error)
    
    def handleClose(self):
        clients.remove(self)
        print(self.address, 'closed')
        print("connected clients", len(clients))

server = SimpleWebSocketServer('', websocket_prot, SimpleWebSocketHub)
server.serveforever()
