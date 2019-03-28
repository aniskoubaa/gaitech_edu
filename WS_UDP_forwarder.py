import socket               # Import socket module
from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket # pip install git+https://github.com/dpallot/simple-websocket-server.git
import base64
import time
import threading

# Load an color image in grayscale


ws_stream_port = 1111
wscmdport=9090
UDP_IP = "127.0.0.1"
UDP_PORT = 25500
MAX_SIZE=65000
robot_UDP_Port = 1109
WS = None
data=''
addr=None;
sockToDrone = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) 
websocket = None;
websocket_connection= False;



class Simplecmd(WebSocket):

    def handleMessage(self):
        # print(self.data)
        try:
            # print('connected to robot ')
	    global addr;
            print('sent to robot:  ',addr)
            sockToDrone.sendto(self.data,addr)

        except socket.error as e:
            print(e)
    def handleConnected(self):
        global websocket,websocket_connection
        websocket = self;
        websocket_connection= True;
        print(self.address, 'connected...')
         
        

    def handleError(self, error):
        print (error)
    
    def handleClose(self):
	global websocket,websocket_connection        
	websocket = None;
        websocket_connection= False;
        print(self.address, 'closed')

class WSThread ():
    def __init__(self,port, type , data_rate=1.0):
        self.type = type
        self.port = port
        self.data_rate = data_rate
        t = threading.Thread(target=self.run)
        t.setName('WSThread')
        t.start()
    
    def run ( self ):
        server = SimpleWebSocketServer('', self.port, self.type)
        server.serveforever()

class UDPThread():
    def __init__(self,websocket ,data , data_rate=1.0):
        self.data = data
        self.websocket = websocket
        self.data_rate = data_rate
        t = threading.Thread(target=self.run)
        t.setName('UDPThread')
        t.start()
    
    def run ( self ):
        self.websocket.sendMessage(u''+self.data);
	print ('sent to user')





wsc= WSThread(wscmdport, Simplecmd)
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT)) 
while True:
    #global addr;
    data, addr = sock.recvfrom(MAX_SIZE)
    print(addr)
    # print(data)
    print(websocket_connection)

    if(websocket_connection):
        
        UDPThread(websocket,data)

    

