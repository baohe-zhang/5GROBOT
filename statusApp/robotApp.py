# ...

# START send message to database
import requests
from time import strftime
import socket, fcntl, struct

def sendToDatabase(msg):
    url = 'https://robotstatusapp.appspot.com/status/add'
    msg = json.loads(msg)
    msg["trackStatus"] = str(msg.pop("TrackStatus"))
    msg["angle"] = str(msg["angle"])
    msg["speed"] = str(msg["speed"])
    msg["detectStatus"] = "0"

    # get time
    msg["time"] = strftime('%d/%b/%Y %H:%M:%S')

    # get IP address
    ifname = 'wlan0'    # interface name, usually wlan0
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);  
    inet = fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', ifname[:15]));  
    ip = socket.inet_ntoa(inet[20:24]);  
    msg["ipAddress"] = ip

    
    r = requests.post(url, json=msg)
    
    print 'Send To Database OK'
# END


# ...

#/restconf/config/robotApp/
class RobotappMethodView(MethodView):

    def put(self, ):
        print "Update operation of resource: robotApp"
        
        # send command to cloud database
        try:
            sendToDatabase(request.data)
        except Exception:
            pass
        
        # ...
