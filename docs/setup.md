# Configuration Documentation
## 

`The purpose of this document is the detail the process of how to set up the navigation software on a wheelchair platform and host device. It is primarily intended for developers and will focus on the technical aspects`

### Introduction
This is an autonomous wheelchair platform built primarily on Python and Arduino C. The software is run on a Raspberry pi and uses a standard camera for motion detection and line tracking.
### Dependancies

#### Raspian
 You can install the most recent version of raspbian By following the instructions [here](https://www.raspberrypi.org/software/).

#### Python

All python dependencies are managed by pipenv. You can follow the installation instructions [here](https://pipenv.pypa.io/en/latest/install/). Once you've downloaded the git repository onto both the host and wheelchair devices run ```pipenv install``` while inside the repo to install the required modules.
In order to fix a downstream packaging issue will need to install another dependency ```sudo apt-get install libatlas-base-dev```. Also you may need to uninstall and reinstall pyserial it's a normal issue but not one I found a cause for.

### Networing
You'll need to reserve an IP address for the base station you'll be using and for the wheelchair.
The specific way of going about this will depend on your router but just reserve two addresses and keep them aside
```
chair_ip = "192.168.0.1"
base_ip = "192.168.0.2"
```

#### Start Up Script

https://www.dexterindustries.com/howto/run-a-program-on-your-raspberry-pi-at-startup/
For convenience the run that by file is launched automatically when the computer turns on
In order to set this up you'll need to create a systemd service

https://unix.stackexchange.com/questions/455261/how-to-set-environmental-variable-in-systemd-service


### Navigation

### P1
The first step is creating QR codes you should create a code for every destinations that you want the system to be able to navigate to. And each code should contain only a unique integer ID. For example in an airport the QR code located at Gate A might contain an ID of 3. The actual information of how these QR codes are connected is stored elsewhere in the software.

While there are plenty of alternatives one say that generates QR codes would be [www.qr-code-generator.com](https://www.qr-code-generator.com/)
### P2
Once all of the QR codes have been generated the next step is to define their relationships to one another so that we can navigate between them.

``` python 
class QrMap():
    def __init__(self):
        """All the time the option to be connected to another note through one of four directions"""
        self.__connections = {
            # [top, bottom, left, right]
            1: [0,3,0,0], 
            2: [0,3,0,0], 
            3: [4,2,0,1], 
            4: [3,0,0,0]
        }
```

The top of a QR code is described as such 
<add image>

As long as the connections between the map and the actual QR codes are consistent the wheelchair should be able to go anywhere along the guiding line and navigate the intersections.

Continue in the configurations