# Configuration Documentation
## 

`The purpose of this document is the detail the process on how to create a new map of QR codes for navigation`

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