# Configuration Documentation
## 

`The purpose of this document is the detail the process of how to set up the navigation software on a wheelchair platform and host device. It is primarily intended for developers and will focus on the technical aspects`

### Introduction
sfs
### Dependancies

#### Raspian
All python dependencies are managed by pipenv. Once you've downloaded the git repository onto both the host and wheelchair devices run pipenv install to install the required modules.

#### Arduino


### Networing

How to run the setup reliably you'll need to reserve an IP address for the base station you'll be using

 (Install MQTT](<https://www.arrow.com/en/research-and-events/articles/mqtt-tutorial>) on the raspberry pie so that it can host The transfer of data between the controller and the wheelchair

### Automation

#### udev rules

If we add python code to the start up procedures of the pi we can turn on wireless control automatically whenever the pi has power

#### Start Up Script
https://www.dexterindustries.com/howto/run-a-program-on-your-raspberry-pi-at-startup/
{{Add a more detailed description}}


https://unix.stackexchange.com/questions/455261/how-to-set-environmental-variable-in-systemd-service

