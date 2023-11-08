# pico-navbot

Digital maker project: Self navigating robot with RPi Pico 

The idea is to give the robot a route consisting of legs which consist of a bearing and a distance. As such a route must have at least one leg.
The robot would then drive in the direction specified for the required distance. When it encounters an obstacle it must try and avoid it. For each detour it makes it must calculate the new heading and distance to the original target point of the leg.

I have now finally sourced all the basic components and completed the build. I most probably will have to upgrade the battery pack.

Now the code for the navigation must be written and tested.

Then field testing to see if the concept and implementation actually works :)

![Testing](../main/images/Planning.png "Planning")

Added a Command module. It enables the creation, updating and deleting routes and the legs of routes. It also connects to the bot and enables the transmission of a route to the bot. Once the route is loaded the bot can be commanded to drive the route. The module also displays progress information received from the bot during the drive.

![Command Design](../main/images/command.png "Command Planning")

The STOP is not implemented because MicroPython does not provide support for an IRQ on UART

Now the detour navigation must be completed.