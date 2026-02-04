# Modem
*This is a new page which will see rapid changes over the next few weeks*  

The modem had a custom housing designed for it as the original housing was too large. The antenni were snapped off to adhear to comp rules and for space, however original housing and antenni should still be in the AI storage closet.

## Modem 101 (for dummies)  
### Important Information:  
Model: Archer C50 (EU) - Ver: 6.20  
Full name: Archer C50 AC1200 Dual Band Wi-Fi Router.  
Power: 9V - 0.85A  
Default Access: http://tplinkwifi.net  
Wireless Password/Pin: 47142855  

Link to product specs: https://www.tp-link.com/uk/home-networking/wifi-router/archer-c50/#specifications  
Link to product manual: https://www.tp-link.com/uk/support/download/archer-c50/ (pdf available in directory with modem CAD files)  
You can see a picture of the original UI lighting and info-card in the reference pictures saved alongside the CAD files.  

#### Lighting Meanings 
(from left to right):  

**Power:**  
<font color="green">On (solid)</font>: Router is powered on and running normally (if you came here to figure out what this means, I have no hope for you).  
Off: No power (please make sure it is plugged in AND On button pressed, also it takes ~30s to fully turn on after starting, so wait before being sure it's dead).  
<font color="DarkGreen">Slow Flashing</font>: System is starting up or firmware is updating. Do not disconnect or power off.  
<font color="DarkGreen">Quick Flashing</font>: WPS connection in progress.  

**Wi-Fi (2.4 GHz):**  
*Ignore this, considering we removed the antennas - however I wrote out what you need to know anyway.*  
<font color="green">On</font>: The 2.4 GHz band is enabled.  
Off: Use your brain.  
	
**Wi-Fi (5 GHz):**  
*Ignore this, considering we removed the antennas - however I wrote out what you need to know anyway.*  
<font color="green">On</font>: The 2.4 GHz band is enabled.  
Off: Use your brain.  

**Ethernet/LAN:**  
*Note: I (Conor) snapped this light on 24/01/2026. If you open the modem and the LED is straight up not there, that's on me - my bad*  
<font color="green">On</font>: At least 1 Ethernet port is connected to a powered-on device.  
Off: No Ethernet port connected (or detected). Cry.  

**Internet/WAN:**  
<font color="green">Green On</font>: Internet Service is available.  
<font color="orange">Orange On</font>: Internet Port is connected, but internet is not available.  
Off: Router's internet port is not connected.  

*(4 LEDs should be green, the rightmost changes between green and orange so requires colour changing LED.)*

#### Buttons:  
WPS/Wi-Fi Button (to left of ethernet ports)  
Power On/Off Button (to right of ethernet ports, beside plug)  
Reset Button (need pin to poke, below WPS/Wi-Fi Button)  

#### Back Panel (I/O)
(mostly ripped from user manual):  
  
**Wi-Fi/WPS Button:**  
 Press this button for 1 second, and immediately press the WPS button on your device.  
 The Power LED of the router should change from flashing to solid on, indicating successful WPS connection.  
 Press and hold this button for about 5 seconds to turn on or off the wireless function of your router.  

**Reset Button:**  
Press and hold this button for 2 seconds until all LEDs turn off to reset the router to its factory default settings.  

**WAN Port:**  
For connecting to a DSL/Cable modem, or an Ethernet port.  

**LAN Ports (1/2/3/4):**  
For connecting your PCs or other wired network devices to the router.  

### Frequently Asked Questions:  
**Q:** Does the modem work wirelessly?  
**A:** No, per FSUK rules this expensive wireless modem does not work wirelessly.  
  
**Q:** Are you aware you could have gotten a much cheaper product considering you didn't even need 95% of what this does?  
**A:** Yes, but we are just silly guys (non-gender specifically :D).  
  
**Q:** Is the 3d printed mini-case gigantically over-engineered?  
**A:** Of course is it, anything Conor works on ends up way too complicated compared to the actual use case.  
  
**Q:** Why is the 4th LED missing?  
**A:** Conor is an idiot and broke it but didn't have a soldering iron and extra LED handy.  
  
**Q:** Why do you need an FAQ for how to boot a modem that doesn't answer anything to do with the modem or how to use it?  
**A:** Conor thought it was funny at the start and now regrets it because it's corny AF and he was just really bored D:.  

## Design Process Log  
Ignore the file "screw it" - it is for the screw bit on the modem casing top if it snaps off, but realistically just reprint it.  
The STL files should match the SolidWorks files, same with the 3mf files - those are optimised for a Bambu A1.  
The modem insides file is inaccurate. It is loosely right, but is particularly wrong around the I/O interface.  

You should be able to just directly print from the 3mf files and it should work, but you might have to sand around the I/O connectors (no clearance).  

## Design Explanation
The 2 pins at the back on the inside are for holes on the modem.  
The key-hole holes on the bottom are for mounting.  
The raised main platform for screwing should align exactly with the board's cutout.  
There are small ridges on the main body printout that align with the roof printout to make them align better.  
The grates on the side are mostly aesthetic, but also save some filament and serve the cooling need letting air flow thorugh.  
The 3mf files are printed in that orientation so the top side looks clean when printed, entirely aesthetic but no harm done anywhere.  
You'll probably have to loctite the screw holder on the roof back together, then it will stay there solid - there's a weird point of failure.  

