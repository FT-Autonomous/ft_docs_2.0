# Sensor Stack Explanation
**Components:**  
- LiDAR ([VLP 16](../../equipment/velodyne.md), (comes with separate LiDAR box))  
- Camera ([Zed2i](../../equipment/zed2.md), note Zed2i new for 25/26, documentation is for older Zed2)  
- Computer ([Jetson AGX Orin](../../equipment/jetson_orin.md),)  
- Modem ([TP Link Archer C50 (EU)](../../departments/hardware/mechanical/mount/modem.md), new for 25/26)  
- Navigation Satellite System ([Xsens MTI-680](https://www.xsens.com/sensor-modules/xsens-mti-680-rtk-gnss-ins), no internal documentation yet, new for 25/26)  

**What has a required fov-based location:**  
- LiDAR  
- Zed2i camera  

**Other physical restraints:**  
- The LiDAR physically cannot be separated from the LiDAR box.  

- We need minimal movement of AI components (e.g. keep everything together except the stuff that has to move. For ADS, AI (Hardware) will make these components convenient by design, all taking a small (less than 20〖cm〗^3 footprint)).  

- Ideally, the LiDAR and Zed2i should be kept as closely together as possible – testing is currently being done to determine the furthest safe distance (specifically on height and depth). Zed2i should be vertically higher than LiDAR. Both should be on the same horizontal plane within the car (wheel-to-wheel plane on car). Depth difference is the current unknown variable currently being tested, it has never been more than 10cm apart on AI car.

- Not relevant for placement, but in manufacturing remember that these components run hot, hot enough to make PLA 3d prints deform if under any pressure, so use PETG or ABS if possible when printing.

{add in picture of LiDAR and Zed2i dimensional drawings}

## LiDAR Placement Requirements:  
- Ideally should be as far forward on the car as possible.  
- Needs to remain level.  
- Should be as low to floor as possible while retaining a full 180-degree view in front of it.  
- Very expensive and on loan from another company, so need to keep it safe (can’t be on front of the car if there’s a crash, because then FT is in debt).  
- For mounting remember that there needs to be a direct line for fast wiring between it and the LiDAR box (space can’t just be left to run wire, the LiDAR will have to fit through gaps as well, maybe hatch somewhere?).  
- *Requires minimal vibrations to work well.*  

## Zed2i Placement Requirements:  
- Needs as clear a view of the road as possible – meaning a placement as high as possible (above driver?).  
- Needs to remain level.  
- Bolted USB-C cable used to plug in, no restraint on placement based on cabling but we would have to buy a dedicated ADS cable for it depending on placement (not issue, just to remember for assembly).  
- Requires minimal vibrations to work well.  
- *Note: Perfect camera placement should be akin to a driver’s vision while they are driving – but we cannot block the driver’s line of sight at all.*  

## Additional Information:  
A comprehensive overview of the mechanical aspects of the AI computer components is available within the [Hardware Department's](../../departments/hardware/mechanical/mechanical_overview.md) section of the docs.  