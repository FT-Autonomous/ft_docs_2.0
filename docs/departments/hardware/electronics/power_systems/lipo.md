# LiPo battery

Hardware uses two 4S 16V 6500mAh LiPos

## why do we use LiPos?
For testing purposes, its useful to be able to move systems around without having it connected to a stationary power supply or wall socket.

Its also fundamental for onboard power systems for rigby, as the power source needs to be fully mobile.

## why do we need two identical LiPos?
By splitting the power systems in to two independantly powered systems, we reduce overall voltage and current, thus signifigantly reducing potential hazards and failure points in the system.

thus, one LiPo can be used to power Rigby's driving systems (velocity and steering), while another can be used to power the compute stack (Jetson AGX, LiDar, ethernet switch). This also means if the compute stack wants to be tested independantly, the power supply is adequately sized for the need.

## LiPo Safety
some important ground rules for using the LiPo

1. Dont use the LiPo if you are unsure about the safety or havent used one previously. Contact a member of the hardware team to recieve adequate training on LiPo safety.
1. Before using the liPo, use the charger to check state of charge (SoC) of the LiPo. once the cells are emptied, they cannot be recharged, and you are left with an expensive, firehazard paperweight.
1. Never, ever connect the red and black terminals of the LiPo directly. Doing so will cause a short in the system and may cause the LiPo to catch fire.
1. LiPos are simply batteries, with no current limiting devices, if you create a circuit with no limit on current draw, it will keep drawing hihger current until something breaks - be it wires melting, or electronics simply dying. Always ensure there is a fuse along the wire, and you NEVER bridge the fuse gap with a conductor to bypass the safety measure.
