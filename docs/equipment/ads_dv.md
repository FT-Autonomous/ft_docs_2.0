# The ADS-DV Vehicle

![Ads-dv car](https://imechewebresources.blob.core.windows.net/imeche-web-content-osf/images/default-source/oscar/Formula-Student/fs19_fsaicar_850.jpg?sfvrsn=0)

The "Automated Driving System Dedicated Vehicle" (ADS-DV) is the vehicle provided to teams tat do the DDT competiton at FSUK.

Here are some useful links:

- [FS-AI\_ADS-DV\_CAD GitHub repository](https://github.com/FS-AI/FS-AI_ADS-DV_CAD) - Contains CAD files for the ADS-DV which include images denoting the maximum height for mounts and provide guidelines for mount design.

### Interfacing with the ADS-DV

We use the `ros_can` library from EUFS to interface with the ADS-DV.

- [EUFS ros_can repository](https://gitlab.com/eufs/ros_can/)

The ADS-DV is pretty tempermental and has a lot of strict conditions for safety. When these conditions are not met, you'll find that the vehicle triggers its Emergency Brake System (EBS) immediately instead of running your code. At other times the vehicle may enter the driving state and... not drive. Here's a link to the manual which you should read in full in the weeks coming up to the competition, or any event where you'll be using the vehicle.

- [ADS-DV Manual](https://github.com/FS-AI/FS-AI_API/blob/main/Docs/ADS-DV%20User%20Manual%202021.pdf)
- [Reasons The Car May Not Move on Track **(☢ this is important ☢)**](../resources/why_is_car_not_moving.md)

Possibly the most important figure to understand is *Figure 20* which outlines the internal state machine of hte vehicle and the necessary conditions for transitioning from one state to another.

### Deriving 12V DC from the ADS-DV

{% image src="https://www.te.com/content/dam/te-com/images/industrial-and-commercial-transportation/global/product-hero-rendition/dt-series-470x445.jpg/jcr:content/renditions/hero-product-master.jpg?w=470" width="200px" /%}

The DDT car provides power over an esoteric "Deutche DT" cable which you probably will have *never* heard of before joining FT.

Here are some resources:

- [Amazon Link for Deutche DT Connectors](https://www.amazon.co.uk/dp/B0BTHLB357?ref=nb_sb_ss_w_as-reorder_k1_1_6&amp=&crid=2CHDQTQO8MDHX&amp=&sprefix=deutch).
- [Official (?) Website for Deutche Automotive Connectors](https://www.te.com/en/products/connectors/automotive-connectors.html)

There are tutorials online on how to adapt existing positive-negative wire setups to the 2-pin Deutche DT connectors that are used with the car. It is essential that you ensure that you *can* power the mount over a Deutche DT connector before going to the competition.