# Lunchy
*This is a new page which will see rapid changes over the next few weeks*  
Still needed in this section: breakdown of the design features in the current Lunchy design (spacing, materials, etc)  
## Design Process Logs
*Documentation wasn't done on V1-V3*

### LunchyV4
Lunchy 4 won't work as built, and new ideas need to be added at the beginning.
#### Why it won't work:
- It looks hella ugly.
- The vent should be split in 2 and moved to the sides. Hot air gathers more there, and for structural integrity 2 smaller ones are better.
- Dimensions are wrong for the camera as this year we are switching from the Zed 2 to the Zed 2i

#### What needs to be added.
- Sponsor wall
- Change to z2i camera (and maybe make that mount a separate part).
- Considering making an internal ribbed part holder.

### LunchyV5
#### Active fixes needed in V5:
- It looks hella ugly.
- The vent should be split in 2 and moved to the sides. Hot air gathers more there, and for structural integrity 2 smaller ones are better.
- Dimensions are wrong for the camera as this year we are switching from the Zed 2 to the Zed 2i


#### Goals for V5:
- Add sponsorship tile compatibility
- Add 1mm to each Lunchy dimenstion, then make the wall 3mm thick, finally cut back wall by 1mm on the smooth edges. 
- Divide the wall out to add the 3mm holes for sponsor wall.
- Add a computer frame that can fit within lunchy - "Skips"
- Accommodate Z2i in Skips and Lunchy cutout

#### Choices made in V5:
- Maximized creating fillets on any long straight corner - this reduces elephant footing and warping in the 3d printing process.
- I am concerned that the number of holes on each side might reduce the waterproofing, I know they will be covered with sponsors, but it is an extra point of failure.
- will create alternate Lunchy where each panel is one side of the case, then we can fuck with sponsors on that panel. Body stays the same each year, but panel gets reprinted.


*Now after Christmas trying to fix design, confident 5 would have printed, however I am unhappy with the sponsor tile solution.Want to try a picture frame solution so will create V6, hopefully reusing all same dimensions from V5 except wall thickness to 4mm (2mm back, 1mm plate, 1mm frame) to accommodate.*

### LunchyV6
#### Main changes:
- potentially an O-ring design might work printed out of TPU or used just around where the openings are. Biggest question is where they will go. Perhaps best if they go around where the holes go and we use screws with fasteners to ensure a tight fit to Lunchy.
- I also want to explore adding the camera mount as an attachment to the base plate.

#### Plan after reviewing logistics
- Having screws interface with the sponsor board 
- Do a picture frame design, covered in the Lunchy MS Whiteboard.
- Focus on adding camera mount to base plate secondly as should be easier concept to incorporate into Lunchy (but separate thing to Lunchy).

#### Still needed to change
- Need to add in wall positioning for modem casing to slot
- Need to add in screw holes for connecting base plate to lunchy body.


*(Note on naming of files - once I update a file from the previous model or remake it, the number changes to the current version. If the design ends up with 0 changes between version x and x+1, the name stays - product_x.)*

### LunchyV7
#### Main idea:
- After printing and testing of spacing, current Lunchy design is too small. Everything fits, but there is no clearance for wiring.
- For cabling, we will add a tuxedo-style back to V7 so the wires can run out and down (matching how the top design looks.
- For sponsor plates, the bottom piece was too flimsy and potentially unnecessary. Will attempt a design without it while using friction to keep the pieces in.
- Screws to attach Lunchy to Basey and Basey to Base Plate will now extrude as they were too hard to access going in from side. This will also help with Lunchy design for ADS keeping it easier to install.
- We will develop a separate test-bench style holder for the parts to be printed in PETG so they can stay together with/without Lunchy support (including Modem)
- The front access for the LiDAR cable will now be in Basey

#### Mid-Development log
- New naming scheme implemented :D.
- Tuxedo-style back added, decided that it doesn't need to go below where Lunchy already stops because we're simply protecting from water, not other elements.
- Removed extra cabling things. Yet to add new cabling for front to Basey.
- Left a small disconnect between tux and rest of box to allow compliant mechanism, I would fear random people operating it could snap, so gave it some built-in flex.
- Had to adjust Liddy to be 2mm longer because it was off in previous versions.
- Need to extend the flute (tuxedo) to be 2cm bigger, making it a 4cm extrusion, then will print and hopefully that's enough.

*(Note, Basey is my new unofficial term for the Lunchy base plate, distinguishing it from the actual base plate Lunchy as a whole will be mounted on, naturally the lid will now be Liddy).
