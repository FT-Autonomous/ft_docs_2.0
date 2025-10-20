# Feedback from Competition.

## General Notes

* The judges love redundancies.
* The safey chart/Risk Assessment is very important
  * Need to include Grip of tires



## Design:

### 2024-2025 Feedback

#### Overall Concept:

- Good: Clear overview of software architecture displayed. Very good explanation from the presenters
- To improve on: Spend time on how they improved the software architecture to work better than the previous iteration
  
#### Selection of AI computer and integration of software:

- Good: Showed AI computer and integration of software. Also gave strengths and weaknesses for software
- To improve on: Show some AI computer specification to reinforce on the choice of computer
  
#### Selection of sensors:

- Good: Chose GPS + Camera + Lidar and were able to explain their use
- To improve on: Go over sensor options, strengths, and weaknesses if money was no issue. Go over synchronisation process between sensors
  
#### Generation of vehicle location information:

- Good: Talked about use of IMU and GPS sensor
- To improve on:
  

#### Path Planning:

- Good: Track cone space envelope created in in case one cone went missing and software will assume a ghost cone
- To improve on:
  

#### Vehicle Control:

- Good: Justified decision to use Pure Pursuit model
- To improve on: Could take into account vehicle physics if not following command -> Vehicle skidding -> Stop
  

#### Functional Safety:

- Good: Excellent chart showing functional safety plan
- To improve on: Add failure mitigation solution if a failure happens (current chart was showing how to prevent/fix solution)
  
### Bruno’s comments on Design:

One of the slides noted that in 2024, there was a lack of clearly validated improvements or metrics. In 2025, the team said this was addressed by documenting perception and control performance using quantitative metrics however nothing was shown in the slides. The presentation would be strengthened by comparing the current and previous software architectures, highlighting specific improvements, optimizations, or lessons learned. Demonstrating measurable gains in performance or reliability would make the evolution clearer.

The team developed a mechanically robust, waterproof sensor mount, but the presentation did not show the final design or explain the development process. Was it created using Design-Build-Test (traditional physical prototyping), Upfront Simulation (current industry practice for optimized, cost-effective design), or another approach?

Key improvements to highlight could include:
- Vibration and natural frequency (using Ansys Mechanical for example)
- Sensor visibility (using Ansys AVx for example or even a gaming engine)
- System static strength to weight ratio (using Ansys Mechanical for example)
- Material selection versus cost (using Ansys Granta MI tool for example)

Providing detailed AI computer specifications—such as CPU/GPU type, memory, power consumption, and benchmark performance—would reinforce the rationale for its selection. Discussing potential bottlenecks or limitations would also demonstrate deeper understanding.

Expanding on alternative sensor configurations, even hypothetically without budget constraints, would show awareness of the broader design space. Explaining how sensor data is synchronized, fused, and timestamped would clarify the reliability and precision of localization and perception. (See a similar comment in the Real World section.)

Including vehicle dynamics or physics-based modeling when executing commands would demonstrate a deeper understanding of control limitations. Discussing scenarios such as skidding, oversteer, or understeer, and how the system responds, would make the control strategy more comprehensive.

Another team failed to account for starting friction/torque and issued insufficient torque commands for vehicle startup. Explaining how your control algorithm addresses this would be beneficial.

Adding explicit failure mitigation strategies—detailing what happens if a sensor or controller fails—would complement the prevention-focused plan. Including redundancy, fallback modes, or safe-state strategies would demonstrate a complete functional safety approach.

| Assessed Area | Good | To improve on |
|---|---|---|
| Overall Concept: | Good: Clear overview of software architecture displayed. Very good explanation from the presenters | To improve on: Spend time on how they improved the software architecture to work better than the previous iteration |
| Selection of AI computer and integration of software: | Good: Showed AI computer and integration of software. Also gave strengths and weaknesses for software | To improve on: Show some AI computer specification to reinforce on the choice of computer |
| Selection of sensors: | Good: Chose GPS + Camera + Lidar and were able to explain their use | To improve on: Go over sensor options, strengths, and weaknesses if money was no issue. Go over synchronisation process between sensors |
| Generation of vehicle location information: | Good: Talked about use of IMU and GPS sensor | To improve on: |
| Path Planning: | Good: Track cone space envelope created in in case one cone went missing and software will assume a ghost cone | To improve on: |
| Vehicle Control: | Good: Justified decision to use Pure Pursuit model | To improve on: Could take into account vehicle physics if not following command -> Vehicle skidding -> Stop |
| Functional Safety: | Good: Excellent chart showing functional safety plan | To improve on: Add failure mitigation solution if a failure happens (current chart was showing how to prevent/fix solution) |

**Second Place, Presentaion**

### 2023-2024 Feedback

- Do you have any weightings on each of those different areas (slide 8)?
- Slide 8 Perception Diagram: Lots of questions related to this slide!
- If you had an unlimited budget what sensors would you choose?
- What are the main characteristics of each sensor that you look for?
- Is the VLP-16 an upgrade from last year’s Livox avia?
- Explain the sensor fusion pipeline
- What are the AI models used?
- How does the (perception) sensor fusion exactly work?
- In control what do you use in throttle?
- How was the integration of your software managed? / how do you go about the software
- development process.
- Can you explain your process for algorithm development?
- Functional safety: what happens if both the lidar and ZED fail?
- What about the PID controller? (we showed him the hidden slide)
- Why did you only used a P controller instead of a PID controller?
- We were asked to talk about the risks associated with our system.

**General notes:** most questions were perception based. There were five judges. 15 min timer
was set on the desk for the presentation and another 15 mins for questions. Judges took notes
avidly at the beginning of the presentation and as it went on, they seemed the note taking
decreased significantly. There was one judge who would take photos of some of the slides as
we went along. Example: the full system slide, the state machine events logic slide… Note:
judges asked follow-up questions from what was mentioned in answers to questions.

**Award Winning Presentation**

## Simulation:

### 2024-2025 Feedback

#### Dynamic events modelled:

- Good:
- To improve on: Mentioned 3 tracks when asked. Give examples of different tracks they used. More tracks to be generated
  
#### Selection of AI computer and integration of software:

- Good: Admitted at presentation start about need for knowledge building and have plans for next year to improve on it
- To improve on: Demonstrate simulation workflow and how it will be used to make decisions
  
#### Driver models, controls and path planning development:

- Good:
- To improve on: Demonstrate driver models, controls and path planning development techniques
  
#### Localisation and perception algorithm development:

- Good: Talked about use of UEFS simulation for testing their perception algorithm. Demonstrated example of sun effect/glare on perception
- To improve on: Use more simulation to generate synthetic data for perception
  
#### Vehicle and Sensor modeling:

- Good: Used previously captured camera data for perception development/improvement
- To improve on: Switch to commercially available solutions like IPG CarMaker (vehicle dynamics) and Ansys AVx (sensors). Scored 0 as were using Edinburgh’s open source model)
  
#### Correlation and validation:

- Good: Used UEFS to validate existing perception. Used Rigby for validation of perception but this ideally is more a design/prototype testing methodology
- To improve on: Use simulation the enhance perception, path planning and control algorithms development and validation
  
#### Mission planning and vehicle interface integration:

- Good: Talked about integration of their AMS into Rigby
- To improve on: Consider full digital twin of DDT vehicle compared to Rigby in order to get closer representation of FS-AI vehicle
  
#### Debugging tools, visualisation and data analysis tools:

- Good: KPIs used (e.g. amount of cones hit)
- To improve on: Implement debugging tools and explore/list KPIs used and data analysis workflow
  
#### Environment factors:

- Good: Talked about camera in ideal vs poor lighting conditions and how it affected perception
- To improve on: Talk about how noise can affect localisation and control. Also consider how it could affect lidar
  
### Bruno’s comments on Simulation:

* Provide detailed examples of the three tracks used during testing.

* Generate and test more track variations, including scenarios with unexpected obstacles or changes in track layout, to better evaluate algorithm robustness.

* Include visualizations of track variations in presentations to clearly show coverage of different dynamic conditions.

* Demonstrate a clear simulation workflow, showing how simulations inform software decisions and vehicle behavior before real-world testing.

* Include examples of how simulation outputs guide AI computer selection or integration choices.

* Provide demonstrations or visualizations of driver models in operation.

* Show techniques used in path planning development, including how control algorithms respond to dynamic obstacles or missing track elements.

* Highlight improvements made to control strategies compared to previous iterations.

* Use more simulation to generate synthetic datasets for perception testing, such as varying lighting, weather, and dynamic obstacles.

* Include quantitative evaluation metrics (accuracy, precision, recall) to show algorithm performance across simulated scenarios.

* Extend discussion of environmental effects to include noise, sensor interference, and other real-world perturbations.

* Consider how environmental factors affect lidar, GPS, IMU, and sensor fusion outputs.

* Include mitigation strategies for adverse conditions (e.g., filtering, calibration, redundant sensors).

* Simulation should be used to develop your algorithms before gaining access to the vehicle.

* Simulation results should be analysed and leveraged to refine your design choices. The presentation should clearly explain how this was done using debugging, visualisation and data analysis tools and how it influenced the final design (there should be a very close connection between Design and Simulation parts).

* The Static Event does not expect teams to develop their own driving simulator. This was unclear in the 2024 rules but clarified in the 2025 rules.

* Teams using UEFS will score lower than those using other tools.

* Teams are encouraged to use commercially available solutions such as IPG CarMaker (Vehicle Dynamics) and Ansys AVx (Physics-Based Sensor Simulation). Licenses can be provided free of charge to teams actively using these tools as part of an Academic Programme (Ansys) or via special arrangements (IPG).

* CarMaker and AVx include a connector that allows cosimulation between the two products.

* AVx can stream data via Python, ROS1, or ROS2, with sample code provided for adaptation to team-specific perception tools.

* The default version of CarMaker (used by the AVx connector) does not support ROS. Teams can instead use the CarMaker Python API to send control commands and retrieve data (e.g., GPS), or use the special CarMaker ROS edition without AVx.

* Using Ansys AVx requires an NVIDIA RTX card. While gaming RTX cards may work, AVx was not developed or tested on these, so any issues encountered on GeForce RTX cards are unsupported.

| Assessed Area | Good | To improve on |
|---|---|---|
| Dynamic events modelled: | Good: | To improve on: Mentioned 3 tracks when asked. Give examples of different tracks they used. More tracks to be generated |
| Selection of AI computer and integration of software: | Good: Admitted at presentation start about need for knowledge building and have plans for next year to improve on it | To improve on: Demonstrate simulation workflow and how it will be used to make decisions |
| Driver models, controls and path planning development: | Good: | To improve on: Demonstrate driver models, controls and path planning development techniques |
| Localisation and perception algorithm development: | Good: Talked about use of UEFS simulation for testing their perception algorithm. Demonstrated example of sun effect/glare on perception | To improve on: Use more simulation to generate synthetic data for perception |
| Vehicle and Sensor modeling: | Good: Used previously captured camera data for perception development/improvement | To improve on: Switch to commercially available solutions like IPG CarMaker (vehicle dynamics) and Ansys AVx (sensors). Scored 0 as were using Edinburgh’s open source model) |
| Correlation and validation: | Good: Used UEFS to validate existing perception. Used Rigby for validation of perception but this ideally is more a design/prototype testing methodology | To improve on: Use simulation the enhance perception, path planning and control algorithms development and validation |
| Mission planning and vehicle interface integration: | Good: Talked about integration of their AMS into Rigby | To improve on: Consider full digital twin of DDT vehicle compared to Rigby in order to get closer representation of FS-AI vehicle |
| Debugging tools, visualisation and data analysis tools: | Good: KPIs used (e.g. amount of cones hit) | To improve on: Implement debugging tools and explore/list KPIs used and data analysis workflow |
| Environment factors: | Good: Talked about camera in ideal vs poor lighting conditions and how it affected perception | To improve on: Talk about how noise can affect localisation and control. Also consider how it could affect lidar |
  
### 2023-2024 Feedback

- What are the most important aspects in terms of transferability to the real world? / in terms of reproducing what would happen in the real world?
- Why the EUFS sim? (and not IPG for example)
- Does the EUFS sim give you dynamics of the vehicle? Ans: Internally: bicycle model
- Can you fully verify the dynamics of the car in simulation like we can you can in real life?
- What about the sensors?
- How would you get the simulator closer to a one to one in terms of real world?
- What was the output from perception?
- How are you extracting your data for simulation? Ans: bags!
- Did we conduct sensitivity studies to see what factors the simulation is sensitive to? Ans: Wet track v dry track yes but fogginess no.
- Is the control similar to how you implement it in the DDT? / Control outputs? Ans: mentioned that we run the same stack on both the rig and the DDT car
- Why choose scale model (i.e., the rig) over a full one? Ans: Money limited!
- What if money wasn’t an issue?
- What onboard computer have we looked at?
- What do you use for debugging? (I.e. For point cloud etc.) Ans: EUFS sim
- Are you planning on going into the next class, ADS?
- Suggestions as to how IMeche can improve the FSAI experience for students in the future? Ans: More testing time! & mentioned in-car PC

**General notes:** Heavy and intensive questioning, judges very interested in why the EUFS
sim was used instead of IPG.


## Real World:

### 2024-2025 Feedback

#### Interactions with existing road infrastructure such as road signs, excluding any V2I:

- Good: Excellent overview of interactions with infrastructure. Car to car communication
- To improve on:
  

#### Other road users including vehicles and pedestrians and other obstacles:

- Good: Discussed vulnerable road users and interaction of vehicles with human drivers
- To improve on: Consider different kind of vehicles (cars, tractors, etc…) and road users they might encounter (child, cat, sheep, horse rider, etc…)
  

#### Functional safety requirements for autonomous vehicles:

- Good: Talked about hardware redundancy and cybersecurity
- To improve on: Knowledge of standards for OEMS (NCAP, etc…)
  

#### Dealing with weather e.g. snow, fog and rain:

- Good: Very good overview of weather effect on each sensor type and how to overcome inclement weather
- To improve on:
  

#### Wider philosophy and socioeconomic consequences of a driverless world including ethical issues:

- Good: Covered ethical, socioeconomic and legal impact
- To improve on: Think about seeing the topic from an automotive manufacturer point of view rather than just end user. Discuss ethics of autonomous car avoidance choices (autonomous car choosing between driving over a child or a grandmother)
  

#### Case Study: Is the world ready for Level 4 autonomous vehicles deployed on public road?
Legislation, society and technology readiness for driver-out vehicles.

- Good: Reviewed legislation, society and infrastructure for Denmark and Singapore
- To improve on: Consider using countries with major differences (e.g. Denmark vs India) where issues would be different (different driving styles, drivers following -or not following- highway code, etc...). Consider a more global in-depth study that can answer the question ‘Is the world ready for Level4 autonomous vehicles deployed on public road?’ rather than comparing 2 countries only. If the answer is no to the question, maybe suggest other places where autonomous vehicles could be already accepted (mining operations, logistical centres…)
  

### Bruno’s comments on Real World:

The Case Study alone accounts for half of the total points in the Real World section.

Teams that perform well in the Real World section usually do well in the Case Study as well as in the other topics.

In the case of Trinity College, you performed well in the non–Case Study portion but lost valuable points on the Case Study, which could have benefited from more substance to clearly demonstrate a Level 4 Autonomous Vehicles perspective and its implications for the world.

One of the aims of this section is for teams to demonstrate their understanding of how car manufacturers configure sensors and autonomous systems to comply with global and local regulations.

For example, criteria used to evaluate the best sensor combination

* Object detection accuracy
* Range and field of view
* Cost and power consumption
* Performance in poor weather
* Ease of integration
* Redundancy and safety

Examples of regulations that manufacturers must comply with to ensure vehicle safety and reassure the end users:

* Crash Safety Standards
* FMVSS (Federal Motor Vehicle Safety Standards, USA) – e.g., crashworthiness, seatbelt requirements, airbag deployment.
* ECE Regulations (Europe) – e.g., ECE R94 (frontal impact), R95 (side impact), R129 (child restraints).
* Vehicle Lighting & Visibility
* FMVSS 108 / ECE R48 & R112 – requirements for headlights, brake lights, turn signals, and reflectors to ensure visibility.
* Autonomous & Advanced Driver Assistance Systems (ADAS) Compliance
* ISO 26262 – functional safety standard for automotive electrical and electronic systems.
* UNECE WP.29 / UN R157 – regulations for Automated Lane Keeping Systems (ALKS) and autonomous driving functions.
* Emissions & Environmental Safety
* Euro 6 / EPA Standards – ensure vehicles meet emissions limits.
* REACH / RoHS – safe chemical use in vehicle components.
* Cybersecurity & Data Privacy
* ISO/SAE 21434 – standard for cybersecurity in road vehicles.
* GDPR / Local data protection laws – for vehicles collecting personal data.
* Tire, Brakes, and Mechanical Safety
* UNECE R30 / R90 – tire and braking system requirements.
* FMVSS 135 / 571 – brake system standards for passenger cars.
* Pedestrian & Vulnerable Road User Safety
* ECE R127 / Euro NCAP Pedestrian Safety – front*end design to reduce pedestrian injury in collisions.
* Recalls and Post*Market Safety
* Regulations requiring manufacturers to report and remedy defects (e.g., NHTSA recall rules in the US, RAPEX in Europe).

| Assessed Area | Good | To improve on |
|---|---|---|
| Interactions with existing road infrastructure such as road signs, excluding any V2I: | Good: Excellent overview of interactions with infrastructure. Car to car communication | To improve on: |
| Other road users including vehicles and pedestrians and other obstacles: | Good: Discussed vulnerable road users and interaction of vehicles with human drivers | To improve on: Consider different kind of vehicles (cars, tractors, etc…) and road users they might encounter (child, cat, sheep, horse rider, etc…) |
| Functional safety requirements for autonomous vehicles: | Good: Talked about hardware redundancy and cybersecurity | To improve on: Knowledge of standards for OEMS (NCAP, etc…) |
| Dealing with weather e.g. snow, fog and rain: | Good: Very good overview of weather effect on each sensor type and how to overcome inclement weather | To improve on: |
| Wider philosophy and socioeconomic consequences of a driverless world including ethical issues: | Good: Covered ethical, socioeconomic and legal impact | To improve on: Think about seeing the topic from an automotive manufacturer point of view rather than just end user. Discuss ethics of autonomous car avoidance choices (autonomous car choosing between driving over a child or a grandmother) |
| Case Study: Is the world ready for Level 4 autonomous vehicles deployed on public road? Legislation, society and technology readiness for driver-out vehicles. | Good: Reviewed legislation, society and infrastructure for Denmark and Singapore | To improve on: Consider using countries with major differences (e.g. Denmark vs India) where issues would be different (different driving styles, drivers following -or not following- highway code, etc...). Consider a more global in-depth study that can answer the question ‘Is the world ready for Level4 autonomous vehicles deployed on public road?’ rather than comparing 2 countries only. If the answer is no to the question, maybe suggest other places where autonomous vehicles could be already accepted (mining operations, logistical centres…) |
