# Walking Oligimeric Robotic Mobility System (WORMS) Artificial Potential Field Planner
## Descriptions: ##
* Developeed simulation space representing a rocky lunar surface. Under the AeroAstro Space Resources Workshop, the research proposal for the Walking, Oligomeric, Robotic Mobility System (WORMS) was selected as a finalist by NASA’s 2022 BIG Idea Challenge and received $175K in funding for the project. WORMS is a modular architecture where the robotic mobility capability emerges from the swarm-like integration of a small (“oligomeric”) set of nearly-identical, articulating “worms.”. The WORMS APF uses inverse kinematics, attractive forces, and repulsive forces to guide a volume (in this case a robotic arm) from its start state to a target state.
## Contributions: $$
* Coded an Artificial Potential Field Planner (APF) in MATLAB that utilizes inverse kinematics, attractive forces, and repulsive forces to guide a robot through obstacles to its target.
* Developed attractive and repulsive algorithms for a volume representing a WORM.
* Coded a simulation environment representing a WORM on the moon for the APF in MATLAB
* Co-authoring a paper called "Energy Efficient Gait Adaptation for Legged Multi-Agent Robotic Systems Using Artificial Potential Fields"
* Assembled and disassembled WORMS from its pallet.
## File Structure of APF_3D: ##
* EnvInfAvg.m is the program to run the entire planner and references other files in the APF_3D directory
* create_rock.m creates rock like objects
* potential_attraction.m is the function for the attractive force algorithm
* diff_distance_factor.m is the function for the gradient of distance
## Structure of EnvInfAvg.m: ##
* The most outer loop represents every iteration of a movement
* The middle loop loops throught every point defining a WORMS volume
* The inner loop traverses every obstacle to calculate the attractive and repulsive forces
* After, artifical forces,
## Algorithms and Calculations used for APF(in EnvInfAvg.m and other files): ##
<image width="50%" src="https://github.com/samkrem/ACL_UWB_SLAM/blob/main/images/Metronome_Transformation_Diagram.png"></image>


## Future plans: ##
* Increase effeciency and consistency of APF even more
* Utilize calculated robot volume when it is travelling between two obstacles to avoid robot getting trapped.




