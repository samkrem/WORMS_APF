# Walking Oligimeric Robotic Mobility System (WORMS) Artificial Potential Field Planner
## Descriptions: ##
*  Under the AeroAstro Space Resources Workshop, the research proposal for the Walking, Oligomeric, Robotic Mobility System (WORMS) was selected as a finalist by NASA’s 2022 BIG Idea Challenge and received $175K in funding for the project. WORMS is a modular architecture where the robotic mobility capability emerges from the swarm-like integration of a small (“oligomeric”) set of nearly-identical, articulating “worms.”. The WORMS APF uses inverse kinematics, attractive forces, and repulsive forces to guide any volume from its start state to a target state.
## Main Objectives:
* Guide a WORM, or robotic arm to its target state in an energy efficient manner
* Develop a realistic lunar simulation space 
## Contributions: $$
* Coded an Artificial Potential Field Planner (APF) in MATLAB that utilizes inverse kinematics, attractive forces, and repulsive forces to guide a robot through obstacles to its target.
* Developed attractive and repulsive algorithms for a volume representing a WORM.
* Coded a simulation environment representing a WORM on the moon for the APF in MATLAB
* Co-authoring a paper called "Energy Efficient Gait Adaptation for Legged Multi-Agent Robotic Systems Using Artificial Potential Fields"

<image width="50%" src="https://github.com/samkrem/WORMS_APF/blob/main/Images/Env_Img_1.png"></image>
<image width="50%" src="https://github.com/samkrem/WORMS_APF/blob/main/Images/Env_Img_2.png"></image>

## File Structure of APF_3D: ##
* EnvInfAvg.m is the program to run the entire planner and references other files in the APF_3D directory
* create_rock.m creates rock like objects
* potential_attraction.m is the function for the attractive force algorithm
* dist_factor: function for distance
* diff_distance_factor.m is the function for the gradient of distance
## Structure of EnvInfAvg.m: ##
* The most outer loop represents every iteration of a movement
* The middle loop loops throught every point defining a WORMS volume
* The inner loop traverses every obstacle to calculate the attractive and repulsive forces
* After, artifical forces,
## Algorithms, variables and calculations used for APF(in EnvInfAvg.m and other files): ##
<image width="100%" src="https://github.com/samkrem/WORMS_APF/blob/main/Images/APF_Algorithms.png"></image>
* F_avg: Average of the forces of multiple points that make up a volume
* F: The sum of the repulsive and attractive forcces
* F_att: The attractive force so its the distance between the current position and goal times some constant
* Frep: The sum of two individual repulsive forces
* F_rep1: Based on a repulsive constant, distance between the goal and the current point, and zeta which is a constant based on the size of an obstacle, 
* F_rep2: Similar to F_rep1 but it also factors in the gradient of the distance for accuracy measurements and the constant n
* dist_factor: formula based on distance 
* diff_distance_factor: Gradient of dist_factor
* rou: the xy plane distance between the point and obstacle
* d_rou: a unit vector pointing from the current point to the obstacle
## Future plans: ##
* Increase effeciency and consistency of APF even more
* Utilize calculated robot volume when it is travelling between two obstacles to avoid robot getting trapped.




