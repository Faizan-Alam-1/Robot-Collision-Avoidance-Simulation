import numpy as np
import matplotlib.pyplot as plt

# Parameters for simulation
total_time = 250  # Total time
step_time = 0.1  # duration of simuation
velocit_robot = 1.0  # Constant speed of robots
kappa_force = 10  #  attractive potential field
lambda_force = 10  # repulsive  potential field

# Initialize positions for two robots and their goal positions
pos_r1 = np.array([0.0, 0.0])  # Starting position of Robot 1
pos_r2 = np.array([5.0, 5.0])  # Starting position of Robot 2
goal_r1 = np.array([5.0, 5.0])  # Goal position of Robot 1
goal_r2 = np.array([0.0, 0.0])  # Goal position of Robot 2

# Lists to store positions for visualization
pos_r1_history = [pos_r1.copy()]
pos_r2_history = [pos_r2.copy()]

# Attractive force function
def attractive_force(current_pos, goal_pos, strength):
    distance = goal_pos - current_pos
    force = strength * distance / np.linalg.norm(distance)
    return force

#  repulsive  force function  to avoidance collision 
def repulsive_force(pos_r1, pos_r2, strength):
    distance_vector = pos_r2 - pos_r1
    distance = np.linalg.norm(distance_vector)
    if distance < 5:  # It will repulsive force trigger when distance will less then 5 
        direction = np.array([-distance_vector[1], distance_vector[0]])  # acting Perpendicular direction
        force =  strength * direction / (distance ** 2)
        return force
    return np.array([0.0, 0.0])


# Total force acting on R1
def total_force_on_robot_1():
    force_r1_att = attractive_force(pos_r1, goal_r1, kappa_force)
    force_r1_rep = repulsive_force(pos_r1, pos_r2, lambda_force)
    total_force_r1 = force_r1_att + force_r1_rep
    return total_force_r1

# Total force acting on R2
def total_force_on_robot_2():
    force_r2_att = attractive_force(pos_r2, goal_r2, kappa_force)
    force_r2_rep = repulsive_force(pos_r2, pos_r1, lambda_force)
    total_force_r2 = force_r2_att + force_r2_rep
    return total_force_r2
   
# Simulation loop
for time in range(total_time):

    total_force_r1 = total_force_on_robot_1()
    total_force_r2 = total_force_on_robot_2()
    #print(total_force_r2)


    # Magnitude of the total force on R1
    magnitude_total_force_r1 = np.linalg.norm(total_force_r1)
    # Magnitude of the total force on R2
    magnitude_total_force_r2 = np.linalg.norm(total_force_r2)
    #print(norm_r2)
    
    # Update the position
    if magnitude_total_force_r1 > 0:
        pos_r1 += velocit_robot * total_force_r1 * step_time / magnitude_total_force_r1
    if magnitude_total_force_r2 > 0:
        pos_r2 += velocit_robot * total_force_r2 * step_time / magnitude_total_force_r2

    # Store positions for visualization
    pos_r1_history.append(pos_r1.copy())
    pos_r2_history.append(pos_r2.copy())


# Convert positions history to numpy arrays for easy plotting
pos_r1_history = np.array(pos_r1_history)
pos_r2_history = np.array(pos_r2_history)


# Plotting the results
plt.figure(figsize=(8, 8))
plt.plot(pos_r1_history[:, 0], pos_r1_history[:, 1], 'o-', color='red', label="Robot 1 Path")
plt.plot(pos_r2_history[:, 0], pos_r2_history[:, 1], 'o-', color='green', label="Robot 2 Path")

plt.text(pos_r1_history[0, 0]+2, pos_r1_history[0, 1], "Start R1 (t=0)", fontsize=10, color="red", ha="right")
plt.scatter(*goal_r1, color='red', marker='*', s=100, label="Goal 1")
plt.scatter(*goal_r2, color='green', marker='*', s=100, label="Goal 2")
plt.text(pos_r2_history[0, 0], pos_r2_history[0, 1], "Start R2 (t=0)", fontsize=10, color="green", ha="right")

plt.title("Two Robots Avoiding Collision Using Vortex Potential Fields")
plt.xlabel("X position")
plt.ylabel("Y position")
plt.legend()
plt.grid()
plt.axis('equal')  # Equal scaling on both axes
plt.show()
