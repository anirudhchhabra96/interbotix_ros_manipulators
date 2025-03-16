import matplotlib.pyplot as plt
import re
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Function to parse the data file
def parse_data(file_path):
    with open(file_path, 'r') as file:
        data = file.read()
    
    # Extract the start position
    start_match = re.search(r'Moving to start position: \(([^)]+)\)', data)
    start_position = None
    if start_match:
        coords = start_match.group(1).split(', ')
        start_position = (float(coords[0]), float(coords[1]), float(coords[2]))
    
    # Extract all current positions and errors
    position_pattern = r'Current position: \(([^)]+)\), Error: ([0-9.]+)m'
    positions = []
    errors = []
    
    for match in re.finditer(position_pattern, data):
        coords = match.group(1).split(', ')
        position = (float(coords[0]), float(coords[1]), float(coords[2]))
        error = float(match.group(2))
        positions.append(position)
        errors.append(error)
    
    return start_position, positions, errors

# Parse the data
start_position, positions, errors = parse_data('temp.txt')

# Extract x, y, z coordinates for plotting
x_coords = [pos[0] for pos in positions]
y_coords = [pos[1] for pos in positions]
z_coords = [pos[2] for pos in positions]

# Create a colormap based on error values
colors = errors
norm = plt.Normalize(min(errors), max(errors))

# Create the 3D plot
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Plot the trajectory with color based on error
scatter = ax.scatter(x_coords, y_coords, z_coords, c=colors, cmap='viridis', 
                     s=50, alpha=0.8, edgecolors='w', linewidths=0.5)

# Add the start position as a special marker
if start_position:
    ax.scatter([start_position[0]], [start_position[1]], [start_position[2]], 
               color='red', s=100, marker='*', label='Start Position')

# Add a line connecting all points to show the trajectory
ax.plot(x_coords, y_coords, z_coords, 'gray', linestyle='--', alpha=0.5)

# Add a colorbar to show the error scale
cbar = fig.colorbar(scatter, ax=ax, pad=0.1)
cbar.set_label('Error (m)', rotation=270, labelpad=20)

# Set labels and title
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('Pre-trajectory Motion', fontsize=16)

# Add legend
ax.legend()

# Adjust the view for better visibility
ax.view_init(elev=30, azim=45)

# Add a text annotation for the final position and error
final_pos = positions[-1]
final_error = errors[-1]
ax.text(final_pos[0]+0.02, final_pos[1]+0.02, final_pos[2]-0.004, 
        f"Final Error: {final_error}m", color='black', fontsize=10)

plt.tight_layout()
plt.savefig('pre_trajectory_motion.png', dpi=300)
plt.show()