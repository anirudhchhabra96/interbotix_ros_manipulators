import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

############################
# Load and parse the files #
############################

# --- Load trajectory tracking data ---
def load_tracking_data(filename):
    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    return data

tracking_data = load_tracking_data('tracking_record.txt')
time = tracking_data[:, 0]
commanded_x = tracking_data[:, 1]
commanded_y = tracking_data[:, 2]
commanded_z = tracking_data[:, 3]
actual_x = tracking_data[:, 4]
actual_y = tracking_data[:, 5]
actual_z = tracking_data[:, 6]

# --- Load pre-trajectory error data ---
def parse_pretrajectory_data(file_path):
    with open(file_path, 'r') as file:
        data = file.read()
    position_pattern = r'Current position: \([^)]+\), Error: ([0-9.]+)m'
    errors = []
    for match in re.finditer(position_pattern, data):
        error = float(match.group(1))
        errors.append(error)
    return np.array(errors)

pre_errors = parse_pretrajectory_data('temp.txt')

############################
# Pre-trajectory time array
############################
pre_time = np.linspace(0, 7, len(pre_errors))  # 7 seconds duration

############################
# Calculate trajectory error
############################
error_x = commanded_x - actual_x
error_y = commanded_y - actual_y
error_z = commanded_z - actual_z
error_magnitude = np.sqrt(error_x**2 + error_y**2 + error_z**2)

########################################
# Identify stabilization point in error
########################################
def find_stabilization_point(error, time, threshold_factor=0.5, window_size=3):
    steady_state_mean = np.mean(error[-len(error)//3:])
    threshold = steady_state_mean * (1 + threshold_factor)
    for i in range(len(error) - window_size):
        if all(error[i:i+window_size] < threshold):
            return i
    return len(error) // 4

stabilization_idx = find_stabilization_point(error_magnitude, time)
stabilization_time = time[stabilization_idx]

# Error segmentation
initial_error = error_magnitude[:stabilization_idx]
stabilized_error = error_magnitude[stabilization_idx:]

############################
# Simulate Pre-Trajectory Path
############################
start_point = np.array([0.079, -0.047, 0.165])  # Starting point as specified
end_point = np.array([commanded_x[0], commanded_y[0], commanded_z[0]])  # First commanded point

# Linear interpolation for pre-trajectory
pre_x = np.linspace(start_point[0], end_point[0], len(pre_errors))
pre_y = np.linspace(start_point[1], end_point[1], len(pre_errors))
pre_z = np.linspace(start_point[2], end_point[2], len(pre_errors))

############################
#     --- PLOTS ---        #
############################

### --- Figure 1: Full 3D Trajectory including pre-trajectory & tracking ---
fig1 = plt.figure(figsize=(12, 10))
ax1 = fig1.add_subplot(111, projection='3d')

# Plot pre-trajectory path
ax1.plot(pre_x, pre_y, pre_z, 'g--', label='Pre-Trajectory Path', linewidth=2)
ax1.scatter(pre_x[0], pre_y[0], pre_z[0], color='green', marker='o', s=80)  # Start
ax1.scatter(pre_x[-1], pre_y[-1], pre_z[-1], color='green', marker='x', s=80)  # End

ax1.set_zlim(0, 1)

# Trajectory tracking
ax1.plot(commanded_x, commanded_y, commanded_z, 'b-', linewidth=2, label='Commanded Trajectory')
ax1.plot(actual_x, actual_y, actual_z, 'r-', linewidth=2, label='Actual Trajectory')
ax1.scatter(actual_x[0], actual_y[0], actual_z[0], color='red', marker='o', s=80)
ax1.scatter(actual_x[-1], actual_y[-1], actual_z[-1], color='red', marker='x', s=80)

ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.legend()
ax1.grid(True)
ax1.view_init(elev=30, azim=135)

### --- Figure 2: Combined Error Plot ---
plt.figure(figsize=(12, 6))
steps = np.arange(len(pre_errors) + len(time))

# Combine errors
combined_error = np.concatenate((pre_errors, error_magnitude))

# Plot pre-trajectory errors
plt.plot(steps[:len(pre_errors)], pre_errors, 'g--', label='Pre-Trajectory Phase', linewidth=2)

# Plot trajectory tracking errors
plt.plot(steps[len(pre_errors):], error_magnitude, 'b-', label='Trajectory Tracking Error', linewidth=2)

# Highlight stabilization
plt.axvline(x=len(pre_errors) + stabilization_idx, color='r', linestyle='--',
            label=f'Stabilization at t={stabilization_time:.2f}s')

plt.axhspan(0, np.max(combined_error), xmin=0, xmax=len(pre_errors)/steps[-1],
            alpha=0.2, color='green', label='Pre-Trajectory Phase')

plt.axhspan(0, np.max(combined_error),
            xmin=len(pre_errors)/steps[-1], xmax=(len(pre_errors)+stabilization_idx)/steps[-1],
            alpha=0.2, color='red', label='Initial Phase')

plt.axhspan(0, np.max(combined_error),
            xmin=(len(pre_errors)+stabilization_idx)/steps[-1], xmax=1,
            alpha=0.2, color='blue', label='Stabilized Phase')

plt.xlabel('Step Number')
plt.ylabel('Error (m)')
plt.legend()
plt.grid(True)

### --- Figure 3: Zoomed-in Stabilized Trajectory ---
fig3 = plt.figure(figsize=(10, 8))
ax3 = fig3.add_subplot(111, projection='3d')
ax3.plot(commanded_x[stabilization_idx:], commanded_y[stabilization_idx:], commanded_z[stabilization_idx:], 
         'b-', linewidth=2, label='Commanded (Stabilized)')
ax3.plot(actual_x[stabilization_idx:], actual_y[stabilization_idx:], actual_z[stabilization_idx:], 
         'r-', linewidth=2, label='Actual (Stabilized)')
ax3.set_xlabel('X (m)')
ax3.set_ylabel('Y (m)')
ax3.set_zlabel('Z (m)')
ax3.legend()
ax3.grid(True)
ax3.view_init(elev=53, azim=132)

plt.tight_layout()
plt.show()

############################
#      Print Stats
############################
print(f"\nAnalysis after system stabilization (t > {stabilization_time:.2f}s):")
print(f"Mean tracking error: {np.mean(stabilized_error):.4f} m")
print(f"Max tracking error: {np.max(stabilized_error):.4f} m")
print(f"Min tracking error: {np.min(stabilized_error):.4f} m")
print(f"Standard deviation of error: {np.std(stabilized_error):.4f} m")
print(f"Percentage improvement: {100*(1 - np.mean(stabilized_error)/np.mean(initial_error)):.2f}%")


# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import re

# ############################
# # Load and parse the files #
# ############################

# # --- Load trajectory tracking data ---
# def load_tracking_data(filename):
#     data = np.genfromtxt(filename, delimiter=',', skip_header=1)
#     return data

# tracking_data = load_tracking_data('tracking_record.txt')
# time = tracking_data[:, 0]
# commanded_x = tracking_data[:, 1]
# commanded_y = tracking_data[:, 2]
# commanded_z = tracking_data[:, 3]
# actual_x = tracking_data[:, 4]
# actual_y = tracking_data[:, 5]
# actual_z = tracking_data[:, 6]

# # --- Load pre-trajectory error data ---
# def parse_pretrajectory_data(file_path):
#     with open(file_path, 'r') as file:
#         data = file.read()
#     position_pattern = r'Current position: \([^)]+\), Error: ([0-9.]+)m'
#     errors = []
#     for match in re.finditer(position_pattern, data):
#         error = float(match.group(1))
#         errors.append(error)
#     return np.array(errors)

# pre_errors = parse_pretrajectory_data('temp.txt')

# ############################
# # Pre-trajectory time array
# ############################
# pre_time = np.linspace(0, 7, len(pre_errors))  # 7 seconds duration

# ############################
# # Calculate trajectory error
# ############################
# error_x = commanded_x - actual_x
# error_y = commanded_y - actual_y
# error_z = commanded_z - actual_z
# error_magnitude = np.sqrt(error_x**2 + error_y**2 + error_z**2)

# ########################################
# # Identify stabilization point in error
# ########################################
# def find_stabilization_point(error, time, threshold_factor=0.5, window_size=3):
#     steady_state_mean = np.mean(error[-len(error)//3:])
#     threshold = steady_state_mean * (1 + threshold_factor)
#     for i in range(len(error) - window_size):
#         if all(error[i:i+window_size] < threshold):
#             return i
#     return len(error) // 4

# stabilization_idx = find_stabilization_point(error_magnitude, time)
# stabilization_time = time[stabilization_idx]

# # Error segmentation
# initial_error = error_magnitude[:stabilization_idx]
# stabilized_error = error_magnitude[stabilization_idx:]

# ############################
# #     --- PLOTS ---        #
# ############################

# ### --- Figure 1: Full 3D Trajectory including pre-trajectory & tracking ---
# fig1 = plt.figure(figsize=(12, 10))
# ax1 = fig1.add_subplot(111, projection='3d')

# # Dummy pre-trajectory path
# pre_x = np.linspace(actual_x[0], commanded_x[0], len(pre_errors))
# pre_y = np.linspace(actual_y[0], commanded_y[0], len(pre_errors))
# pre_z = np.linspace(actual_z[0], commanded_z[0], len(pre_errors))

# ax1.plot(pre_x, pre_y, pre_z, 'g--', label='Pre-Trajectory Path', linewidth=2)
# ax1.scatter(pre_x[0], pre_y[0], pre_z[0], color='green', marker='o', s=80)
# ax1.scatter(pre_x[-1], pre_y[-1], pre_z[-1], color='green', marker='x', s=80)
# ax1.set_zlim(0, 1)

# # Trajectory tracking
# ax1.plot(commanded_x, commanded_y, commanded_z, 'b-', linewidth=2, label='Commanded Trajectory')
# ax1.plot(actual_x, actual_y, actual_z, 'r-', linewidth=2, label='Actual Trajectory')
# ax1.scatter(actual_x[0], actual_y[0], actual_z[0], color='red', marker='o', s=80)
# ax1.scatter(actual_x[-1], actual_y[-1], actual_z[-1], color='red', marker='x', s=80)

# ax1.set_xlabel('X (m)')
# ax1.set_ylabel('Y (m)')
# ax1.set_zlabel('Z (m)')
# ax1.legend()
# ax1.grid(True)

# ### --- Figure 2: Combined Error Plot ---
# plt.figure(figsize=(12, 6))
# steps = np.arange(len(pre_errors) + len(time))

# # Combine errors
# combined_error = np.concatenate((pre_errors, error_magnitude))

# # Plot pre-trajectory errors
# plt.plot(steps[:len(pre_errors)], pre_errors, 'g--', label='Pre-Trajectory Phase', linewidth=2)

# # Plot trajectory tracking errors
# plt.plot(steps[len(pre_errors):], error_magnitude, 'b-', label='Trajectory Tracking Error', linewidth=2)

# # Highlight stabilization
# plt.axvline(x=len(pre_errors) + stabilization_idx, color='r', linestyle='--',
#             label=f'Stabilization at t={stabilization_time:.2f}s')

# plt.axhspan(0, np.max(combined_error), xmin=0, xmax=len(pre_errors)/steps[-1],
#             alpha=0.2, color='green', label='Pre-Trajectory Phase')

# plt.axhspan(0, np.max(combined_error),
#             xmin=len(pre_errors)/steps[-1], xmax=(len(pre_errors)+stabilization_idx)/steps[-1],
#             alpha=0.2, color='red', label='Initial Phase')

# plt.axhspan(0, np.max(combined_error),
#             xmin=(len(pre_errors)+stabilization_idx)/steps[-1], xmax=1,
#             alpha=0.2, color='blue', label='Stabilized Phase')

# plt.xlabel('Step Number')
# plt.ylabel('Error (m)')
# plt.legend()
# plt.grid(True)

# ### --- Figure 3: Zoomed-in Stabilized Trajectory ---
# fig3 = plt.figure(figsize=(10, 8))
# ax3 = fig3.add_subplot(111, projection='3d')
# ax3.plot(commanded_x[stabilization_idx:], commanded_y[stabilization_idx:], commanded_z[stabilization_idx:], 
#          'b-', linewidth=2, label='Commanded (Stabilized)')
# ax3.plot(actual_x[stabilization_idx:], actual_y[stabilization_idx:], actual_z[stabilization_idx:], 
#          'r-', linewidth=2, label='Actual (Stabilized)')
# ax3.set_xlabel('X (m)')
# ax3.set_ylabel('Y (m)')
# ax3.set_zlabel('Z (m)')
# ax3.legend()
# ax3.grid(True)

# plt.tight_layout()
# plt.show()

# ############################
# #      Print Stats
# ############################
# print(f"\nAnalysis after system stabilization (t > {stabilization_time:.2f}s):")
# print(f"Mean tracking error: {np.mean(stabilized_error):.4f} m")
# print(f"Max tracking error: {np.max(stabilized_error):.4f} m")
# print(f"Min tracking error: {np.min(stabilized_error):.4f} m")
# print(f"Standard deviation of error: {np.std(stabilized_error):.4f} m")
# print(f"Percentage improvement: {100*(1 - np.mean(stabilized_error)/np.mean(initial_error)):.2f}%")
