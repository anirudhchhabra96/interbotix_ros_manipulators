import matplotlib.pyplot as plt
import re

# Function to parse the data file
def parse_data(file_path):
    with open(file_path, 'r') as file:
        data = file.read()
    
    # Extract all errors
    position_pattern = r'Current position: \([^)]+\), Error: ([0-9.]+)m'
    errors = []
    
    for match in re.finditer(position_pattern, data):
        error = float(match.group(1))
        errors.append(error)
    
    return errors

# Parse the data
errors = parse_data('temp.txt')

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(range(len(errors)), errors, 'b-', linewidth=2)
plt.plot(range(len(errors)), errors, 'ro', markersize=4)

# Set labels and title
plt.xlabel('Step Number')
plt.ylabel('Error (m)')
# plt.title('Pre-trajectory Motion: Error vs. Steps', fontsize=14)

# Add grid
plt.grid(True, alpha=0.3)

# Highlight the final error point
plt.plot(len(errors)-1, errors[-1], 'go', markersize=8)

# Add annotation for the final error
plt.annotate(f'Final Error: {errors[-1]}m', 
             xy=(len(errors)-1, errors[-1]),
             xytext=(len(errors)-10, errors[-1]+0.05),
             arrowprops=dict(facecolor='black', shrink=0.05, width=1.5),
             fontsize=10)

# plt.legend()
plt.tight_layout()
plt.savefig('error_progression.png', dpi=300)
plt.show()