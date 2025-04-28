import os
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib as mpl

# Imports for latex plots
mpl.rcParams['text.usetex'] = True
mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['text.latex.preamble'] = r'\usepackage{amsmath}'

# ERP1 (complete) task durations
pp_durations = [13.41, 13.16, 8.53, 8.56, 8.79, 8.53]
TVP_durations = [0.83, 1.52, 2.21, 0.43, 1.63, 2.44]
human_durations = [10.0, 15.0, 10.0, 10.0, 15.0, 10.0, 15.0]
robot_plan = [
    r"$\textit{P$\&$P}_{12}$", r"$\textit{P$\&$P}_{11}$", r"$\textit{P$\&$P}_{13}$",
    r"$\textit{P$\&$P}_{23}$", r"$\textit{P$\&$P}_{22}$", r"$\textit{P$\&$P}_{21}$"
]
human_plan = [
    r"\textit{CreateBox}$_{A}$", r"\textit{Offline}", r"\textit{BringItems}$_{B}$",
    r"\textit{FillPallet}$_{C}$", r"\textit{Offline}", r"\textit{BringItems}$_{D}$", r"\textit{Offline}"
]
file_name = 'distance_test3_ERP1_complete.txt'

'''
# ERP2 (complete) task durations
pp_durations = [13.4, 10.48, 8.49, 8.56, 8.79, 8.55]
TVP_durations = [0.96, 1.48, 2.23, 0.2, 1.65, 2.46]
human_durations = [8.0, 7.0, 10.0, 15.0, 10.0, 10.0, 15.0]
robot_plan = [
    r"$\textit{P$\&$P}_{12}$", r"$\textit{P$\&$P}_{11}$", r"$\textit{P$\&$P}_{13}$",
    r"$\textit{P$\&$P}_{23}$", r"$\textit{P$\&$P}_{22}$", r"$\textit{P$\&$P}_{21}$"
]
human_plan = [
    r"\textit{BringItems}$_{B}$", r"\textit{Offline}", r"\textit{BringItems}$_{D}$",
    r"\textit{Offline}", r"\textit{FillPallet}$_{C}$", r"\textit{BringItems}$_{D}$", r"\textit{Offline}"
]
file_name = 'distance_test3_ERP2_complete.txt'
'''

# ERP2 (only time) task durations
''' 
pp_durations = [8.53, 10.55, 10.50, 8.56, 8.58, 8.54]
TVP_durations = [1.31, 1.94, 0.63, 2.13, 1.63, 0.79]
human_durations = [8.0, 7.0, 10.0, 15.0, 10.0, 10.0, 15.0]
robot_plan = [
    r"$\textit{P$\&$P}_{13}$", r"$\textit{P$\&$P}_{12}$", r"$\textit{P$\&$P}_{11}$",
    r"$\textit{P$\&$P}_{23}$", r"$\textit{P$\&$P}_{22}$", r"$\textit{P$\&$P}_{21}$"
]
human_plan = [
    r"\textit{BringItems}$_{B}$", r"\textit{Offline}", r"\textit{BringItems}$_{D}$",
    r"\textit{Offline}", r"\textit{FillPallet}$_{C}$", r"\textit{BringItems}$_{D}$", r"\textit{Offline}"
]
file_name = 'distance_test3_ERP2_time.txt'
'''

# Relative file paths
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
file_path = os.path.join(base_dir, 'data', file_name)

# Data containers
times = []
distances = []
items = []

# Constants
sampling_interval = 0.10  # seconds

# Read and parse the log
with open(file_path, 'r') as file:
    for idx, line in enumerate(file):
        if "distance is:" in line and "item is:" in line:
            try:
                parts = line.strip().split(',')
                dist = float(parts[0].split(":")[1].strip())
                item = int(parts[1].split(":")[1].strip())

                if dist != -1 and item != 6:
                    time = idx * sampling_interval
                    times.append(time)
                    distances.append(dist)
                    items.append(item)

            except (ValueError, IndexError):
                continue  # skip malformed lines

# Plot setup
plt.figure(figsize=(8, 5))
ax = plt.gca()

# Y axis bands (limits)
bands = [(0, 200), (200, 500), (500, 1000), (1000, 1500), (1500, float('inf'))]
band_colors = ['#d9d9d9', '#cce6ff', '#ccffcc', '#ffe0cc', '#f2ccff']  # stronger pastel tones
band_labels = [r'$\mathcal{K}\{5\}=4$', r'$\mathcal{K}\{4\}=3$',r'$\mathcal{K}\{3\}=2$', r'$\mathcal{K}\{2\}=1$', r'$\mathcal{K}\{1\}=0$']

# Plot bands with more visible fill
ymax = max(distances) * 1.05
for (ymin, ymax_local), color, label in zip(bands, band_colors, band_labels):
    if ymax_local == float('inf'):
        ymax_local = ymax
    ax.axhspan(ymin, ymax_local, facecolor=color, alpha=0.6, label=label)  # 💡 more opacity

# Draw data points
unique_items = sorted(set(items))
cmap = plt.get_cmap('tab10')
for i in unique_items:
    label = robot_plan[i]
    color = cmap(i % 10)
    x_vals = [t for t, it in zip(times, items) if it == i]
    y_vals = [d for d, it in zip(distances, items) if it == i]
    plt.scatter(x_vals, y_vals, label=label, color=color, s=10)

# Dashed horizontal lines
for y in [0, 200, 500, 1000, 1500]:
    plt.axhline(y=y, linestyle='--', color='gray', linewidth=0.8)

# Axis setup
plt.ylim(bottom=0)
plt.xlabel('Time (s)', fontsize = 20)
plt.ylabel(r'$\bar{d}_{\text{min}}$', fontsize = 20)
#plt.title(r'$\mathcal{T}_{h,2}, w_t=1, w_{\delta}=0$', fontsize=20)
plt.tick_params(axis='both', labelsize=17)  # Adjust size as needed

# Custom legend (combine robot plans and area bands)
handles, labels = ax.get_legend_handles_labels()
by_label = dict(zip(labels, handles))
plt.legend(by_label.values(), by_label.keys(), loc = 'upper left')

total_robot_time = sum(TVP_durations[i] + pp_durations[i] for i in range(len(robot_plan)))
ax.set_xlim([0, total_robot_time])

plt.grid(True, linestyle='--', alpha=0.8)
plt.tight_layout()
plt.show()
