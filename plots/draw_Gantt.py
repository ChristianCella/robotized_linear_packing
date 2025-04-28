import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib as mpl

# Imports for LaTeX plots
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
'''

# Unique human tasks in order
unique_human_tasks = list(dict.fromkeys(human_plan))
human_y_start = 0
robot_y_start = len(unique_human_tasks) + 1

fig, ax = plt.subplots(figsize=(8, 5))

# Plot robot tasks
robot_start_time = 0
for i, task in enumerate(robot_plan):
    tvp = TVP_durations[i]
    pp = pp_durations[i]
    ax.barh(robot_y_start + i, tvp, left=robot_start_time, color='lightcoral', edgecolor='crimson')
    ax.barh(robot_y_start + i, pp, left=robot_start_time + tvp, color='indianred', edgecolor='crimson')
    robot_start_time += tvp + pp

# Plot human tasks
human_start_time = 0
for task, duration in zip(human_plan, human_durations):
    y_pos = unique_human_tasks.index(task)
    ax.barh(y_pos, duration, left=human_start_time, color='cornflowerblue', edgecolor='mediumblue')
    human_start_time += duration

# Y-axis labels
y_labels = unique_human_tasks + robot_plan
y_positions = list(range(len(unique_human_tasks))) + [robot_y_start + i for i in range(len(robot_plan))]
ax.set_yticks(y_positions)
ax.set_yticklabels(y_labels)

# Title and labels
ax.set_xlabel('Time (s)', fontsize=20)
# Set tick label font sizes
ax.tick_params(axis='x', labelsize=17)
ax.tick_params(axis='y', labelsize=12)
#ax.set_title(r'$\mathcal{T}_{h,2}, w_t=1, w_{\delta}=0$', fontsize=20)

# Custom legend
legend_handles = [
    mpatches.Patch(color='lightcoral', label=r'$\mathcal{T}_{r,\text{TVP}}$'),
    mpatches.Patch(color='indianred', label=r'$\mathcal{T}_{r,\textit{P$\&$P}}$'),
    mpatches.Patch(color='cornflowerblue', label=r'$\mathcal{T}_h$')
]
ax.legend(handles=legend_handles, fontsize=12)

# Limit x-axis to total robot duration
total_robot_time = sum(TVP_durations[i] + pp_durations[i] for i in range(len(robot_plan)))
ax.set_xlim([0, total_robot_time])

# Final formatting
plt.grid(True, axis='x', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.show()
