from matplotlib import pyplot as plt
import numpy as np

labels = ["Standalone Closed-Loop", "VICON Open-Loop", "VICON Closed-Loop"]

# Data taken from process_vicon.py and process_standlone.py for YPRT tests
means = np.array([[0.00576384, 0.00956543, 0.00481988],
         [0.04246944, 0.02740343, 0.06014794],
         [0.29069963, 0.172689, 0.21060626]])
lower_err = np.array([[0.0004, 0.0007, 0.0005],
            [0.00302529, 0.00174251, 0.00886812],
            [0.00790201, 0.01116633, 0.03076867]])
upper_err = np.array([[0.015, 0.0259, 0.0143],
            [0.11075141, 0.06685809, 0.11605694],
            [0.85395859, 0.39270808, 0.88736489]])

rel_lower_err = means - lower_err   # mean - lower percentile
rel_upper_err = upper_err - means   # upper percentile - mean


# Plotting grouped bars
x = np.arange(3)
bar_width = 0.25

fig, ax = plt.subplots(figsize=(10, 6))

components = ["Y", "P", "R"]  # name your 3 error components

for j in range(3):
    ax.bar(
        x + j * bar_width,
        means[:, j],
        bar_width,
        label=components[j],
        yerr=[rel_lower_err[:, j], rel_upper_err[:, j]],
        capsize=5
    )

ax.set_xticks(x + bar_width)
ax.set_xticklabels(labels)
ax.set_ylabel("Error")
ax.set_title("Mean Error with 95% Percentile Intervals for YPRT tests")
ax.legend()
plt.tight_layout()
plt.show()