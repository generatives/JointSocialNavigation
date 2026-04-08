import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("/workspaces/JointSocialNavigation/results/metrics_steps_wide_hallway_1.csv")
t = df["time_stamps.1"] if "time_stamps.1" in df.columns else df["time_stamps"]

fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

axs[0].plot(t, df["avg_distance_to_closest_person"])
axs[0].set_ylabel("Closest person dist (m)")

axs[1].plot(t, df["avg_robot_linear_speed"], label="linear")
axs[1].plot(t, df["avg_robot_angular_speed"], label="angular")
axs[1].plot(t, df["avg_acceleration"], label="accel")
axs[1].legend()
axs[1].set_ylabel("Motion")

axs[2].plot(t, df["social_force_on_robot"], label="social")
axs[2].plot(t, df["obstacle_force_on_robot"], label="obstacle")
axs[2].legend()
axs[2].set_ylabel("Forces")
axs[2].set_xlabel("Time (s)")

plt.tight_layout()
plt.show()
