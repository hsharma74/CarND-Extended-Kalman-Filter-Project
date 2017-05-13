import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns


# read-in the radar output
df_radar = pd.read_csv('../build/myoutput_radar.txt')
# plt radar estimated values against ground truth
fig = plt.figure(figsize=(12,8))
plt.plot(df_radar['est_px'], df_radar['est_py'], alpha=0.4, color='b', marker='o', label='radar_estimate')
plt.plot(df_radar['gt_px'], df_radar['gt_py'], color='r', marker='.', linewidth=2.0, label='radar_truth')
plt.xlabel('px')
plt.ylabel('py')
plt.title('Radar-only EKF estimates vs. Ground truth')
plt.legend(loc='best')
#plt.show()
plt.savefig('./radar_est_vs_gt.jpg')


# read-in the laser output
df_laser = pd.read_csv('../build/myoutput_laser.txt')
# plt radar estimated values against ground truth
fig = plt.figure(figsize=(12,8))
plt.plot(df_laser['est_px'], df_laser['est_py'], alpha=0.4, color='b', marker='o', label='laser_estimate')
plt.plot(df_laser['gt_px'], df_laser['gt_py'], color='r', marker='.', linewidth=2.0, label='laser_truth')
plt.xlabel('px')
plt.ylabel('py')
plt.title('Laser-only EKF estimates vs. Ground truth')
plt.legend(loc='best')
#plt.show()
plt.savefig('./laser_est_vs_gt.jpg')


# read-in the combined laser and radar output
df_comb = pd.read_csv('../build/myoutput_combined.txt')
# plt radar estimated values against ground truth
fig = plt.figure(figsize=(12,8))
plt.plot(df_comb['est_px'], df_comb['est_py'], alpha=0.4, color='b', marker='o', label='combined_estimate')
plt.plot(df_comb['gt_px'], df_comb['gt_py'], color='r', marker='.', linewidth=2.0, label='combined_truth')
plt.xlabel('px')
plt.ylabel('py')
plt.title('Combined (radar + laser) EKF estimates vs. Ground truth')
plt.legend(loc='best')
#plt.show()
plt.savefig('./combined_est_vs_gt.jpg')




