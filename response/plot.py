import matplotlib.pyplot as plt
import sys

file_name = sys.argv[1]
setpoint, value, acc_x, acc_y = [], [], [], []
with open(file_name) as f:
  for line in f:
    if '[INFO]' not in line: continue
    csv = line.strip().replace('"', '').replace(',', '').split(' ')
    if 'RPS:' in line:
      if len(csv) < 6: break
      setpoint.append(float(csv[3]) / 0.6)
      value.append(float(csv[4]))
    elif 'IMU:' in line:
      if len(csv) < 5: break
      acc_x.append(float(csv[3]))
      acc_y.append(float(csv[4]))


t = [i * (1000.0 / 64.0) for i in range(len(setpoint))]

fig, ax = plt.subplots(2, 1, sharex=True)

ax[0].plot(t, setpoint)
ax[0].plot(t, value)
ax[0].legend(['setpoint (throttle %)', 'RPS'])

ax[1].plot(t, acc_x)
ax[1].plot(t, acc_y)
ax[1].legend(['acceleration X', 'acceleration Y'])
ax[1].set_xlabel('t (ms)')

plt.tight_layout()
plt.show()
