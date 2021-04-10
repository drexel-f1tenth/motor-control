import matplotlib.pyplot as plt
import pandas as pd
import sys

file_name = sys.argv[1]
data = pd.read_csv(file_name)

data['t_ms'] = [i * (1000.0 / 64.0) for i in range(len(data))]

fig, ax = plt.subplots(2, 1, sharex=True)

ax[0].plot(data['t_ms'], data['setpoint'])
ax[0].plot(data['t_ms'], data['RPS'])
ax[0].legend(['setpoint (throttle %)', 'RPS'])

ax[1].plot(data['t_ms'], data['acc_x'])
ax[1].plot(data['t_ms'], data['acc_y'])
ax[1].legend(['acceleration X', 'acceleration Y'])
ax[1].set_xlabel('t (ms)')

plt.tight_layout()
plt.show()
