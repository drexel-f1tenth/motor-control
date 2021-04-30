import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
import sys


data = pd.read_csv(sys.argv[1])
data.drop(range(max(0, data['setpoint'].ne(0).idxmax() - 10)))
data['t_ms'] = [float(i) * (1000.0 / 64.0) for i in range(len(data))]

y_labels = ['setpoint', 'rps']
for label in y_labels:
  plt.plot(data['t_ms'], data[label])

plt.xlabel('t (ms)')
plt.ylabel('value (RPS)')
plt.legend(y_labels)

plt.tight_layout()
plt.savefig("pid-plot.png", format='png', dpi=500)
