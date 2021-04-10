import matplotlib.pyplot as plt
import sys

file_name = sys.argv[1]
setpoint, value = [], []
with open(file_name) as f:
  for line in f:
    if 'RPS:' not in line or '[INFO]' not in line: continue
    csv = line.strip().replace('"', '').replace(',', '').split(' ')
    if len(csv) < 6: break
    setpoint.append(int(csv[3]))
    value.append(float(csv[4]))

t = [i * (1000.0 / 64.0) for i in range(len(setpoint))]

plt.plot(t, setpoint)
plt.plot(t, value)
plt.show()
