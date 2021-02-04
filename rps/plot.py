import plotly.express as px
import pandas as pd
import sys

log_file = sys.argv[1]

data = pd.DataFrame(columns=["t_ms", "rps", "setpoint", "adjust"])
t = 0
with open(log_file) as f:
  for line in f:
    if line.startswith("data:"):
      csv = line.replace('"', "").replace("\n", "").split(" ")
      t += 1000.0 / 64.0
      data.loc[len(data)] = [t, int(csv[2]), int(csv[3]), int(csv[4])]

print(data)

fig = px.line(data, x="t_ms", y=["rps", "setpoint"])
fig.show()
