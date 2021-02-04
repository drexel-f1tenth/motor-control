import plotly.express as px
import pandas as pd
import sys

log_file = sys.argv[1]
data = pd.read_csv(
  log_file,
  sep=" ",
  usecols=[1, 2, 3],
  columns=["rps", "setpoint", "adjust"])

data["t_ms"] = map(
  range(0, len(data)),
  lambda i: i * (1000.0 / 64.0))

print(data)

fig = px.line(data, x="t_ms", y=["rps", "setpoint"])
fig.show()
