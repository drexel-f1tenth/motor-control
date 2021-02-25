import pandas as pd
import plotly.graph_objects as go


def slurp(filename):
  data = pd.DataFrame(columns=['time_ms', 'RPS1', 'RPS2'])
  row = []
  with open(filename) as f:
    for i, line in enumerate(f):
      if not line.startswith('data: "rps:'): continue
      csv = [s.removeprefix("+") for s in line.replace('"', '').split()]
      if len(csv) != 5: continue
      data.loc[len(data)] = [i * 125, int(csv[2]), int(csv[3])]

  return data

fig = go.Figure()

rps_inputs = [6, 8, 10, 11, 12, 13]
for rps in rps_inputs:
  label = f'{rps:02d}'
  data = slurp(f'log-{label}.txt')
  fig.add_trace(go.Scatter(x=data['time_ms'], y=data['RPS1'], name=label))

fig.show()
