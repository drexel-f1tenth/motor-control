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

inputs = [6, 8, 10, 11, 12, 13]
for i in inputs:
  label = f'+{(float(i) / 60.0):.2f}% throttle'
  data = slurp(f'log-{i:02d}.txt')
  fig.add_trace(go.Scatter(x=data['time_ms'], y=data['RPS1'], name=label))

fig.update_xaxes(title=dict(text='t_ms'))
fig.update_yaxes(title=dict(text='RPS'))
fig.show()
