import pandas as pd
import plotly.express as px


def slurp(filename):
  data = pd.DataFrame(columns=["time_ms", "RPS1", "RPS2"])
  row = []
  with open(filename) as f:
    for i, line in enumerate(f):
      csv = line.split()
      if len(csv) != 7: continue

      if line.startswith("0:"):
        row = [i * 125, float(csv[6])]
      elif line.startswith("1:"):
        row.append(float(csv[6]))
        print(row)
        data.loc[len(data)] = row

  return data


log1 = slurp("log-forward-neutral.txt")
log2 = slurp("log-forward-reverse.txt")

fig1 = px.line(log1, x="time_ms", y=["RPS1", "RPS2"], labels={"y": "RPS"})
fig1.show()

fig2 = px.line(log2, x="time_ms", y=["RPS1", "RPS2"], labels={"y": "RPS"})
fig2.show()
