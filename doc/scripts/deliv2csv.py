from sys import stdout

import ruamel.yaml as yaml

stream = file("deliverables.yaml", "r")

deliverables = yaml.round_trip_load(stream)

ref_keys = list(list(deliverables[0].values())[0].keys())

stdout.write("devliverable")
for k in ref_keys:
    stdout.write("\t" + k)
stdout.write("\n")

for d in deliverables:
    stdout.write(list(d.keys())[0])
    for k in ref_keys:
        s = ""
        try:
            v = list(d.values())[0][k]
            s = str(list(d.values())[0][k])
        except:
            pass
        stdout.write("\t" + s)
    stdout.write("\n")
