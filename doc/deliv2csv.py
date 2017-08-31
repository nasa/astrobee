import ruamel.yaml as yaml
from sys import stdout

stream = file('deliverables.yaml', 'r')

deliverables = yaml.round_trip_load(stream)

ref_keys = deliverables[0].values()[0].keys()

stdout.write("devliverable")
for k in ref_keys:
    stdout.write("\t" + k)
stdout.write("\n")

for d in deliverables:
    stdout.write(d.keys()[0])
    for k in ref_keys:
        s = ""
        try:
            v = d.values()[0][k]
            s = str(d.values()[0][k])
        except:
            pass
        stdout.write("\t"+s)
    stdout.write("\n")
