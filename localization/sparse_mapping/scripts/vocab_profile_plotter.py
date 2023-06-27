import json
import argparse
import matplotlib.pyplot as plt
from rbo import rbo

# Create an ArgumentParser object
parser = argparse.ArgumentParser(description='Description of your script')

# Add a flag
parser.add_argument('--flag_name', type=str, default="data.json", help='Description of the flag')

# Parse the command-line arguments
args = parser.parse_args()

x = []

filename = "/home/lmao/Documents/" + args.flag_name

with open (filename, "r") as f:
    data = json.load(f)

data_to_plot = []
for vocab in data.keys():
    y = []
    depth = int(vocab.split("^")[1])
    cid_to_name = data[vocab]["cid_to_filename"]
    for i in range(len(data[vocab]['data'])):
        # rbo_score = float(data[vocab]['data'][i]["RBO_score"])
        # if (rbo_score > 1):
        #     print("rbo_score > 1 ", rbo_score)
        # if (rbo_score < .15):
        #     print("rbo_score < .15 ", rbo_score)
        #     print([cid_to_name[idx] for idx in data[vocab]['data'][i]["query_results"]])
        #     print("_________________________________")
        #     print([cid_to_name[idx] for idx in data[vocab]['data'][i]["brute_force_results"]])
        rbo_score = rbo(data[vocab]['data'][i]["query_results"], data[vocab]['data'][i]["brute_force_results"], p=0.95).ext
        y.append(rbo_score)
    x.append(depth)
    data_to_plot.append(y)

# Plot the points
fig, ax = plt.subplots()
ax.boxplot(data_to_plot)
ax.set_xticklabels(x)

# Customize the plot
plt.xlabel('depth')
plt.ylabel('rank biased overlap')
plt.title('Vocabulary Size vs Rank Biased Overlap')

plt.savefig("/home/lmao/Documents/rbo_vs_depth.png")
    
plt.show()