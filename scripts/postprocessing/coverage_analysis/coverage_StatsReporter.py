#!/usr/bin/env python

import sys, re
import math
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt
import numpy as np


"""
Create a file summarizing the coverage percentage across the different
walls of the JEM at the ISS
"""

class Coverage_StatsReporter:

 def __init__(self):
     print("Coverage_StatsReporter started")
     self.max_num_features = 0
     self.first_bin = 0
     self.second_bin = 0
     self.third_bin = 0
     self.fourth_bin = 0
     self.fifth_bin = 0

     #Wall, 0%,1-10%,11-20%,21-40%,40+%, total_wall
     self.coverage_per_wall = ["", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
     self.final_stats = []

 def reset_values(self):
     self.total_distribution = 0
     self.first_bin = 0
     self.second_bin = 0
     self.third_bin = 0
     self.fourth_bin = 0
     self.fifth_bin = 0
     self.coverage_per_wall = ["", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

 # Categorize the number of cubes in each category
 # 0, 25, 50, 75, 100 percent of map coverage defined as
 # number of registered ML features per cube.
 def analize_wall_coverage(self, num_features_per_cube):
     """ # Normalized visualization desired (x' = (x -xmin)/ (xmax-xmin))
     """

     # If ML number based visualization is desired
     if (num_features_per_cube == 0):
        self.first_bin += 1.0
     elif (num_features_per_cube >= 1 and num_features_per_cube <= 10):
          self.second_bin += 1.0
     elif (num_features_per_cube >= 11 and num_features_per_cube <= 20):
          self.third_bin += 1.0
     elif (num_features_per_cube >= 21 and num_features_per_cube <= 40):
          self.fourth_bin += 1.0
     elif (num_features_per_cube >= 40):
          self.fifth_bin += 1.0

     self.total_distribution = self.first_bin + self.second_bin + self.third_bin + self.fourth_bin + self.fifth_bin

     self.coverage_per_wall[1] = (self.first_bin)* 100.0 / self.total_distribution
     self.coverage_per_wall[2] = (self.second_bin)*100.0 / self.total_distribution
     self.coverage_per_wall[3] = (self.third_bin)* 100.0 / self.total_distribution
     self.coverage_per_wall[4] = (self.fourth_bin)*100.0 / self.total_distribution
     self.coverage_per_wall[5] = (self.fifth_bin)* 100.0 / self.total_distribution

     self.coverage_per_wall[6] = self.coverage_per_wall[6] + num_features_per_cube
     #self.max_num_features = self.max_num_features + self.coverage_per_wall[5]
     #print("Total Num reg ML: {:.2f}".format(self.max_num_features))

     #self.coverage_per_wall[7] = self.max_num_features #(self.coverage_per_wall[5])*100.0 / self.max_num_features

 # Generate a report file containing the wall, percentage in a five-bin
 # distribution and the total number of registered ML features in a given
 # wall of the ISS' JEM.
 def generate_coverage_stats(self, fileIn):

     print("Reading {:s}".format(fileIn))

     wall_counter = -1

     with open(fileIn) as f_in:
          for line in f_in:
              # Skip empty lines
              if re.match("^\s*\n", line):
                 #print("Skipped empty line\n")
                 continue

              # Check if comments has string "Wall"
              if re.match("^\s*\#", line):
                 #print("Found a comment\n")

                 vals = []
                 for v in re.split("\s+", line):
                     if (v == ""):
                        continue
                     vals.append(v)

                 if (vals[1] == "Wall"):
                    #print("Found a Wall\n")
                    if (wall_counter == -1):
                       self.coverage_per_wall[0] = vals[2]  # Save the name of the wall
                       wall_counter += 1

                    else:
                       # Save the coverage of this wall in final array
                       self.final_stats.insert(wall_counter, self.coverage_per_wall)

                       # Reset values for each wall.
                       self.reset_values()

                       self.coverage_per_wall[0] = vals[2] # Save the name of the wall
                       wall_counter += 1

              # Extract all the values separated by spaces
              else:
                   vals = []
                   for v in re.split("\s+", line):
                       if v == "":
                          continue
                       vals.append(v)

                   if (len(vals) == 4):
                      self.analize_wall_coverage(int(vals[0]))

     # EOF so write the last values
     #print("Writing report\n")
     self.final_stats.insert(wall_counter, self.coverage_per_wall)
     self.write_report(fileIn)


 def write_report(self, fileIn):
     max_num_features = 0.0

     last_column = len(self.final_stats[0]) - 1
     #print("last_col: {:d}".format(last_column))

     f_stats_index = fileIn.find(".csv")
     stats_filename = fileIn[:f_stats_index] + "_stats.csv"
     print("Creating {:s}".format(stats_filename))

     with open(stats_filename, 'w') as f_stats:
          f_stats.write("# Coverage Distribution along the different walls of the JEM\n")
          f_stats.write("# Columns represent percentage of registered ML Features per wall (0, 1-10, 11-20, 21-40, 40+)\n")
          #f_stats.write("# Map corresponding to: {:s}\n".format(stats_filename[:stats_filename.find("_coverage")]))
          f_stats.write("# Analyisis corresponding to: {:s}\n".format(fileIn))
          f_stats.write("JEM Wall, 0ML, 1-10ML, 11-20ML, 21-40ML, 40+ML, Total Reg. ML Features per wall, Percentage of Total Num ML Features Analyzed\n")

          for i in range(len(self.final_stats)):
              max_num_features = max_num_features + self.final_stats[i][last_column]
          #a = []
          #for i in range(len(self.final_stats)):
          #    a.insert(i, [self.final_stats[i][last_column] * 100.0 / max_num_features])
          #print(a)

          for i in range(len(self.final_stats)):
              #self.final_stats[i].extend(a[i])
              self.final_stats[i].extend([self.final_stats[i][last_column] * 100.0 / max_num_features])
          for i in range(len(self.final_stats)):
              #for j in range(len(self.final_stats[i])):
                   #print(self.final_stats[i][j])
              f_stats.write("{:s}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}\n".format(
              self.final_stats[i][0],    #JEM Wall
              self.final_stats[i][1],    #0ML     percentage
              self.final_stats[i][2],    #1-10ML  %
              self.final_stats[i][3],    #11-20ML %
              self.final_stats[i][4],    #21-40ML %
              self.final_stats[i][5],    #40+ML   %
              self.final_stats[i][6],    #Total Reg ML/wall
              self.final_stats[i][7] ))  #Total Num ML Features Analyzed %
          f_stats.write("Total Number of Registered Features, {:d}\n".format(int(max_num_features)))

 def print_pdf_report(self, fileIn):

     f_in = fileIn.find(".csv")
     report_filename = fileIn[:f_in] + "_stats_report.pdf"
     print("Creating {:s}".format(report_filename))

     # Organize data
     num_walls = len(self.final_stats) # All the walls analyzed

     x_data = np.arange(num_walls)
     first_bin   = np.zeros(num_walls)
     second_bin  = np.zeros(num_walls)
     third_bin   = np.zeros(num_walls)
     fourth_bin  = np.zeros(num_walls)
     fifth_bin   = np.zeros(num_walls)
     x_ticks     = np.empty(num_walls, dtype=object)
     total_prcnt = np.zeros(num_walls)

     for i in range(len(self.final_stats)):
         x_ticks[i]     = str(self.final_stats[i][0])
         first_bin[i]   = self.final_stats[i][1]
         second_bin[i]  = self.final_stats[i][2]
         third_bin[i]   = self.final_stats[i][3]
         fourth_bin[i]  = self.final_stats[i][4]
         fifth_bin[i]   = self.final_stats[i][5]
         total_prcnt[i] = self.final_stats[i][7]

     # Create the PdfPages object to which we will save the pages:
     # The with statement makes sure that the PdfPages object is closed properly at
     # the end of the block, even if an Exception occurs.
     with PdfPages(report_filename) as pdf:

          # Plot General Coverage
          plt.title('General Coverage Distribution per Wall\n' + fileIn, fontsize=8)

          # plot data in grouped manner of bar type
          plt.bar(x_data, total_prcnt, width=0.8, label='Coverage Percentage', color='blue')

          plt.xticks(x_data, x_ticks, fontsize=4, rotation=45, ha='right')
          plt.yticks(fontsize=8)

          plt.xlabel(" ", fontsize=5)
          plt.ylabel("Coverage %", fontsize=8)
          plt.tight_layout() #adjusts the bottom margin of the plot so that the labels don't go off the bottom edge.
          #plt.legend(loc='best')

          pdf.savefig()  # saves the current figure into a pdf page
          plt.close()

          # Plot Detailed Coverage
          plt.title('Detailed Coverage Distribution per Wall\n' + fileIn, fontsize=8)

          # plot data in grouped manner of bar type
          plt.bar(x_data, first_bin,  width=0.8, label='0ML',     color='red')
          plt.bar(x_data, second_bin, width=0.8, label='1-10ML',  color='orange', bottom=first_bin)
          plt.bar(x_data, third_bin,  width=0.8, label='11-20ML', color='yellow', bottom=second_bin+first_bin)
          plt.bar(x_data, fourth_bin, width=0.8, label='21-40ML', color='green',  bottom=third_bin+second_bin+first_bin)
          plt.bar(x_data, fifth_bin,  width=0.8, label='40+ML',   color='blue',   bottom=fourth_bin+third_bin+second_bin+first_bin)

          plt.xticks(x_data, x_ticks, fontsize=4, rotation=45, ha='right')
          plt.yticks(fontsize=8)
          #plt.xlabel("Analyzed Locations", fontsize=5)
          plt.xlabel(" ", fontsize=5)
          plt.ylabel("Coverage %", fontsize=8)
          plt.tight_layout(rect=[0,0,0.95,1.0]) #adjusts the bottom margin of the plot so that the labels don't go off the bottom edge.
          #plt.tight_layout() #adjusts the bottom margin of the plot so that the labels don't go off the bottom edge.
          #plt.legend(loc='best')
          #plt.legend(fontsize=5, bbox_to_anchor=(0.0, 0.0, 1., .2), loc='lower left', ncol=5, mode="expand", borderaxespad=0., bbox_transform=plt.transFigure)
          plt.legend(fontsize=5, bbox_to_anchor=(0.5, -0.1), loc='upper center', ncol=5) #, mode="expand", borderaxespad=0.0)
          #plt.legend(fontsize=5, bbox_to_anchor=(1.02, 0.5), loc='center left', borderaxespad=0.0)
          #plt.show()

          pdf.savefig()  # saves the current figure into a pdf page
          plt.close()


# The main program
if __name__ == "__main__":

    try:
        if len(sys.argv) < 2:
           print("Usage: " + sys.argv[0] + ' <activity_coverage_database_file>')
           print("       <activity_coverage_database_file>: 'dir/to/activity-coverage-database-file.csv' ")
           sys.exit(1)

        fileIn  = sys.argv[1]

        obj = Coverage_StatsReporter()

        obj.generate_coverage_stats(fileIn)
        obj.print_pdf_report(fileIn)


    except KeyboardInterrupt:
        print("\n <-CTRL-C EXIT: USER manually exited!->")
        sys.exit(0)
