""" Constants in meters based on JEM keepin dimensions as defined in
    https://babelfish.arc.nasa.gov/bitbucket/projects/ASTROBEE/repos/astrobee_ops/browse/gds/ControlStationConfig/IssWorld-Default/keepins/keepin.json
"""

# Lower FWD corner x coordinate at the entry node, keepin zone from bay 1 to 6
XMIN_BAY16 = 11.99
YMIN_BAY16 = -2.75
ZMIN_BAY16 = 5.92
# Higher AFT corner x coordinate at bay 6/7, keepin zone from bay 1 to 6
XMAX_BAY16 = 9.86
YMAX_BAY16 = -9.24
ZMAX_BAY16 = 3.78
# Lower FWD corner x coordinate at bay 6/7. keepin zone from bay 6/7 to 8
XMIN_BAY67 = 12.34
YMIN_BAY67 = -9.24
ZMIN_BAY67 = 5.95
# Higher AFT corner x coordinate at bay 6/7. keepin zone from bay 6/7 to 8
XMAX_BAY67 = 9.54
YMAX_BAY67 = -11.64
ZMAX_BAY67 = 3.75
# Lower right side corner, airlock
XMIN_AIRLK = 11.75
YMIN_AIRLK = -10.60
ZMIN_AIRLK = 6.00
# Higher left side corner, airlock
XMAX_AIRLK = 10.25
YMAX_AIRLK = -10.70
ZMAX_AIRLK = 4.55

# Number of features and robot coordinates saved in database CSV file
NUM_FEAT_AND_ROBOT_COORDS_COLUMNS = 8
# Number of columns representing ML Features X-Y-Z coordinates saved in database CSV file
FEAT_COORDS_COLUMNS = 3

# Start and End points along each bay's y-axis
BAYS_Y_LIMITS = {
    0: [-2.750, -4.300],
    1: [-4.300, -5.220],
    2: [-5.220, -6.500],
    3: [-6.430, -7.550],
    4: [-7.430, -8.660],
    5: [-8.550, -9.500],
    6: [-9.480, -10.800],
    7: [-10.712, -12.050],
}
