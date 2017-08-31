#!/usr/bin/env python
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import os
import sys

class FaultEntry:
    fault_id = 0
    warning = ""
    blocking = ""
    response = ""
    description = ""
    key = ""
    unload_nodelet = ""
    timeout_sec = -1
    misses = -1

def SplitRowIntoColumns(line):
    # Excel surrounds cells that contain a comma with quotation marks before
    # converting the workbook to csv, need to split the line such that we don't
    # split the contents of a cell in half
    
    cells = line.split(",")

    # If line doesn't contain quotations, we can just return the result of split
    if line.find("\"") == -1:
        return cells

    append = False;
    s = ","
    new_cells = []
    for cell in cells:
        # Last cell contained a quotation mark
        if append:
            current_cell = s.join((current_cell, cell))
            # Check if quotations were closed
            if cell.find("\"") != -1:
                current_cell = current_cell.replace("\"", "")
                new_cells.append(current_cell)
                current_cell = ""
                append = False
        else:
            # Check for quotation mark
            if cell.find("\"") != -1:
                current_cell = cell
                append = True
            else:
                new_cells.append(cell)

    return new_cells

# Return true, true if the value is yes, true, false if the value is no,
# and false, false otherwise
def ConvertToBool(value):
    # Convert string to lower case so we don't have to check no, No, or NO
    value = value.lower()
    # Value could be yes ? or no ? which isn't valid
    if value.find("?") != -1:
        return False, ""
    
    if value.find("no") != -1:
        return True, "false"
    
    if value.find("yes") != -1:
        return True, "true"

    return False, ""

# Return false if value isn't a number or if it is a negative number
# Return true if value is N/A since not all faults are heartbeats
def ConvertToNumber(value):
    if value.find("N/A") != -1:
        return True, -1
    try:
        num_value = float(value)
        if num_value > -1:
            return True, num_value
    except ValueError:
        return False, -1

    return False, -1

def main():
    if len(sys.argv) < 3:
        print "Incorrect nmuber of arguments! Please restart with path to " \
                + "the FMECA csv file and the path to the config files."
        return

    try:
        file = open(sys.argv[1], 'r')
    except IOError:
        print "Couldn't open file " + sys.argv[1]
        return

    lines = file.readlines()

    file.close()

    faults_config_file_name = sys.argv[2] + "/faults.config"
    fault_table_config_file_name = sys.argv[2] + "/fault_table.config"

    titles_read = False
    id_index = -1
    subsystem_index = -1
    description_index = -1
    warning_index = -1
    blocking_index = -1
    node_index = -1
    response_index = -1
    key_index = -1
    unload_index = -1
    timeout_index = -1
    misses_index = -1

    fault_table = {}
    fault_ids_not_added = []
    for line in lines:
        if titles_read:
            cells = SplitRowIntoColumns(line)
            fault_id = cells[id_index]
            subsystem = cells[subsystem_index]
            # Skip fault if we aren't sure what subsystem it belongs to
            if subsystem.find("?") != -1:
                fault_ids_not_added.append(fault_id)
                continue

            # Remove FSW- from subsystem because it is only needed in the FMECA 
            if subsystem.find("FSW-") != -1:
                subsystem = subsystem.replace("FSW-", "")
            subsystem = subsystem.lower()

            node_name = cells[node_index]
            # Skip fault if we aren't sure what node it belongs to
            if len(node_name) == 0 or node_name.find("?") != -1:
                fault_ids_not_added.append(fault_id)
                continue

            fault_entry = FaultEntry()
            fault_entry.fault_id = fault_id
            fault_entry.description = cells[description_index]

            succeed, fault_entry.warning = ConvertToBool(cells[warning_index])
            if succeed == False:
                fault_ids_not_added.append(fault_id)
                continue

            succeed, fault_entry.blocking = ConvertToBool(cells[blocking_index])
            if succeed == False:
                fault_ids_not_added.append(fault_id)
                continue

            fault_entry.response = cells[response_index]
            if fault_entry.response.find("?") != -1 or \
                    len(fault_entry.response) == 0:
                fault_ids_not_added.append(fault_id)
                continue

            fault_entry.key = cells[key_index]
            if fault_entry.key.find("?") != -1 or \
                    len(fault_entry.key) == 0:
                fault_ids_not_added.append(fault_id)
                continue

            succeed, fault_entry.unload_nodelet = \
                    ConvertToBool(cells[unload_index])
            if succeed == False:
                fault_ids_not_added.append(fault_id)
                continue

            succeed, fault_entry.timeout_sec = \
                    ConvertToNumber(cells[timeout_index])
            if succeed == False:
                fault_ids_not_added.append(fault_id)
                continue

            succeed, fault_entry.misses = \
                    ConvertToNumber(cells[misses_index])
            if succeed == False:
                fault_ids_not_added.append(fault_id)
                continue

            # Fault talbe is a dictionary of dictionary of lists. First
            # dictionary is keyed on the subsystem name, the second dictionary
            # is keyed on the node name and the list is a list of faults for the
            # node
            if fault_table.has_key(subsystem): 
                if fault_table[subsystem].has_key(node_name):
                    fault_table[subsystem][node_name].append(fault_entry)
                else: # Node isn't in the dictionary
                    fault_table[subsystem][node_name] = [fault_entry]
            else:  # Subsystem isn't in the dictionary
                fault_table[subsystem] = {}
                fault_table[subsystem][node_name] = [fault_entry]

        else:
            # Find line row containing column heading so we know what columns to
            # extract
            if line.find("Fault ID") != -1:
                titles_read = True
                # Line is comma seperated so split the line on the comma
                titles = SplitRowIntoColumns(line)
                index = 0
                for title in titles:
                    if title.find("Fault ID") != -1:
                        id_index = index
                    if title.find("Subsystem Name") != -1:
                        subsystem_index = index
                    if title.find("Failure Mechanism") != -1:
                        description_index = index
                    if title.find("Warning") != -1:
                        warning_index = index
                    if title.find("Blocking") != -1:
                        blocking_index = index
                    if title.find("Node Name") != -1:
                        node_index = index
                    if title.find("Response Command") != -1:
                        response_index = index
                    if title.find("Key") != -1:
                        key_index = index
                    if title.find("Unload") != -1:
                        unload_index = index
                    if title.find("Timeout") != -1:
                        timeout_index = index
                    if title.find("Misses") != -1:
                        misses_index = index
                    index = index + 1

                if id_index == -1:
                    print  "Could not find fault id column!"
                    break
                if subsystem_index == -1:
                    print "Could not find subsystem column!"
                    break
                if description_index == -1:
                    print "Could not find failure mechanism column!"
                    break
                if warning_index == -1:
                    print "Could not find warning column!"
                    break
                if blocking_index == -1:
                    print "Could not find blocking column!"
                    break
                if node_index == -1:
                    print "Could not find node name column!"
                    break
                if response_index == -1:
                    print "Could not find response command column!"
                    break
                if key_index == -1:
                    print "Could not find key column!"
                    break
                if unload_index == -1:
                    print "Could not find unload nodelet column!"
                    break
                if timeout_index == -1:
                    print "Could not find heartbeat timeout column!"
                    break
                if misses_index == -1:
                    print "Could not find heartbeat timeout column!"
                    break


    if titles_read == False: 
        print "Could not find row with column headers!"
    else:
        fc_file = open(faults_config_file_name, 'w')
        ftc_file = open(fault_table_config_file_name, 'w')
        line = "-- Autogenerated from FMECA! DO NOT CHANGE!\n\n"
        fc_file.write(line)
        ftc_file.write(line + "require \"sys_monitor\"\n\nsubsystems={\n")
        for subsys_key in fault_table.keys():
            ftc_file.write("  {name=\"" + subsys_key + "\", nodes={\n")
            for nodes_key in fault_table[subsys_key].keys():
                fc_file.write(nodes_key + " = {\n")
                ftc_file.write("    {name=\"" + nodes_key + "\", faults={\n")
                for fault in fault_table[subsys_key][nodes_key]:
                    ftc_file.write("      {id=" + fault.fault_id + \
                            ", warning=" + fault.warning + ", blocking=" + \
                            fault.blocking + ", response=command(\"" + \
                            fault.response + "\"), unload=" + \
                            fault.unload_nodelet + ", key=\"" + fault.key + \
                            "\", description=\"" + fault.description + "\"")
                    
                    # Don't add heartbeat faults to the fault table since nodes
                    # don't trigger their own heartbeat missing fault
                    if fault.key.find("HEARTBEAT") == -1:
                        fc_file.write("  {id=" + fault.fault_id + ", " + \
                                "key=\"" + fault.key + "\", description=\"" + \
                                fault.description + "\"},\n")
                        # Close fault entry in fault table
                        ftc_file.write("},\n")
                    else: 
                        # Add heartbeat portion of the fault to the fault entry
                        ftc_file.write(", heartbeat={timeout_sec=" + \
                                str(fault.timeout_sec) + ", misses=" + \
                                str(fault.misses) + "}},\n")
                fc_file.write("}\n\n")  # close node table
                ftc_file.write("    }},\n")
            ftc_file.write("  }},\n")
        ftc_file.write("}\n")

        fc_file.close()
        ftc_file.close()

        faults_not_added = "The follow faults weren't added: "
        for fault_id in fault_ids_not_added:
            faults_not_added = faults_not_added + fault_id + ", "
        last_comma = len(faults_not_added) - 2
        list_faults = list(faults_not_added)
        list_faults[last_comma] = '!'
        faults_not_added = ''.join(list_faults)
        print faults_not_added

if __name__ == '__main__':
    main()
