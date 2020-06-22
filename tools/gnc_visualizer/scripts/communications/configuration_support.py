#!/usr/bin/env python

import xml.dom.minidom as minidom
import socket
import string
import datetime
import ConfigParser
from os import path as osPath
from os import remove
from collections import OrderedDict

filepath = osPath.dirname(osPath.realpath(__file__))

BASE_DDS_PROFILE_FILE=filepath + "/dds_types/BaseDDSProfile.xml"
DDS_PROFILE_FILE=filepath + "/dds_types/CurrentDDSProfile.xml"
CONFIG_FILE=filepath + "/config.ini"

class Preferences:
    def __init__(self, partition_name = None, given_peer = None, domain = None, public_ip = None):
        self.config = ConfigParser.ConfigParser()
        self.dom = None
        self.partition_name = partition_name
        self.initial_peers = dict()
        self.given_peer = given_peer
        self.domain = domain
        self.public_ip = public_ip
        self.err = OrderedDict()
        self.warn = OrderedDict()
        self.info = OrderedDict()

    def is_valid_ipv4(self, ip):
        if ip != None and ip.count('.') == 3:
            try:
                socket.inet_aton(ip)
            except socket.error:
                pass
            else:
                return True

        return False

    def validate_initial_peer(self, peer):
        dictionary = dict()

        # Check if the given peer is a valid IP
        if self.is_valid_ipv4(peer):
            dictionary[peer] = peer
            return dictionary

        self.add_warning("The given peer is NOT a valid IP, valid profiles " +
            "will be checked")

        # Check if input is a valid profile
        for key, value in self.initial_peers.items():
            if string.lower(peer) == key:
                self.add_warning("The given peer IS a valid profile. IP: " +
                value + " will be used.")
                dictionary[key] = value
                return dictionary

        self.add_warning("The given peer is NOT a valid profile, IPs from " +
            "config file will be used")
        return None

    def validate_public_ip(self, ip):
        if self.is_valid_ipv4(ip):
            return ip
        else:
            return -1

    def get_robot_name(self):
        if self.partition_name == None: # Robot name not given at start
            self.partition_name = self.config.get('RobotName', 'name')

        if self.partition_name.strip() == "":
            self.partition_name = None

        return self.partition_name

    def get_initial_peers(self):
        if len(self.initial_peers) == 0:
            peers_items = self.config.items('DiscoveryPeers')
            for key, ip in peers_items:
                if key.strip() != "" and ip.strip() != "" and self.is_valid_ipv4(ip):
                    self.initial_peers[key] = ip
                else:
                    self.add_warning("There are unvalid profile/IPs in the config file")

        if self.given_peer != None: # Robot IP was given at start
            validated_peer = self.validate_initial_peer(self.given_peer)
            if validated_peer != None:
                self.initial_peers.clear()
                for key, ip in validated_peer.items():
                    self.initial_peers[key] = ip
            else:
                # Valdate functions shows a message explaining the value is
                # invalid. We proceed to use peers on config file
                pass

        self.given_peer = None
        return self.initial_peers

    def get_domain_id(self):
        fallback = False
        if self.domain == None: # Domain id not given at start
            # Read from config file
            self.domain = self.config.get('DdsDomain', 'domain')
            self.add_warning("Domain ID will be read from config file")
            fallback = True

        if not self.domain.strip().isdigit():
            self.domain = None
            if not fallback:
                self.add_warning("Argument Domain ID is not valid, we will " +
                "fallback to the value in the configuration file")
                self.get_domain_id()

        return self.domain

    def get_public_ip(self):
        returnValue = None

        # Get ip from config file and put in a temp variable
        tmp_ip = self.config.get('TRek', 'public_ip')

        if self.public_ip == None:  # IP not given at startup
            if tmp_ip == None or tmp_ip == "":
                pass # No public IP anywhere, ignore quietly
            else:
                # Validate config file value
                returnValue = self.validate_public_ip(tmp_ip)
        else:
            returnValue = self.validate_public_ip(self.public_ip)
            if returnValue == -1:
                returnValue = self.validate_public_ip(tmp_ip)

        self.public_ip = returnValue
        return self.public_ip

    def clear_node_list(self, node_list, sub_tag_name):
        peers = node_list.getElementsByTagName(sub_tag_name)
        for peer in peers:
            node_list.removeChild(peer)
            peer.unlink()

    def write_node_list(self, node_list, sub_tag_name, children):
        children = [children] if isinstance(children, basestring) else children
        for child in children:
            new_node = node_list.ownerDocument.createElement(sub_tag_name)
            new_node.appendChild(new_node.ownerDocument.createTextNode(
                children[child] if isinstance(children, dict) else child))
            node_list.appendChild(new_node)

    def override_node_children(self, node_list, sub_tag_name, children):
        self.clear_node_list(node_list, sub_tag_name)
        self.write_node_list(node_list, sub_tag_name, children)

    def write_xml_file(self, file_out):
        self.dom.writexml( open(file_out, 'w'),
                   indent="",
                   addindent="",
                   newl='')

    def replace_initial_peers(self):
        new_peers = self.get_initial_peers()
        if new_peers == None or len(new_peers) == 0:
            self.add_error("No valid robot IPs were provided from arguments " +
                "neither config file. We cannot continue. Please review your " +
                "arguments and/or configuration file")
            return False
        else:
            self.override_node_children(self.dom.getElementsByTagName(
                'initial_peers')[0],'element', new_peers)
            return True

    def replace_partition_name(self):
        new_name = self.get_robot_name()
        if new_name == None:
            self.add_error("Robot name is empty. We cannot continue. " +
                "Please review your arguments and/or configuration file")
            return False
        else:
            subscriber_partitions = self.dom.getElementsByTagName(
                'domain_participant_library')[0].getElementsByTagName('name')
            for sub_part in subscriber_partitions:
                self.override_node_children(sub_part, 'element', new_name)
            return True

    def replace_domain_id(self):
        new_domain = self.get_domain_id()
        if new_domain == None:
            self.add_error("Domain is not valid. No valid domain ids were " +
            "found in the arguments or config file")
            return False
        else:
            domain_node = self.dom.getElementsByTagName('domain')[0]
            domain_node.attributes["domain_id"].value = new_domain
            return True

    def insert_public_ip(self):
        new_public_ip = self.get_public_ip()
        if new_public_ip == None:
            return True
        elif new_public_ip == -1:
            self.add_error("Public IP is not a valid IPv4. We cannot continue." +
                " Please review your arguments and/or configuration file")
            return False
        else:
            parents = self.dom.getElementsByTagName('qos_library')
            for child in parents:
                if child.getAttribute("name") == "RapidQosLibrary":
                    parent = child.getElementsByTagName('property')[0] \
                        .getElementsByTagName("value")[0]
                    node_element = parent.ownerDocument.createElement("element")
                    n_name = node_element.ownerDocument.createElement("name")
                    n_name.appendChild(n_name.ownerDocument.createTextNode( \
                        "dds.transport.UDPv4.builtin.public_address"))
                    n_value = node_element.ownerDocument.createElement("value")
                    n_value.appendChild(n_value.ownerDocument \
                        .createTextNode(new_public_ip))
                    node_element.appendChild(n_name)
                    node_element.appendChild(n_value)

                    sibling_node = None
                    elements = parent.getElementsByTagName("element")
                    for element in elements:
                        if element.getElementsByTagName('name')[0].firstChild.nodeValue \
                                == "dds.transport.UDPv4.builtin.parent.message_size_max":
                            sibling_node = element
                            break

                    parent.insertBefore(node_element, sibling_node)
                    break

            return True

    def add_warning(self, text):
        self.warn[str(datetime.datetime.now())] = text

    def add_error(self, text):
        self.err[str(datetime.datetime.now())] = text

    def add_info(self, text):
        self.info[str(datetime.datetime.now())] = text

    def get_warnings(self):
        return self.warn

    def get_errors(self):
        return self.err

    def get_info(self):
        return self.info

    def get_all_warnings(self):
        warnings_text = "\nThe configuration proccess produced the following warnings:\n"
        for key, value in self.warn.items():
            warnings_text += "\n" + key + " : " + value
        warnings_text += "\n    ----"
        return warnings_text

    def get_all_errors(self):
        errors_text = "\nThe configuration proccess produced the following errors:\n"
        for key, value in self.err.items():
            errors_text += "\n" + key + " : " + value
        errors_text += "\n    ----"
        return errors_text

    def get_all_info(self):
        info_text = "\nResume of configuration process:\n"
        for key, value in self.info.items():
            info_text += "\n" + key + " : " + value
        info_text += "\n    ----"
        return info_text

    def validate_config_file(self):
        return (
            self.config.sections() and
            self.config.has_section("RobotName") and
            self.config.has_section("DiscoveryPeers") and
            self.config.has_section("DdsDomain") and
            self.config.has_section("TRek") and
            self.config.has_option("RobotName", "name") and
            self.config.options("DiscoveryPeers") and
            self.config.has_option("DdsDomain", "domain") and
            self.config.has_option("TRek", "public_ip")
        )

    def destroy_dom(self):
        if self.dom != None:
            self.dom.unlink()
        remove(DDS_PROFILE_FILE)

    def set_preferences(self, partition_name = None, given_peer = None,
            domain = None, public_ip = None):

        # Override preferences with argument values
        if partition_name != None:
            self.partition_name = partition_name
        if given_peer != None:
            self.given_peer = given_peer
        if domain != None:
            self.domain = domain
        if public_ip != None:
            self.public_ip = public_ip

        # Read DDS XML config file
        try:
            self.dom = minidom.parse(BASE_DDS_PROFILE_FILE)
        except Exception as e:
            self.add_error("DDS profile was NOT found or is corrupted." +
                " We cannot continue.")
            self.add_info("Configuration process failed. See warnings and errors")
            return False

        # Read and validate DDS INI config file
        self.config.read(CONFIG_FILE)

        if not self.validate_config_file():
            self.add_error("""Config file was NOT found or is corrupted.
            We cannot continue""")
            self.add_info("Configuration process failed. See warnings and errors")
            return False

        if not (self.replace_initial_peers() and
                    self.replace_partition_name() and
                    self.replace_domain_id() and
                    self.insert_public_ip()):
            self.add_error("DDS Profile could not be configured." +
                " We cannot continue")
            self.add_info("Configuration process failed. See warnings and errors")
            return False

        self.write_xml_file(DDS_PROFILE_FILE)

        info_text = "Configuration process was SUCCESSFUL. Following values" + \
            " will be used:" + "\n\nRobot Name: " + self.get_robot_name() + \
            "\nInitial Peers:\n"

        for key, value in self.get_initial_peers().items():
            info_text += " - " + value + "\n"

        info_text += "\nDDS Domain ID: " + self.domain

        if self.public_ip != None:
            info_text += "\nATTENTION: Public IP: " + self.public_ip + "\n"

        self.add_info(info_text)
        return True

# Usage
#config = Preferences()
#config.set_preferences()
#print config.get_all_warnings()
#print config.get_all_errors()
#print config.get_all_info()
