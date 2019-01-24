#!/usr/bin/env python

import xml.dom.minidom as minidom
import socket
import string
import datetime
import ConfigParser
from os import path as osPath

filepath = osPath.dirname(osPath.realpath(__file__))

DDS_PROFILE_FILE=filepath + "/dds_types/DDSProfile.xml"
CONFIG_FILE=filepath + "/config.ini"

class Preferences:
    def __init__(self, partition_name = None, given_peer = None):
        self.config = ConfigParser.ConfigParser()
        self.dom = None
        self.partition_name = partition_name
        self.initial_peers = dict()
        self.given_peer = given_peer
        self.err = dict()
        self.warn = dict()
        self.info = dict()

    def validate_initial_peer(self, peer):
        dictionary = dict()

        # Check if the given peer is a valid IP
        if peer.count('.') == 3:
            try:
                socket.inet_aton(peer)
            except socket.error:
                pass
            else:
                dictionary[peer] = peer
                return dictionary

        self.add_warning("""The given peer is NOT a valid IP,
        valid profiles will be checked""")

        # Check if input is a valid profile
        for key, value in self.initial_peers.items():
            if string.lower(peer) == key:
                self.add_warning("The given peer IS a valid profile. IP: " +
                value + " will be used.")
                dictionary[key] = value
                return dictionary

        self.add_warning("""The given peer is NOT a valid profile, IPs from config
        file will be used""")
        return None

    def get_robot_name(self):
        # Catch error if name is not placed
        if self.partition_name == None:
            self.partition_name = self.config.get('RobotName', 'name')

        return self.partition_name

    def get_initial_peers(self):
        if len(self.initial_peers) == 0:
            # Catch error if ips are not placed
            peers_items = self.config.items('DiscoveryPeers')
            for key, ip in peers_items:
                self.initial_peers[key] = ip

        if self.given_peer != None:
            validated_peer = self.validate_initial_peer(self.given_peer)
            if validated_peer != None:
                self.initial_peers.clear()
                for key, ip in validated_peer.items():
                    self.initial_peers[key] = ip
            else:
                # Show a message explaining the value is invalid. We proceed to
                # use peers on config file
                pass

        self.given_peer = None
        return self.initial_peers

    def clear_node_list(self, node_list, sub_tag_name):
        peers = node_list.getElementsByTagName(sub_tag_name)
        for peer in peers:
            node_list.removeChild(peer)
            peer.unlink()

    def write_node_list(self, node_list, sub_tag_name, children):
        children = [children] if isinstance(children, basestring) else children
        for child in children:
            new_node = node_list.ownerDocument.createElement(sub_tag_name)
            new_node.appendChild(new_node.ownerDocument.createTextNode(children[child] if isinstance(children, dict) else child))
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
        self.override_node_children(self.dom.getElementsByTagName('initial_peers')[0], 'element', new_peers)

    def replace_partition_name(self):
        new_name = self.get_robot_name()
        subscriber_partitions = self.dom.getElementsByTagName('domain_participant_library')[0].getElementsByTagName('name')
        for sub_part in subscriber_partitions:
            self.override_node_children(sub_part, 'element', new_name)

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
        if self.config.sections():
            if self.config.has_section("RobotName") and self.config.has_section("DiscoveryPeers"):
                if self.config.has_option("RobotName", "name") and self.config.options("DiscoveryPeers"):
                    return True
        return False

    def destroy_dom(self):
        if self.dom != None:
            self.dom.unlink()

    def set_preferences(self, partition_name = None, given_peer = None):

        # Override preferences
        if partition_name != None:
            self.partition_name = partition_name
        if given_peer != None:
            self.given_peer = given_peer

        try:
            self.dom = minidom.parse(DDS_PROFILE_FILE)
        except Exception as e:
            self.add_error("DDS profile was NOT found or is corrupted. We cannot continue.")
            self.add_info("Configuration process failed. See warnings and errors")
            return False

        self.config.read(CONFIG_FILE)

        if not self.validate_config_file():
            self.add_error("""Config file was NOT found or is corrupted.
            We cannot continue""")
            self.add_info("Configuration process failed. See warnings and errors")
            return False

        # TODO(rgarciar): Catch errors here
        self.replace_initial_peers()
        self.replace_partition_name()
        self.write_xml_file(DDS_PROFILE_FILE)

        info_text = "Configuration process was SUCCESSFUL. Following values will be used:" \
         + "\n\nRobot Name: " + self.get_robot_name() + "\nInitial Peers:\n"

        for key, value in self.get_initial_peers().items():
            info_text += " - " + value + "\n"

        self.add_info(info_text)
        return True

# Usage
#config = Preferences()
#config.set_preferences()
#print config.get_all_warnings()
#print config.get_all_errors()
#print config.get_all_info()
