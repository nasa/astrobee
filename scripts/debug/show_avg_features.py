#!/usr/bin/env python

import rospy
import signal
import time
import sys
import argparse
import rosgraph
import socket
import numpy as np
from ff_msgs.msg import EkfState, VisualLandmarks, Feature2dArray

ARRAY_SIZE = 75
DEFAULT_SPAN = 5

ML_TOPIC = '/loc/ml/features'
OF_TOPIC = '/loc/of/features'
EKF_TOPIC = '/gnc/ekf'

favg = None

class FeatureAverager(object):
    def __init__(self, span):
        self.span = span
        self.subscribers = {}
        self.data = {}
        self.timer = None

        self.ml_data = {
            'ml_time': lambda x: rospy.Time(x.header.stamp.secs, x.header.stamp.nsecs).to_sec(),
            'ml_count': lambda x: len(x.landmarks)
        }

        self.of_data = {
            'of_time': lambda x: rospy.Time(x.header.stamp.secs, x.header.stamp.nsecs).to_sec(),
            'of_count': lambda x: len(x.feature_array)
        }

        self.ekf_data = {
            'ekf_time': lambda x: rospy.Time(x.header.stamp.secs, x.header.stamp.nsecs).to_sec(),
            'ekf_ml_count': lambda x: x.ml_count,
            'ekf_of_count': lambda x: x.of_count
        }

        self.callbacks = [self.ml_data, self.of_data, self.ekf_data]

        for d in self.callbacks:
            for k in d.keys(): self.data[k] = np.full(ARRAY_SIZE * self.span, 1e-10)

    def __add_data(self, callbacks, data):
        for key in callbacks:
            self.data[key] = np.roll(self.data[key], 1)
            self.data[key][0] = callbacks[key](data)

    def __ml_callback(self, data):
        self.__add_data(self.ml_data, data)

    def __of_callback(self, data):
        self.__add_data(self.of_data, data)

    def __ekf_callback(self, data):
        self.__add_data(self.ekf_data, data)

    def __get_average(self, t, time_d, count_d):
        span = float(self.span)
        idx = count = 0
        for d in time_d:
            idx += 1
            if abs(t - d) > span:
                break
            count += count_d[idx]
        return count / span

    def __check_topics(self):
        ml_found = of_found = ekf_found = False

        topics = rospy.get_published_topics()
        for t in topics:
            if t[0] == ML_TOPIC: ml_found = True
            if t[0] == OF_TOPIC: of_found = True
            if t[0] == EKF_TOPIC: ekf_found = True

        if not ml_found:
            print ' > Topic %s was not found' % ML_TOPIC

        if not of_found:
            print ' > Topic %s was not found' % OF_TOPIC

        if not ekf_found:
            print ' > Topic %s was not found' % EKF_TOPIC

        return ml_found and of_found and ekf_found

    def __delete_last_line(self):
        #cursor up one line
        sys.stdout.write('\x1b[1A')

        #delete last line
        sys.stdout.write('\x1b[2K')

    def is_ros_running(self):
        try:
            rosgraph.Master('/rostopic').getPid()
        except socket.error:
            return False

        return True

    def start(self):
        if not self.is_ros_running():
            print " > Could not find ROS master"
            return False

        rospy.init_node("loc_feature_avg", anonymous=False, disable_signals=True)

        if not self.__check_topics():
            print " > Some topics were not found. Behavior may be erratic. Do you wish to continue?"
            raw_input(" > Press any key to continue or Ctrl + C the program")
            sys.stdout.flush()

        self.subscribers['ml'] = rospy.Subscriber(ML_TOPIC,
            VisualLandmarks, self.__ml_callback)
        self.subscribers['of'] = rospy.Subscriber(OF_TOPIC,
            Feature2dArray, self.__of_callback)
        self.subscribers['ekf'] = rospy.Subscriber(EKF_TOPIC,
            EkfState, self.__ekf_callback)

        print '\nTimespan: %d second%s' % (self.span, 's' if self.span > 1 else '')
        print "\nAverage features per second:\n"
        sys.stdout.flush()

        self.timer = rospy.Timer(rospy.Duration(1), self.average_features)

        return True

    def average_features(self, event):
        idx = ml_count = 0
        t = event.current_real.to_sec()

        ml_avg = self.__get_average(t, self.data['ml_time'], self.data['ml_count'])
        of_avg = self.__get_average(t, self.data['of_time'], self.data['of_count'])
        ekf_ml_avg = self.__get_average(t, self.data['ekf_time'], self.data['ekf_ml_count'])
        ekf_of_avg = self.__get_average(t, self.data['ekf_time'], self.data['ekf_of_count'])

        msg = '\rRAW ML: %s | RAW OF: %s | EKF ML: %s | EKF OF: %s' % (ml_avg,
            of_avg, ekf_ml_avg, ekf_of_avg)

        # Delete old line
        sys.stdout.write('\x1b[2K')
        # Write new line
        sys.stdout.write(msg)
        sys.stdout.flush()

def main():
    parser = argparse.ArgumentParser(description='Average Localization Feature Display')

    parser.add_argument('--span', dest='span', action='store',
                               help='Timespan (in seconds) to average features')

    args, unknown = parser.parse_known_args()

    if args.span == None:
        args.span = DEFAULT_SPAN
    elif not args.span.isdigit():
        print " > Timespan must be an integer value higher than zero"
        exit()
    else:
        args.span = int(args.span)
        if args.span < 1:
            print " > Timespan must be higher than 0"
            exit()

    global favg
    favg = FeatureAverager(args.span)

    if favg.start():
        rospy.spin()
    else:
        print " > Unable to complete node start sequence"

    time.sleep(0.5)


def handler(signum, frame):
    print '\n\nShutting down gracefully...'
    favg.timer.shutdown()
    rospy.signal_shutdown("User stopped the program")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    main()
