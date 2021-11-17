##############################################################################
# Copyright (c) 2005-2015 Real-Time Innovations, Inc. All rights reserved.
# Permission to modify and use for internal purposes granted.
# This software is provided "as is", without warranty, express or implied.
##############################################################################
"""Samples's reader."""


from os import path as osPath
from sys import path as sysPath
from time import sleep

filepath = osPath.dirname(osPath.realpath(__file__))
sysPath.append(filepath + "../")
import rticonnextdds_connector as rti

connector = rti.Connector("MyParticipantLibrary::Zero", filepath + "/PositionTest.xml")
inputDDS = connector.getInput("MySubscriber::MyTextReader")

for i in range(1, 500):
    print("i loop")
    inputDDS.take()
    numOfSamples = inputDDS.samples.getLength()
    for j in range(1, numOfSamples + 1):
        print("j loop")
        if inputDDS.infos.isValid(j):
            # Or you can just access the field directly
            pose = inputDDS.samples.getDictionary(j)
            toPrint = "pose: %s" % repr(pose)

            print(toPrint)
    sleep(2)
