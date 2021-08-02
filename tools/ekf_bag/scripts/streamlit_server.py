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


import glob
import os
import subprocess

import numpy as np
import pandas as pd
import streamlit as st
from pdf2image import convert_from_path


def PDFtoJPEG(filename, folder, start=0):
    pages = convert_from_path(filename + ".pdf", 500, size=1000)
    pageList = []
    for i in range(len(pages)):
        pageList.append(folder + "/" + str(start + i) + ".jpg")
        pages[i].save(pageList[-1], "JPEG")
    os.system("rm " + filename + '".pdf"')
    return pageList


@st.cache
def ekf_diffJPEGs(files, flags, folder):
    oldFiles = glob.glob(folder + "/*")
    for f in oldFiles:
        os.remove(f)
    cmd = "rosrun ekf_bag ekf_diff "
    cmd += " ".join(files)
    cmd += flags
    os.system(cmd)
    pageList = []
    for i in range(len(files)):
        for j in range(i + 1, len(files)):
            filename = files[i][:-4] + "_" + files[j][:-4]
            pageList.extend(PDFtoJPEG(filename, folder))
    if "-m" in flags:
        filename = files[0][:-4] + "_heatmap"
        pageList.extend(PDFtoJPEG(filename, folder, 3))
    return pageList


@st.cache
def ekf_graphJPEGs(mapFile, bagFile, flags, folder):
    oldFiles = glob.glob(folder + "/*")
    for f in oldFiles:
        os.remove(f)
    cmd = "rosrun ekf_bag ekf_graph " + bagFile + " " + mapFile + flags
    os.system(cmd)
    return PDFtoJPEG(bagFile[:-4], folder)


@st.cache
def ekf_video(mapFile, bagFile, flags, folder):
    oldFiles = glob.glob(folder + "/*")
    for f in oldFiles:
        os.remove(f)
    cmd = "rosrun ekf_video ekf_video " + bagFile + " " + mapFile + flags
    os.system(cmd)
    os.system(
        "ffmpeg -i "
        + bagFile[:-4]
        + ".mkv -c copy "
        + folder
        + "/"
        + bagFile.split("/")[-1][:-4]
        + ".mp4"
    )
    os.system("rm " + bagFile[:-4] + ".mkv")
    return folder + "/" + bagFile.split("/")[-1][:-4] + ".mp4"


st.title("Astrobee Pose Estimation Evaluation")
script = st.selectbox(
    "Script to Run:", ("ekf_diff", "ekf_graph", "ekf_video", "change branch")
)

if script == "ekf_diff":
    flags = ""

    textFiles = glob.glob("cache/*/*.txt")

    numFiles = st.slider("Number of Input Files", min_value=2, value=2, step=1)

    ekfFiles = []
    for i in range(numFiles):
        ekfFiles.append(st.selectbox("File " + str(i), textFiles))

    if st.sidebar.checkbox("Enable Resampling"):
        flags += " -i"
    if st.sidebar.checkbox("Use Relative Poses"):
        flags += " -r"
    if st.sidebar.checkbox("Generate Heatmap"):
        flags += " -m"
    if st.sidebar.checkbox("Use Sparse Map Pose Ground Truth"):
        flags += " -sm"
    if st.sidebar.checkbox("Define Start Time"):
        flags += " -s " + str(st.sidebar.number_input("", key=1, format="%f"))
    if st.sidebar.checkbox("Define Stop Time"):
        flags += " -e " + str(st.sidebar.number_input("", key=2, format="%f"))
    if st.sidebar.checkbox("Define Period"):
        flags += " -p " + str(st.sidebar.number_input("", key=3, format="%f"))

    if st.button("Run"):
        fileNames = []
        for ekfFile in ekfFiles:
            fileNames.append(ekfFile.split("/")[-1])
            os.system("cp " + ekfFile + " " + fileNames[-1])
        folder = "cache/ekf_diff__" + "__".join(fileNames + flags.split())
        os.system("mkdir -p cache")
        os.system("mkdir -p " + folder)

        imageList = ekf_diffJPEGs(fileNames, flags, folder)

        for ekfFile in fileNames:
            os.system("rm " + ekfFile)

        for image in imageList:
            st.image(image)
elif script == "ekf_graph":
    flags = ""

    bagFiles = glob.glob("bags/*.bag")
    mapFiles = glob.glob("maps/*.map")

    mapFile = st.selectbox("Map File", mapFiles)
    bagFile = st.selectbox("Bag File", bagFiles)

    if st.sidebar.checkbox("Use EKF Messages From Bag"):
        flags += " -e"
    if st.sidebar.checkbox("Use Feature Messages From Bag"):
        flags += " -f"
    if st.sidebar.checkbox("Do Not Rerun EKF"):
        flags += " --cached"
    if st.sidebar.checkbox("Define Start Time"):
        flags += " --start " + str(st.sidebar.number_input("", key=1, format="%f"))
    if st.sidebar.checkbox("Define Stop Time"):
        flags += " --end " + str(st.sidebar.number_input("", key=2, format="%f"))
    if st.sidebar.checkbox("Specify Robot"):
        flags += " -r " + st.sidebar.text_input(
            "Enter Robot Name Without Extension/Path", key=1
        )
    gitHash = str(
        subprocess.check_output(["git", "rev-parse", "origin/master"])[:-1], "utf-8"
    )
    folder = "cache/ekf_graph__" + "__".join(
        [gitHash, mapFile[5:], bagFile[5:]] + flags.split()
    )
    if st.sidebar.checkbox("Specify Image Topic", value=True):
        flags += " -i " + st.sidebar.text_input(
            "", value="/mgt/img_sampler/nav_cam/image_record", key=2
        )

    if st.button("Run"):
        os.system("mkdir -p cache")
        os.system("mkdir -p " + folder)

        imageList = ekf_graphJPEGs(mapFile, bagFile, flags, folder)

        os.system(
            "mv bags/" + bagFile[5:-4] + ".txt " + folder + "/" + bagFile[5:-4] + ".txt"
        )

        for image in imageList:
            st.image(image)
elif script == "ekf_video":
    flags = ""

    bagFiles = glob.glob("bags/*.bag")
    mapFiles = glob.glob("maps/*.map")

    mapFile = st.selectbox("Map File", mapFiles)
    bagFile = st.selectbox("Bag File", bagFiles)

    if st.sidebar.checkbox("Use EKF Messages From Bag"):
        flags += " -e"
    if st.sidebar.checkbox("Use Feature Messages From Bag"):
        flags += " -f"
    if st.sidebar.checkbox("Use JEM Mode: Top and Side View"):
        flags += " -j"
    if st.sidebar.checkbox("Specify Robot"):
        flags += " -r " + st.sidebar.text_input(
            "Enter Robot Name Without Extension/Path"
        )
    folder = "cache/ekf_video__" + "__".join(
        [bagFile.split("/")[-1], mapFile.split("/")[-1]] + flags.split()
    )
    os.system("mkdir -p " + folder)
    if st.sidebar.checkbox("Specify Image Topic", value=True):
        flags += " -i " + st.sidebar.text_input(
            "", value="/mgt/img_sampler/nav_cam/image_record"
        )

    if st.button("Run"):
        videoFile = ekf_video(mapFile, bagFile, flags, folder)

        video_file = open(videoFile, "rb")
        video_bytes = video_file.read()
        st.video(video_bytes)
elif script == "change branch":
    allBranches = str(subprocess.check_output(["git", "branch", "-a"]), "utf-8").split(
        "\n"
    )
    branches = []
    for branch in allBranches:
        if branch[2:17] == "remotes/origin/":
            branches.append(branch[17:])
    branch = st.selectbox("Branch", branches)
    currBranch = str(subprocess.check_output(["git", "branch"]), "utf-8").split()[1]
    if branch != currBranch:
        os.system("git checkout " + branch)
    if st.button("Build"):
        os.system("make -C ../../../../../astrobee_build/native")
