#!/usr/bin/python
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

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.backends.backend_pdf import PdfPages


def create_pdf(dataframes, pdf_filename, write_values=False, directory="None"):
    with PdfPages(pdf_filename) as pdf:
        for dataframe in dataframes:
            for index in range(len(dataframe.columns)):
                plot = create_histogram(dataframe, index)
                pdf.savefig(plot)
            if write_values and dataframe.columns.name == "values":
                pdf.savefig(create_table(dataframe))
        if directory != "None":
            pdf.savefig(create_directory_page(directory))


def create_directory_page(directory):
    directory_page = plt.figure()
    directory_page.text(0.1, 0.9, "Output directory: " + directory)
    return directory_page


def create_table(dataframe):
    sorted_dataframe = dataframe.sort_values("job_id")
    figure = plt.figure()
    ax = plt.subplot(111)
    ax.axis("off")
    ax.table(
        cellText=sorted_dataframe.values,
        colLabels=sorted_dataframe.columns,
        bbox=[0, 0, 1, 1],
    )
    return figure


def create_histogram(dataframe, index):
    figure = plt.figure()
    job_ids = dataframe["job_id"]
    y = dataframe.iloc[:, index]
    plt.scatter(job_ids, y, c=y, marker="+", s=150, linewidths=4, cmap=plt.cm.coolwarm)
    column_name = dataframe.columns[index]
    plt.title(column_name)
    return figure


def load_dataframe(files):
    dataframes = [pd.read_csv(file) for file in files]
    dataframe = pd.concat(dataframes)
    return dataframe
