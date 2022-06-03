#!/usr/bin/env python

"""
Analyze clock skew LLP->MLP and HLP->MLP. Inputs an 'ars_default' type bag
containing the /mgt/sys_monitor/time_sync topic.
"""

from __future__ import print_function

import matplotlib

matplotlib.use("Agg")

import argparse

import numpy as np
import rosbag
import scipy.signal
from matplotlib import pyplot as plt
from numpy.polynomial.polynomial import Polynomial

REMOTE_PROCESSORS = (
    "LLP",
    "HLP",
)
REMOTE_ID = {name: i for i, name in enumerate(REMOTE_PROCESSORS)}
OUTLIER_PERCENTILE = 99.0


def dt64(t):
    """
    Convert rospy.Time to np.datetime64.
    """
    return np.datetime64(t.to_nsec(), "ns")


def read_time_sync(in_bag_path):
    """
    Read time sync data from @in_bag_path. Return a numpy array with named
    fields.
    """
    topic = "/mgt/sys_monitor/time_sync"
    with rosbag.Bag(in_bag_path, "r") as in_bag:
        msg_count = in_bag.get_type_and_topic_info().topics[topic].message_count
        d = np.empty(
            shape=(msg_count,),
            dtype=[
                ("remote_processor", np.ubyte),
                ("mlp_time", "datetime64[ns]"),
                ("remote_time", "datetime64[ns]"),
            ],
        )
        for i, (topic, msg, t) in enumerate(in_bag.read_messages([topic])):
            remote_id = REMOTE_ID[msg.remote_processor]
            d[i] = (remote_id, dt64(msg.mlp_time), dt64(msg.remote_time))
        return d


def seconds_from_time_delta(td):
    """
    Convert np.timedelta64 to floating point seconds.
    """
    return td / np.timedelta64(1, "s")


def plot_skew(out_path, title, t, delta, subsets, fit_tf):
    """
    Plot measured delta vs. time. @subsets lets you specify different
    parts of the data set to plot in different colors and label the
    legend properly. If @fit_tf is not None, plot the linear fit as
    well.
    """
    fig = plt.figure()

    for label, ind in subsets:
        plt.plot(t[ind], delta[ind] * 1e3, ".")
    labels = [label.capitalize() for label, ind in subsets]

    if fit_tf is not None:
        plt.plot(t, (fit_tf(t) - t) * 1e3, "-")
        labels.append("Linear fit")

    plt.grid()
    plt.legend(labels)
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Time delta (ms)")

    plt.tight_layout()
    fig.set_size_inches((15, 5))
    plt.savefig(out_path)
    plt.close()
    print("Saved figure to:", out_path)


def model_processor(d, remote_name):
    """
    Analyze the clock skew data for @remote_name.
    """
    print("== %s ==" % remote_name)

    remote_id = REMOTE_ID[remote_name]
    # filter entries for given processor
    d = d[(d["remote_processor"] == remote_id).nonzero()]

    t0 = d["remote_time"][0]
    remote_time = seconds_from_time_delta(d["remote_time"] - t0)
    mlp_time = seconds_from_time_delta(d["mlp_time"] - t0)
    delta = mlp_time - remote_time

    # hack to get index array for all indices of an array
    all_indices = (delta == delta).nonzero()

    # select a small subset of low-delta messages, using delta as a proxy for
    # comm latency. these will be used for the model fit.
    low_delta = scipy.signal.argrelextrema(delta, np.less, order=20)

    # reject outliers in "zoom" plots for better y axis scaling
    thresh = np.percentile(delta, OUTLIER_PERCENTILE)
    non_outlier = (delta <= thresh).nonzero()

    fit_tf = Polynomial.fit(remote_time[low_delta], mlp_time[low_delta], deg=1)
    b = fit_tf(0)
    m = fit_tf(1) - b
    mlp_model = fit_tf(remote_time)
    resid = mlp_time - mlp_model

    subsets = (
        ("all", all_indices),
        ("non-outlier", non_outlier),
        ("low-delta", low_delta),
    )

    print("Model form: MLP = m * %s + b" % remote_name)
    print("  Time offset b (ms): %.3f" % (b * 1e3))
    print("  Drift rate m - 1 (ppm): %.3f" % ((m - 1) * 1e6))
    print("  Model referenced to start time:", t0)

    for label, ind in subsets:
        print("Over %s samples:" % label)
        print("  Residual max (ms): %.3f" % (np.max(np.abs(resid[ind])) * 1e3))
        print("  Residual RMS (ms): %.3f" % (np.std(resid[ind]) * 1e3))

    if 1:
        top10_thresh = np.sort(resid)[-10]
        top10_keep = (resid >= top10_thresh).nonzero()
        top10 = d[top10_keep]
        resid_top10 = resid[top10_keep]
        print("Top 10 residuals:")
        for i, rec in enumerate(top10):
            resid_val = resid_top10[i]
            print(
                "  %s %s %s %s resid %.1f ms"
                % (
                    remote_name,
                    rec["remote_time"],
                    "MLP",
                    rec["mlp_time"],
                    resid_val * 1e3,
                )
            )

    for plot_type in ("raw", "residual"):
        if plot_type == "raw":
            y = delta
            plot_tf = fit_tf
            plot_title = "Clock skew MLP - %s [%s]" % (remote_name, plot_type)
        else:
            y = resid
            plot_tf = None
            plot_title = "Clock skew MLP - corrected(%s) [%s]" % (
                remote_name,
                plot_type,
            )

        out_path = "clock_skew_%s_%s.png" % (remote_name, plot_type)
        plot_skew(out_path, plot_title, remote_time, y, subsets, plot_tf)

        out_path_zoom = "clock_skew_%s_%s_zoom.png" % (remote_name, plot_type)
        zoom_subsets = subsets[1:]
        plot_skew(
            out_path_zoom, plot_title + " (zoom)", remote_time, y, zoom_subsets, plot_tf
        )
    print()


def clock_skew(in_bag_path):
    print("Non-outliers are points below percentile:", OUTLIER_PERCENTILE)
    print(
        "Low-delta samples are neigborhood minima with respect to mlp_time - remote_time."
    )
    print(
        "To minimize impact of variable comm latency, model is fit to low-delta samples."
    )
    print()

    d = read_time_sync(in_bag_path)
    for remote_name in REMOTE_PROCESSORS:
        model_processor(d, remote_name)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "in_bag",
        help="Path to input bag for analysis. Normally ends in 'ars_default.bag'.",
    )
    args = parser.parse_args()
    clock_skew(args.in_bag)


if __name__ == "__main__":
    main()
