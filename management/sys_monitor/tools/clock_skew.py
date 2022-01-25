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
import pandas as pd
import rosbag
from matplotlib import pyplot as plt
from numpy.polynomial.polynomial import Polynomial

REMOTE_PROCESSORS = (
    "LLP",
    "HLP",
)
REMOTE_PID = {name: i for i, name in enumerate(REMOTE_PROCESSORS)}
OUTLIER_PERCENTILE = 99.0


def dt64(t):
    return np.datetime64(t.to_nsec(), "ns")


def read_time_sync(in_bag_path):
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
            pid = REMOTE_PID[msg.remote_processor]
            d[i] = (pid, dt64(msg.mlp_time), dt64(msg.remote_time))
        return d


def seconds_from_time_delta(td):
    return td / np.timedelta64(1, "s")


def plot_residual(out_path, t, resid, ymax=None):
    fig = plt.figure()
    if ymax is not None:
        keep = (resid <= ymax).nonzero()
        t = t[keep]
        resid = resid[keep]
    plt.plot(t, resid * 1e3, ".")
    plt.xlabel("Time (s)")
    plt.ylabel("Residual (ms)")
    fig.set_size_inches((10, 10))
    plt.savefig(out_path)
    plt.close()
    print("Saved figure to:", out_path)


def model_processor(d, pname):
    print("== %s ==" % pname)

    print("Model form: MLP = m * %s + b" % pname)
    print("Reject outliers above percentile:", OUTLIER_PERCENTILE)

    pid = REMOTE_PID[pname]
    # filter entries for given processor
    d = d[(d["remote_processor"] == pid).nonzero()]

    t0 = d["remote_time"][0]
    remote_time = seconds_from_time_delta(d["remote_time"] - t0)
    mlp_time = seconds_from_time_delta(d["mlp_time"] - t0)

    # outlier rejection
    delta = mlp_time - remote_time
    thresh = np.percentile(delta, OUTLIER_PERCENTILE)
    keep = (delta <= thresh).nonzero()

    sync_tf = Polynomial.fit(remote_time[keep], mlp_time[keep], deg=1)
    b = sync_tf(0)
    m = sync_tf(1) - b
    mlp_model = sync_tf(remote_time)
    resid = mlp_time - mlp_model

    print("Time offset b (ms): %.3f" % (b * 1e3))
    print("Drift rate m - 1 (ppm): %.3f" % ((m - 1) * 1e6))
    print("Residual max (ms): %.3f" % (np.max(np.abs(resid)) * 1e3))
    print("Residual RMS (ms): %.3f" % (np.std(resid) * 1e3))
    print("Non-outlier residual max (ms): %.3f" % (np.max(np.abs(resid[keep])) * 1e3))
    print("Non-outlier residual RMS (ms): %.3f" % (np.std(resid[keep]) * 1e3))
    print()

    out_path = "clock_skew_%s.png" % pname
    plot_residual(out_path, mlp_time, resid)

    out_path_zoom = "clock_skew_%s_zoom.png" % pname
    plot_residual(
        out_path_zoom, mlp_time, resid, ymax=np.percentile(resid, OUTLIER_PERCENTILE)
    )

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
                % (pname, rec["remote_time"], "MLP", rec["mlp_time"], resid_val * 1e3)
            )
        print()


def clock_skew(in_bag_path):
    d = read_time_sync(in_bag_path)
    for pname in REMOTE_PROCESSORS:
        model_processor(d, pname)


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
