#!/usr/bin/env python

import rospy
import datetime
import argparse
import numpy as np
import matplotlib.pyplot as plt
from periodic_poisson_processes.offline_people_poisson import OfflinePeoplePoisson


class PoissonPlot(object):

    def __init__(
        self, config, time_window=10, time_increment=1, periodic_cycle=10080
    ):
        self.process = OfflinePeoplePoisson(
            config, time_window, time_increment, periodic_cycle
        )
        self.regions = self.process.process.keys()
        self.process.load_from_db()

    def plot_poisson_per_region(self, region):
        start_time, y, low_err, up_err = self.process.get_rate_rate_err_per_region(region)
        _, reconstruction, _, _ = self.process.fourier_reconstruction(region)
        x = np.arange(len(y))
        plt.errorbar(
            x, y, yerr=[low_err, up_err], color='b', ecolor='r',
            fmt="-o", label="Poisson Process"
        )
        plt.plot(
            x, reconstruction, "-x", color="green", label="Fourier Reconstruction"
        )
        # plt.plot(
        #     x, residue, "-x", color="yellow", label="Residue"
        # )
        plt.title("Poisson Process for Region %s" % region)
        start_time = datetime.datetime.fromtimestamp(start_time.secs)
        plt.xlabel(
            """%d minute-period with %d minute increment and %d minute time window \n
            Starting at %s""" % (
                self.process.periodic_cycle, self.process.time_increment,
                self.process.time_window,
                str(start_time)
            )
        )
        plt.ylabel("Poisson rate value")
        plt.ylim(ymin=-1)
        plt.legend()
        plt.show()


if __name__ == '__main__':
    rospy.init_node("poisson_plot")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument('soma_config', help="Soma configuration")
    parser.add_argument(
        "-w", dest="time_window", default="10",
        help="Fixed time window interval (in minute) for each Poisson distribution. Default is 10 minute."
    )
    parser.add_argument(
        "-m", dest="time_increment", default="1",
        help="Incremental time (in minute). Default is 1 minute."
    )
    parser.add_argument(
        "-p", dest="periodic_cycle", default="10080",
        help="Desired periodic cycle (in minute). Default is one week (10080 minutes)"
    )
    args = parser.parse_args()
    plot = PoissonPlot(
        args.soma_config, int(args.time_window),
        int(args.time_increment), int(args.periodic_cycle)
    )
    region = raw_input("Region %s: " % str(plot.regions))
    plot.plot_poisson_per_region(region)
