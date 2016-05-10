#!/usr/bin/env python

import math
import time
import rospy
import datetime
import argparse
from human_trajectory.trajectories import OfflineTrajectories
from region_observation.observation_proxy import RegionObservationProxy
from periodic_poisson_processes.poisson_processes import PeriodicPoissonProcesses
from region_observation.util import create_line_string, is_intersected, get_soma_info


class OfflinePeoplePoisson(object):

    def __init__(self, config, window=10, increment=1, periodic_cycle=10080):
        rospy.loginfo("Starting the offline poisson reconstruction...")
        self._start_time = None
        # get soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        self.obs_proxy = RegionObservationProxy(self.map, self.config)
        # for each roi create PoissonProcesses
        rospy.loginfo("Time window is %d minute with increment %d minute" % (window, increment))
        self.time_window = window
        self.time_increment = increment
        rospy.loginfo("Creating a periodic cycle every %d minutes" % periodic_cycle)
        self.process = {
            roi: PeriodicPoissonProcesses(
                window, increment, periodic_cycle
            ) for roi in self.regions.keys()
        }

    def store_to_db(self):
        rospy.loginfo("Storing to database...")
        for roi in self.process.keys():
            meta = {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "people"
            }
            self.process[roi].store_to_mongo(meta)

    def load_from_db(self):
        rospy.loginfo("Retrieving from database...")
        is_retrieved = list()
        for roi in self.process.keys():
            meta = {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "people"
            }
            is_retrieved.append(self.process[roi].retrieve_from_mongo(meta))
        for roi, process in self.process.iteritems():
            if self._start_time is None or self._start_time > process._init_time:
                self._start_time = process._init_time
        if True in is_retrieved:
            rospy.loginfo("Starting time is %s" % self._start_time.secs)
        return (True in is_retrieved)

    def construct_process_from_trajectory(self):
        trajectories = OfflineTrajectories(size=10000)
        # set the starting time when the first trajectory was captured
        temp = datetime.datetime.fromtimestamp(trajectories.start_secs)
        temp = datetime.datetime(
            temp.year, temp.month, temp.day, temp.hour, temp.minute, 0
        )
        self._start_time = rospy.Time(time.mktime(temp.timetuple()))
        # get trajectory messages
        trajectories = [
            i.get_trajectory_message() for i in trajectories.traj.values()
        ]
        start_time = self._start_time
        end_time = start_time + rospy.Duration(self.time_window*60)
        while len(trajectories) > 0 and start_time < rospy.Time.now():
            rospy.loginfo("Trajectories left are %d" % len(trajectories))
            rospy.loginfo(
                "Obtaining region and trajectories between %s and %s" % (
                    start_time.secs, end_time.secs
                )
            )
            region_observations = self.obs_proxy.load_msg(
                start_time, end_time, minute_increment=self.time_increment
            )
            print len(region_observations)
            count_per_region, trajectories = self.get_count(
                region_observations, trajectories
            )
            for roi, count in count_per_region.iteritems():
                self.process[roi].update(start_time, count)
                self._store(roi, start_time)
            start_time = start_time + rospy.Duration(self.time_increment*60)
            end_time = start_time + rospy.Duration(self.time_window*60)

    def get_count(self, region_observations, trajectories):
        used_trajectories = list()
        count_per_region = dict()
        for observation in region_observations:
            count = 0
            for trajectory in trajectories:
                    points = [
                        [
                            pose.pose.position.x, pose.pose.position.y
                        ] for pose in trajectory.trajectory
                    ]
                    points = create_line_string(points)
                    # trajectory must intersect with any region
                    if is_intersected(self.regions[observation.region_id], points):
                        # it also must be within time boundaries
                        conditions = trajectory.end_time >= observation.start_from
                        # observation until is secs.999999999999
                        conditions = conditions and trajectory.end_time <= observation.until
                        if conditions:
                            count += 1
                            used_trajectories.append(trajectory)
            if count > 0 or observation.duration.secs >= 59:
                count = self._extrapolate_count(observation.duration, count)
            if observation.region_id not in count_per_region.keys():
                count_per_region[observation.region_id] = 0
            count_per_region[observation.region_id] += count
        # remove trajectories that have been updated
        trajectories = [i for i in trajectories if i not in used_trajectories]
        rospy.loginfo(str(count_per_region))
        return count_per_region, trajectories

    def _extrapolate_count(self, duration, count):
        """ extrapolate the number of trajectories with specific upper_threshold.
            upper_threshold is to ceil how long the robot was in an area
            for one minute interval, if the robot was there for less than 20
            seconds, then it will be boosted to 20 seconds.
        """
        upper_threshold_duration = rospy.Duration(0, 0)
        while duration > upper_threshold_duration:
            upper_threshold_duration += rospy.Duration(
                self.time_increment * 20, 0
            )

        multiplier_estimator = 3600 / float(
            (60 / self.time_increment) * upper_threshold_duration.secs
        )
        # rospy.loginfo("Extrapolate count %d by %.2f" % (count, multiplier_estimator))
        return math.ceil(multiplier_estimator * count)


if __name__ == '__main__':
    rospy.init_node("offline_people_poisson")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument("soma_config", help="Soma configuration")
    parser.add_argument(
        "-w", dest="time_window", default="10",
        help="Fixed time window interval (in minute) for each Poisson distribution. Default is 10 minutes"
    )
    parser.add_argument(
        "-i", dest="time_increment", default="1",
        help="Incremental time (in minute). Default is 1 minute"
    )
    parser.add_argument(
        "-p", dest="periodic_cycle", default="10080",
        help="Desired periodic cycle (in minute). Default is one week (10080 minutes)"
    )
    args = parser.parse_args()

    offline = OfflinePeoplePoisson(
        args.soma_config, int(args.time_window),
        int(args.time_increment), int(args.periodic_cycle)
    )
    if not offline.load_from_db():
        offline.construct_process_from_trajectory()
    offline.store_to_db()
