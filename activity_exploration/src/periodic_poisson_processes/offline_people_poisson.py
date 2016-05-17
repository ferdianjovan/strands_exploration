#!/usr/bin/env python

import time
import rospy
import datetime
import argparse
from human_trajectory.trajectories import OfflineTrajectories
from spectral_analysis.util import fourier_reconstruct, rectify_wave
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
        self.periodic_cycle = periodic_cycle
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

    def _store(self, roi, start_time):
        self.process[roi]._store(
            start_time,
            {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "people"
            }
        )

    def construct_process_from_trajectory(self, start_time, end_time):
        # set the starting time when the first trajectory was captured
        temp = datetime.datetime.fromtimestamp(start_time.secs)
        temp = datetime.datetime(
            temp.year, temp.month, temp.day, temp.hour, temp.minute, 0
        )
        start_time = rospy.Time(time.mktime(temp.timetuple()))
        mid_end = start_time + rospy.Duration(self.time_window*60)
        while mid_end <= end_time:
            rospy.loginfo(
                "Obtaining region and trajectories between %s and %s" % (
                    start_time.secs, mid_end.secs
                )
            )
            count_per_region = self.get_count(start_time, mid_end)
            for roi, count in count_per_region.iteritems():
                self.process[roi].update(start_time, count)
                self._store(roi, start_time)
            start_time = start_time + rospy.Duration(self.time_increment*60)
            mid_end = start_time + rospy.Duration(self.time_window*60)
        for roi, process in self.process.iteritems():
            if self._start_time is None or self._start_time > process._init_time:
                self._start_time = process._init_time

    def get_count(self, start_time, end_time):
        used_trajectories = dict()
        count_per_region = dict()
        region_observations = self.obs_proxy.load_msg(
            start_time, end_time, minute_increment=self.time_increment
        )
        # query all trajectories that starts or ends or
        # starts longer than the start_time + the time window
        query = {"$or": [
            {"end_time.secs": {
                "$gte": start_time.secs, "$lt": end_time.secs
            }},
            {"start_time.secs": {
                "$gte": start_time.secs, "$lt": end_time.secs

            }},
            {
                "start_time.secs": {"$lt": start_time.secs},
                "end_time.secs": {"$gte": end_time.secs}
            }
        ]}
        trajectories = OfflineTrajectories(query, size=10000)
        # get trajectory messages
        trajectories = [
            i.get_trajectory_message() for i in trajectories.traj.values()
        ]
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
                    if observation.region_id not in used_trajectories.keys():
                        used_trajectories[observation.region_id] = list()
                    if trajectory not in used_trajectories[observation.region_id]:
                        used_trajectories[observation.region_id].append(trajectory)
                        count += 1
            if count > 0 or observation.duration.secs >= 59:
                if observation.region_id not in count_per_region.keys():
                    count_per_region[observation.region_id] = list()
                count_per_region[observation.region_id].append(count)
        results = dict()
        tot_incre = ((end_time - start_time).secs) / (60 * self.time_increment)
        for roi, counts in count_per_region.iteritems():
            if sum(counts) > 0 or len(counts) == tot_incre:
                results[roi] = sum(counts)
        return results

    def get_rate_rate_err_per_region(self, region):
        start_time = self.process[region]._init_time
        end_time = start_time + rospy.Duration(
            self.process[region].periodic_cycle*60
        )
        end_time += self.process[region].time_window
        end_time -= self.process[region].minute_increment
        poisson = self.process[region].retrieve(start_time, end_time)
        upper = self.process[region].retrieve(
            start_time, end_time, use_upper_confidence=True
        )
        lower = self.process[region].retrieve(
            start_time, end_time, use_lower_confidence=True
        )
        keys = sorted(poisson.keys())
        rates = list()
        upper_bounds = list()
        lower_bounds = list()
        for key in keys:
            rates.append(poisson[key])
            upper_bounds.append(abs(poisson[key] - upper[key]))
            lower_bounds.append(abs(poisson[key] - lower[key]))
        return start_time, rates, lower_bounds, upper_bounds

    def fourier_reconstruction(self, region):
        start_time, original, _, _ = self.get_rate_rate_err_per_region(region)
        reconstruction, residue = fourier_reconstruct(original, num_of_freqs=30)
        reconstruction = rectify_wave(
            reconstruction,
            low_thres=self.process[region].default_lambda().get_rate()
        )
        return start_time, reconstruction, original, residue

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
    offline.load_from_db()

    start_time = raw_input("Start time 'year month day hour minute':")
    start_time = start_time.split(" ")
    start_time = datetime.datetime(
        int(start_time[0]), int(start_time[1]), int(start_time[2]),
        int(start_time[3]), int(start_time[4])
    )
    start_time = rospy.Time(time.mktime(start_time.timetuple()))
    end_time = raw_input("End time 'year month day hour minute':")
    end_time = end_time.split(" ")
    end_time = datetime.datetime(
        int(end_time[0]), int(end_time[1]), int(end_time[2]),
        int(end_time[3]), int(end_time[4])
    )
    end_time = rospy.Time(time.mktime(end_time.timetuple()))
    offline.construct_process_from_trajectory(start_time, end_time)
