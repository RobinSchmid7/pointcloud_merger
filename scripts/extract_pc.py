#!/usr/bin/env python3

"""
Saves point cloud messages from multiple topics at specific timestamps and concatenates them into a single point cloud.

Author: Robin Schmid
Date: Feb 2024
"""
import os
import torch
import tf2_ros
import rospy
import message_filters
import numpy as np
import bisect
from liegroups.torch import SO3, SE3
from typing import Optional
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

INPUT_PC1 = "/pointcloud_out_0"
INPUT_PC2 = "/pointcloud_out_1"
TIME_PATH = "/path/to/timestamps.txt"

OUTPUT_FRAME = "base"
OUTPUT_PATH = "/output_path"

class MessageSaver:
    def __init__(self):
        rospy.init_node('message_saver_node', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        with open(TIME_PATH, "r") as f:
            self.timestamps = sorted(float(line.strip()) for line in f)

        self.pc1_sub = message_filters.Subscriber(INPUT_PC1, PointCloud2)
        self.pc2_sub = message_filters.Subscriber(INPUT_PC2, PointCloud2)

        self.comb_sub = message_filters.ApproximateTimeSynchronizer([self.pc1_sub, self.pc2_sub], queue_size=10, slop=0.5, allow_headerless=True)
        self.comb_sub.registerCallback(self.msg_callback)

    def query_tf(self, parent_frame: str, child_frame: str, stamp: Optional[rospy.Time] = None):
        if stamp is None:
            stamp = rospy.Time(0)

        try:
            res = self.tf_buffer.lookup_transform(parent_frame, child_frame, stamp, timeout=rospy.Duration(0.03))
            trans = (res.transform.translation.x, res.transform.translation.y, res.transform.translation.z)
            rot = np.array([res.transform.rotation.x, res.transform.rotation.y, res.transform.rotation.z, res.transform.rotation.w])
            rot /= np.linalg.norm(rot)
            return (trans, tuple(rot))
        except Exception as e:
            rospy.logerr(f"Error in query tf: {e}")
            return (None, None)

    def project_pc(self, pc, tf):
        position = np.array(tf[:3, -1])
        R = np.array(tf[:3, :3])

        points_list = [np.matmul(R, np.array(p)) + position for p in pc]
        return torch.tensor(points_list, dtype=torch.float32)

    def rospcmsg_to_pctorch(self, ros_cloud):
        try:
            points_list = [torch.tensor(data[:3], dtype=torch.float32) for data in point_cloud2.read_points(ros_cloud, skip_nans=True)]
            return True, torch.stack(points_list)
        except Exception as e:
            rospy.logerr(f"Error in rospcmsg_to_pctorch: {e}")
            return False, None

    def ros_tf_to_torch(self, tf_pose, device="cpu"):
        if tf_pose[0] is None:
            return False, None
        t = torch.FloatTensor(tf_pose[0])
        q = torch.FloatTensor(tf_pose[1])
        return True, SE3(SO3.from_quaternion(q, ordering="xyzw"), t).as_matrix().to(device)

    def save_pc(self, timestamp, pc, output_path):
        timestamp_str = str(timestamp).replace(".", "_").ljust(18, '0')
        save_path = os.path.join(output_path, f"{timestamp_str}.pt")
        torch.save(pc, save_path)

    def msg_callback(self, *pcs):
        msg_time = rospy.Time.to_sec(pcs[0].header.stamp)
        index = bisect.bisect_left(self.timestamps, msg_time) - 1

        if index >= 0:
            closest_past_timestamp = self.timestamps[index]
            time_diff = abs(msg_time - closest_past_timestamp)

            if time_diff <= 0.1:
                point_cloud = []
                for pc in pcs:
                    suc, torch_tf = self.ros_tf_to_torch(self.query_tf(OUTPUT_FRAME, pc.header.frame_id, pc.header.stamp))
                    if not suc:
                        rospy.logerr("Error in tf_callback")
                        return
                    _, pc = self.rospcmsg_to_pctorch(pc)
                    pc = self.project_pc(pc, torch_tf)
                    point_cloud.append(pc)
                point_cloud = torch.cat(point_cloud, dim=0)

                rospy.loginfo(f"Number of timestamps left: {len(self.timestamps)}, saved {msg_time}")

                self.save_pc(msg_time, point_cloud, OUTPUT_PATH)

                # Remove the used timestamp to avoid reprocessing
                del self.timestamps[index]
            else:
                rospy.logwarn(f"Timestamp difference is too large ({time_diff:.3f}s)!")
        else:
            rospy.logwarn(f"Timestamp diff to first msg: {(self.timestamps[0] - msg_time):.3f}s")

    def save_messages(self):
        rospy.spin()

if __name__ == '__main__':
    message_saver = MessageSaver()
    try:
        message_saver.save_messages()
    except rospy.ROSInterruptException:
        rospy.loginfo("Message saver node interrupted and stopped.")