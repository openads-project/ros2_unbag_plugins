# MIT License
#
# Copyright (c) 2026 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import csv
from geometry_msgs.msg import Quaternion
from math import cos, sin
from pathlib import Path
try:
    import perception_msgs_utils
except ImportError:
    perception_msgs_utils = None
from rclpy.time import Time
from tf_transformations import euler_from_quaternion

from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata


@ExportRoutine("perception_msgs/msg/ObjectList", ["object_list/HEXAMOTION"], mode=ExportMode.MULTI_FILE)
def export_object_list_hexamotion(msg, path: Path, fmt: str, metadata: ExportMetadata):
    
    header = [
        'id', 'existence_prob', 'class', 'class_prob', 'x', 'y', 'z',
        'vel_abs', 'vel_lon', 'vel_lat', 'acc_abs', 'acc_lon', 'acc_lat',
        'roll', 'pitch', 'yaw', 'roll_rate', 'pitch_rate', 'yaw_rate',
        'width', 'length', 'height'
    ]
    
    rows = []

    for obj in msg.objects:
        
        id = obj.id
        existence_prob = obj.existence_probability

        # Ensure classifications is a list with at least one entry
        if not obj.state.classifications or not isinstance(obj.state.classifications, list):
            raise ValueError(f"Object {obj.id} has no valid classifications.")

        # Find the classification with the highest probability 
        # returns the first one if there are multiple with the same probability
        classification_with_max_probability = perception_msgs_utils.get_class_with_highest_probability(obj)

        position = perception_msgs_utils.get_position(obj)

        vel_abs = perception_msgs_utils.get_velocity_magnitude(obj)
        vel_lat_lon = perception_msgs_utils.get_velocity(obj)

        acc_abs = perception_msgs_utils.get_acceleration_magnitude(obj)
        acc_lat_lon = perception_msgs_utils.get_acceleration(obj)

        orientation = perception_msgs_utils.get_orientation(obj)
        roll, pitch, yaw  = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        if obj.state.model_id == 17:  # HEXAMOTION model
            roll_rate = obj.state.continuous_state[10] # TODO: Add getter function to perception_msgs_utils
            pitch_rate = obj.state.continuous_state[11] # TODO: Add getter function to perception_msgs_utils
            yaw_rate = obj.state.continuous_state[12] # TODO: Add getter function to perception_msgs_utils
        elif obj.state.model_id == 16:  # ISCACTR model
            roll_rate = 0.0
            pitch_rate = 0.0
            yaw_rate = obj.state.continuous_state[8] # TODO: Add getter function to perception_msgs_utils

        length = perception_msgs_utils.get_length(obj)
        width = perception_msgs_utils.get_width(obj)
        height = perception_msgs_utils.get_height(obj)

        type_to_str = {
            classification_with_max_probability.UNCLASSIFIED: "unclassified",
            classification_with_max_probability.PEDESTRIAN: "pedestrian",
            classification_with_max_probability.BICYCLE: "bicycle",
            classification_with_max_probability.MOTORCYCLE: "motorcycle",
            classification_with_max_probability.CAR: "car",
            classification_with_max_probability.UTILITY: "utility",
            classification_with_max_probability.VAN: "car",
            classification_with_max_probability.BUS: "bus",
            classification_with_max_probability.ANIMAL: "animal",
            classification_with_max_probability.TRAILER: "utility",
            classification_with_max_probability.TRAIN: "utility",
            classification_with_max_probability.VRU: "vru",
            classification_with_max_probability.MICRO: "micro",
            classification_with_max_probability.UNKNOWN: "unknown",
        }

        classification_type_str = type_to_str.get(
            classification_with_max_probability.type,
            "unknown",
        )

        row = [
            id, existence_prob, classification_type_str,
            classification_with_max_probability.probability,
            position.x, position.y, position.z,
            vel_abs, vel_lat_lon.x, vel_lat_lon.y,
            acc_abs, acc_lat_lon.x, acc_lat_lon.y,
            roll, pitch, yaw,
            roll_rate, pitch_rate, yaw_rate,
            width, length, height
        ]

        rows.append(row)

    with open(path.with_suffix(".csv"), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)

@ExportRoutine("perception_msgs/msg/ObjectList", ["object_list/FUSION"], mode=ExportMode.MULTI_FILE)
def export_object_list_fusion(msg, path: Path, fmt: str, metadata: ExportMetadata):
    
    header = ['timestamp', 'id', 'existence_prob',
              'x', 'y', 'z',
              'vel_x', 'vel_y', 'vel_z',
              'acc_x', 'acc_y', 'acc_z',
              'length', 'width', 'height',
              'heading', 'yaw_rate',
              'var_pos_x', 'var_pos_y', 'var_pos_z',
              'var_vel_x', 'var_vel_y', 'var_vel_z',
              'var_acc_x', 'var_acc_y', 'var_acc_z',
              'var_length', 'var_width', 'var_height',
              'var_heading', 'var_yaw_rate',
              'class', 'class_prob']
    
    rows = []

    for obj in msg.objects:
        
        if obj.state.model_id != 16:  # Only export objects with model_id 16 (ISCACTR)
            print(f"Skipping object with id {obj.id} and model_id {obj.state.model_id}. Only model_id 16(ISCACTR) is exported for FUSION.")
            continue

        id = obj.id

        timestamp = Time.from_msg(obj.state.header.stamp).nanoseconds

        existence_prob = obj.existence_probability

        # Ensure classifications is a list with at least one entry
        if not obj.state.classifications or not isinstance(obj.state.classifications, list):
            raise ValueError(f"Object {obj.id} has no valid classifications.")

        # Find the classification with the highest probability 
        # returns the first one if there are multiple with the same probability
        classification_with_max_probability = perception_msgs_utils.get_class_with_highest_probability(obj)

        state = obj.state.continuous_state

        vel_lon = state[3]  # Longitudinal velocity
        vel_lat = state[4]  # Lateral velocity
        yaw = state[7]

        vel_x = vel_lon * cos(yaw) - vel_lat * sin(yaw)
        vel_y = vel_lon * sin(yaw) + vel_lat * cos(yaw)
        vel_z = 0

        acc_lon = state[5]  # Longitudinal acceleration
        acc_lat = state[6]  # Lateral acceleration
        acc_x = acc_lon * cos(yaw) - acc_lat * sin(yaw)
        acc_y = acc_lon * sin(yaw) + acc_lat * cos(yaw)
        acc_z = 0

        # Variances
        cov = obj.state.continuous_state_covariance

        var_pos_x = cov[0]
        var_pos_y = cov[1*12 + 1]
        var_pos_z = cov[2*12 + 2]

        var_vel_lon = cov[3*12 + 3]
        var_vel_lat = cov[4*12 + 4]
        cov_vel_lonlat = cov[3*12 + 4]
        var_vel_x = var_vel_lon * cos(yaw)**2 + var_vel_lat * sin(yaw)**2 - 2 * cov_vel_lonlat * cos(yaw) * sin(yaw)
        var_vel_y = var_vel_lon * sin(yaw)**2 + var_vel_lat * cos(yaw)**2 + 2 * cov_vel_lonlat * cos(yaw) * sin(yaw)
        var_vel_z = 0

        var_acc_lon = cov[5*12 + 5]
        var_acc_lat = cov[6*12 + 6]
        cov_acc_lonlat = cov[5*12 + 6]
        var_acc_x = var_acc_lon * cos(yaw)**2 + var_acc_lat * sin(yaw)**2 - 2 * cov_acc_lonlat * cos(yaw) * sin(yaw)
        var_acc_y = var_acc_lon * sin(yaw)**2 + var_acc_lat * cos(yaw)**2 + 2 * cov_acc_lonlat * cos(yaw) * sin(yaw)
        var_acc_z = 0

        var_yaw = cov[7*12 + 7]
        var_yaw_rate = cov[8*12 + 8]
        var_length = cov[10*12 + 10]
        var_width = cov[9*12 + 9]
        var_height = cov[11*12 + 11]

        row = [
            timestamp, id, existence_prob,
            state[0], state[1], state[2],  # x, y, z
            vel_x, vel_y, vel_z,  # vel_x, vel_y, vel_z
            acc_x, acc_y, acc_z,  # acc_x, acc_y, acc_z
            state[10], state[9], state[11],  # length, width, height
            yaw, state[8],  # heading, yaw_rate
            var_pos_x, var_pos_y, var_pos_z,  # var_pos_x, var_pos_y, var_pos_z
            var_vel_x, var_vel_y, var_vel_z,  # var_vel_x, var_vel_y, var_vel_z
            var_acc_x, var_acc_y, var_acc_z,  # var_acc_x, var_acc_y, var_acc_z
            var_length, var_width, var_height,  # var_length, var_width, var_height
            var_yaw, var_yaw_rate,  # var_heading, var_yaw_rate
            classification_with_max_probability.type,
            classification_with_max_probability.probability
        ]

        rows.append(row)

    with open(path + ".csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)


