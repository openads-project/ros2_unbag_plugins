# MIT License
#
# Copyright (c) 2026 Institute for Automotive Engineering (ika),
# RWTH Aachen University
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

from geometry_msgs.msg import TransformStamped

from ros2_unbag.core.processors.base import Processor

import tf2_perception_msgs


@Processor("perception_msgs/msg/ObjectList", ["transform"])
def object_list_apply_transform(
    msg,
    translation_x: float,
    translation_y: float,
    translation_z: float,
    rotation_x: float,
    rotation_y: float,
    rotation_z: float,
    rotation_w: float,
):
    """
    Apply a rigid-body transform directly from translation and quaternion components.

    Args:
        msg: ObjectList message instance.
        translation_x: Translation along x-axis.
        translation_y: Translation along y-axis.
        translation_z: Translation along z-axis.
        rotation_x: Quaternion x component.
        rotation_y: Quaternion y component.
        rotation_z: Quaternion z component.
        rotation_w: Quaternion w component.

    Returns:
        ObjectList: Transformed ObjectList message.

    Raises:
        ValueError: If inputs are not numeric or message fields are missing.
    """
    transform = TransformStamped()

    try:
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id

        transform.transform.translation.x = float(translation_x)
        transform.transform.translation.y = float(translation_y)
        transform.transform.translation.z = float(translation_z)
        transform.transform.rotation.x = float(rotation_x)
        transform.transform.rotation.y = float(rotation_y)
        transform.transform.rotation.z = float(rotation_z)
        transform.transform.rotation.w = float(rotation_w)
    except (AttributeError, TypeError, ValueError):
        raise ValueError(
            "Message must provide a header and translation/rotation arguments must be numeric."
        )

    return tf2_perception_msgs.do_transform_object_list(msg, transform)
