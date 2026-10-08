# Copyright 2026 Kevin Eppacher
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # ---------------------- Paths ------------------------------#

    config_file = os.path.join(
        get_package_share_directory('lerobot_teleop'),
        'config',
        'leader_follower_config.yaml',
    )

    # ---------------------- Nodes ------------------------------#

    teleop_node = LifecycleNode(
        package='lerobot_teleop',
        executable='leader_follower_teleop',
        name='leader_follower_teleop_node',
        namespace='',
        output='screen',
        parameters=[config_file],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(teleop_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(teleop_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    activate_after_configure = OnStateTransition(
        target_lifecycle_node=teleop_node,
        goal_state='inactive',
        entities=[
            activate_event,
        ],
    )

    # ---------------------- Launch Description ------------------------------#

    ld = LaunchDescription()
    ld.add_action(teleop_node)
    ld.add_action(RegisterEventHandler(activate_after_configure))
    ld.add_action(configure_event)
    return ld
