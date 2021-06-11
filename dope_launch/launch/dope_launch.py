# Copyright 2021 Intelligent Robotics Lab
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
  stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

  # Create the launch configuration variables
  params_file = LaunchConfiguration('params_file')
  namespace = LaunchConfiguration('namespace')
  use_remappings = LaunchConfiguration('use_remappings')

  dope_dir = get_package_share_directory('dope_launch')

# Create our own temporary YAML files that include substitutions
  param_substitutions = {}

  configured_params = RewrittenYaml(
    source_file=params_file,
    root_key=namespace,
    param_rewrites=param_substitutions,
    convert_types=True)

  declare_namespace_cmd = DeclareLaunchArgument(
    'namespace', default_value='',
    description='Top-level namespace')

  declare_params_cmd = DeclareLaunchArgument(
    'params_file',
    default_value=os.path.join(dope_dir, 'config', 'config_pose.yaml'),
    description='Full path to the TF Pose Estimation parameters file to use')
  
  declare_use_remappings_cmd = DeclareLaunchArgument(
    'use_remappings',
    default_value='true',
    description='Arguments to pass to all nodes launched by the file')

  # Specify the actions
  dope_node_cmd = Node(
    package='dope',
    executable='dope',
    name='dope',
    output='screen',
    parameters=[configured_params]
  )

  # Create the launch description and populate
  ld = LaunchDescription()

  ld.add_action(stdout_linebuf_envvar)
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_params_cmd)
  ld.add_action(declare_use_remappings_cmd)

  # Declare the launch options
  ld.add_action(dope_node_cmd)
  
  return ld