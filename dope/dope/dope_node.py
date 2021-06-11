#!/usr/bin/env python

# Copyright (c) 2018 NVIDIA Corporation. All rights reserved.
# This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
# https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

"""
This file starts a ROS node to run DOPE, 
listening to an image topic and publishing poses.
"""

from __future__ import print_function
import os

import cv2
import message_filters
import numpy as np
import resource_retriever
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from message_filters import Cache, Subscriber
from transforms3d.quaternions import mat2quat, qconjugate, qmult
from PIL import Image
from PIL import ImageDraw
from cv_bridge import CvBridge
from dope_src.inference.cuboid import Cuboid3d
from dope_src.inference.cuboid_pnp_solver import CuboidPNPSolver
from dope_src.inference.detector import ModelData, ObjectDetector
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image as ImageSensor_msg
from std_msgs.msg import String
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from ament_index_python.packages import get_package_share_directory

class Draw(object):
    """Drawing helper class to visualize the neural network output"""

    def __init__(self, im):
        """
        :param im: The image to draw in.
        """
        self.draw = ImageDraw.Draw(im)

    def draw_line(self, point1, point2, line_color, line_width=2):
        """Draws line on image"""
        if point1 is not None and point2 is not None:
            self.draw.line([point1, point2], fill=line_color, width=line_width)

    def draw_dot(self, point, point_color, point_radius):
        """Draws dot (filled circle) on image"""
        if point is not None:
            xy = [
                point[0] - point_radius,
                point[1] - point_radius,
                point[0] + point_radius,
                point[1] + point_radius
            ]
            self.draw.ellipse(xy,
                              fill=point_color,
                              outline=point_color
                              )

    def draw_cube(self, points, color=(255, 0, 0)):
        """
        Draws cube with a thick solid line across
        the front top edge and an X on the top face.
        """

        # draw front
        self.draw_line(points[0], points[1], color)
        self.draw_line(points[1], points[2], color)
        self.draw_line(points[3], points[2], color)
        self.draw_line(points[3], points[0], color)

        # draw back
        self.draw_line(points[4], points[5], color)
        self.draw_line(points[6], points[5], color)
        self.draw_line(points[6], points[7], color)
        self.draw_line(points[4], points[7], color)

        # draw sides
        self.draw_line(points[0], points[4], color)
        self.draw_line(points[7], points[3], color)
        self.draw_line(points[5], points[1], color)
        self.draw_line(points[2], points[6], color)

        # draw dots
        self.draw_dot(points[0], point_color=color, point_radius=4)
        self.draw_dot(points[1], point_color=color, point_radius=4)

        # draw x on the top
        self.draw_line(points[0], points[5], color)
        self.draw_line(points[1], points[4], color)


class DopeNode(Node):
    """ROS node that listens to image topic, runs DOPE, and publishes DOPE results"""
    def __init__(self, last_contexts=None):
        super().__init__('dope_node')
        self.pubs = {}
        self.models = {}
        self.pnp_solvers = {}
        self.pub_dimension = {}
        self.draw_colors = {}
        self.dimensions = {}
        self.class_ids = {}
        self.model_transforms = {}
        self.meshes = {}
        self.mesh_scales = {}
        self.cv_bridge = CvBridge()
        self.prev_num_detections = 0
        
        parameters = [
            ('topic_camera', '', ParameterDescriptor()),
            ('topic_camera_info', '', ParameterDescriptor()),
            ('input_is_rectified', True, ParameterDescriptor()),
            ('downscale_height', 500, ParameterDescriptor()),
            ('thresh_angle', 0.5, ParameterDescriptor()),
            ('thresh_map', 0.01, ParameterDescriptor()),
            ('sigma', 3, ParameterDescriptor()),
            ('resize_out_ratio', 0.1, ParameterDescriptor()),
            ('model_transforms', [], ParameterDescriptor()),
            ('objects', [], ParameterDescriptor()),
            ('draw_colors', '', ParameterDescriptor()),
            ('class_ids', '', ParameterDescriptor()),
            ('topic_publishing', '', ParameterDescriptor()),
        ]
        self.declare_parameters('', parameters)
        topic_publishing = self.get_parameter_or('topic_publishing', 
            Parameter('topic_publishing', type_ = Parameter.Type.STRING, value = ""))._value
        topic_camera = self.get_parameter_or('topic_camera', 
            Parameter('topic_camera', type_ = Parameter.Type.STRING, value = ""))._value
        topic_camera_info = self.get_parameter_or('topic_camera_info', 
            Parameter('topic_camera_info', type_ = Parameter.Type.STRING, value = ""))._value
        self.input_is_rectified = self.get_parameter_or('input_is_rectified', 
            Parameter('input_is_rectified', type_ = Parameter.Type.BOOL, value = True))._value
        self.downscale_height = self.get_parameter_or('downscale_height', 
            Parameter('downscale_height', type_ = Parameter.Type.INTEGER, value = 500))._value
        
        self.config_detect = lambda: None
        self.config_detect.mask_edges = 1
        self.config_detect.mask_faces = 1
        self.config_detect.vertex = 1
        self.config_detect.threshold = 0.5
        self.config_detect.softmax = 1000
        self.config_detect.thresh_angle = self.get_parameter_or('thresh_angle', 
            Parameter('thresh_angle', type_ = Parameter.Type.DOUBLE, value = 0.5))._value
        self.config_detect.thresh_map = self.get_parameter_or('thresh_map', 
            Parameter('thresh_map', type_ = Parameter.Type.DOUBLE, value = 0.01))._value
        self.config_detect.sigma = self.get_parameter_or('sigma', 
            Parameter('sigma', type_ = Parameter.Type.INTEGER, value = 3))._value
        self.config_detect.thresh_points = self.get_parameter_or('thresh_points', 
            Parameter('thresh_points', type_ = Parameter.Type.DOUBLE, value = 0.1))._value
        objects = self.get_parameter_or('objects', 
            Parameter('objects', type_ = Parameter.Type.STRING_ARRAY, value = []))._value
        print (objects)
        
        # For each object to detect, load network model, create PNP solver, and start ROS publishers
        for model in objects:
            print (model)
            self.declare_parameter(model + ".weight_url")
            weights_url = self.get_parameter(model + ".weight_url")._value
            print (weights_url)
            self.models[model] = \
                ModelData(
                    model,
                    os.path.join(get_package_share_directory('dope'), weights_url)
                )
            self.models[model].load_net_model()

            try:
                self.declare_parameter(model + ".model_transforms")
                model_transforms = self.get_parameter_or(model + '.model_transforms', 
                    Parameter(model + '.model_transforms', type_ = Parameter.Type.STRING_ARRAY, value = []))._value
                M = np.array(model_transforms[model], dtype='float64')
                self.model_transforms[model] = mat2quat(M)
            except:
                self.model_transforms[model] = np.array([0.0, 0.0, 0.0, 1.0], dtype='float64')

            try:
                self.declare_parameter(model + ".mesh")
                mesh = self.get_parameter_or(model + '.mesh', 
                    Parameter(model + '.mesh', type_ = Parameter.Type.STRING, value = ""))._value
                self.meshes[model] = mesh
            except:
                pass

            try:
                self.declare_parameter(model + ".mesh_scale")
                mesh_scale = self.get_parameter_or(model + '.mesh_scale', 
                    Parameter(model + ".mesh_scale", type_ = Parameter.Type.DOUBLE, value = 1.0))._value
                self.mesh_scales[model] = mesh_scale
            except:
                self.mesh_scales[model] = 1.0

            try:
                self.declare_parameter(model + ".draw_colors")
                draw_colors = self.get_parameter_or(model + '.draw_colors', 
                    Parameter(model + '.draw_colors', type_ = Parameter.Type.INTEGER_ARRAY, value = []))._value
                self.draw_colors[model] = draw_colors
            except:
                self.draw_colors[model] = (np.random.randint(0,255),np.random.randint(0,255),np.random.randint(0,255))
            
            self.declare_parameter(model + ".dimensions")
            dimensions = self.get_parameter_or(model + '.dimensions', 
                    Parameter(model + '.dimensions', type_ = Parameter.Type.DOUBLE_ARRAY, value = []))._value
            self.dimensions[model] = dimensions

            self.declare_parameter(model + ".class_id")
            class_id = self.get_parameter_or(model + '.class_ids', 
                    Parameter(model + '.class_ids', type_ = Parameter.Type.INTEGER, value = 0))._value
            self.class_ids[model] = class_id

            self.pnp_solvers[model] = \
                CuboidPNPSolver(
                    model,
                    cuboid3d=Cuboid3d(dimensions)
                )
            self.pubs[model] = \
                self.create_publisher(
                    PoseStamped,
                    '{}/pose_{}'.format(topic_publishing, model),
                    10
                )
            self.pub_dimension[model] = \
                self.create_publisher(
                    String,
                    '{}/dimension_{}'.format(topic_publishing, model),
                    10
                )

        # Start ROS publishers
        self.pub_rgb_dope_points = \
            self.create_publisher(
                ImageSensor_msg,
                topic_publishing + "/rgb_points",
                10
            )
        self.pub_camera_info = \
           self.create_publisher(
                CameraInfo,
                topic_publishing + "/camera_info",
                10
            )
        self.pub_detections = \
            self.create_publisher(
                Detection3DArray,
                'detected_objects',
                10
            )
        self.pub_markers = \
            self.create_publisher(
                MarkerArray,
                'markers',
                10
            )

        # Start ROS subscriber
        image_sub = Subscriber(self, ImageSensor_msg, topic_camera)
        info_sub = Subscriber(self, CameraInfo, topic_camera_info)

        ts = message_filters.TimeSynchronizer([image_sub, info_sub], 1)
        ts.registerCallback(self.image_callback)

        print("Running DOPE...  (Listening to camera topic: '{}')".format(topic_camera))
        print("Ctrl-C to stop")

    def destroy(self):
        super().destroy_node()

    def image_callback(self, image_msg, camera_info):
        """Image callback"""

        img = self.cv_bridge.imgmsg_to_cv2(image_msg, "rgb8")
        # cv2.imwrite('img.png', cv2.cvtColor(img, cv2.COLOR_BGR2RGB))  # for debugging

        # Update camera matrix and distortion coefficients
        if self.input_is_rectified:
            P = np.matrix(camera_info.p, dtype='float64')
            P.resize((3, 4))
            camera_matrix = P[:, :3]
            dist_coeffs = np.zeros((4, 1))
        else:
            camera_matrix = np.matrix(camera_info.k, dtype='float64')
            camera_matrix.resize((3, 3))
            dist_coeffs = np.matrix(camera_info.d, dtype='float64')
            dist_coeffs.resize((len(camera_info.d), 1))

        # Downscale image if necessary
        height, width, _ = img.shape
        scaling_factor = float(self.downscale_height) / height
        if scaling_factor < 1.0:
            camera_matrix[:2] *= scaling_factor
            img = cv2.resize(img, (int(scaling_factor * width), int(scaling_factor * height)))

        for m in self.models:
            self.pnp_solvers[m].set_camera_intrinsic_matrix(camera_matrix)
            self.pnp_solvers[m].set_dist_coeffs(dist_coeffs)

        # Copy and draw image
        img_copy = img.copy()
        im = Image.fromarray(img_copy)
        draw = Draw(im)

        detection_array = Detection3DArray()
        detection_array.header = image_msg.header

        for m in self.models:
            # Detect object
            results = ObjectDetector.detect_object_in_image(
                self.models[m].net,
                self.pnp_solvers[m],
                img,
                self.config_detect
            )

            # Publish pose and overlay cube on image
            for i_r, result in enumerate(results):
                if result["location"] is None:
                    continue
                loc = result["location"]
                ori = result["quaternion"]

                # transform orientation
                transformed_ori = qmult(ori, self.model_transforms[m])

                # rotate bbox dimensions if necessary
                # (this only works properly if model_transform is in 90 degree angles)
                dims = rotate_vector(vector=self.dimensions[m], quaternion=self.model_transforms[m])                
                dims = np.absolute(dims)
                dims = tuple(dims)
                pose_msg = PoseStamped()
                pose_msg.header = image_msg.header
                CONVERT_SCALE_CM_TO_METERS = 100
                pose_msg.pose.position.x = loc[0] / CONVERT_SCALE_CM_TO_METERS
                pose_msg.pose.position.y = loc[1] / CONVERT_SCALE_CM_TO_METERS
                pose_msg.pose.position.z = loc[2] / CONVERT_SCALE_CM_TO_METERS
                pose_msg.pose.orientation.x = transformed_ori[0]
                pose_msg.pose.orientation.y = transformed_ori[1]
                pose_msg.pose.orientation.z = transformed_ori[2]
                pose_msg.pose.orientation.w = transformed_ori[3]
                dims_msg = String()
                dims_msg.data = str(dims)
                # Publish
                self.pubs[m].publish(pose_msg)
                self.pub_dimension[m].publish(dims_msg)

                # Add to Detection3DArray
                detection = Detection3D()
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = str(self.class_ids[result["name"]])
                hypothesis.score = float(result["score"])
                hypothesis.pose.pose = pose_msg.pose
                detection.results.append(hypothesis)
                detection.bbox.center = pose_msg.pose
                detection.bbox.size.x = dims[0] / CONVERT_SCALE_CM_TO_METERS
                detection.bbox.size.y = dims[1] / CONVERT_SCALE_CM_TO_METERS
                detection.bbox.size.z = dims[2] / CONVERT_SCALE_CM_TO_METERS
                detection_array.detections.append(detection)

                # Draw the cube
                if None not in result['projected_points']:
                    points2d = []
                    for pair in result['projected_points']:
                        points2d.append(tuple(pair))
                    draw.draw_cube(points2d, tuple(self.draw_colors[m]))

        # Publish the image with results overlaid
        rgb_points_img = CvBridge().cv2_to_imgmsg(np.array(im)[..., ::-1], "bgr8")
        rgb_points_img.header = camera_info.header
        self.pub_rgb_dope_points.publish(rgb_points_img)
        self.pub_camera_info.publish(camera_info)
        self.pub_detections.publish(detection_array)
        self.publish_markers(detection_array)

    def publish_markers(self, detection_array):
        # Object markers
        class_id_to_name = {class_id: name for name, class_id in self.class_ids.items()}
        markers = MarkerArray()
        for i, det in enumerate(detection_array.detections):
            name = class_id_to_name[int(det.results[0].id)]
            color = self.draw_colors[name]

            # cube marker
            marker = Marker()
            marker.header = detection_array.header
            marker.action = Marker.ADD
            marker.pose = det.bbox.center
            marker.color.r = color[0] / 255.0
            marker.color.g = color[1] / 255.0
            marker.color.b = color[2] / 255.0
            marker.color.a = 0.4
            marker.ns = "bboxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.scale = det.bbox.size
            markers.markers.append(marker)

            # text marker
            marker = Marker()
            marker.header = detection_array.header
            marker.action = Marker.ADD
            marker.pose = det.bbox.center
            marker.color.r = color[0] / 255.0
            marker.color.g = color[1] / 255.0
            marker.color.b = color[2] / 255.0
            marker.color.a = 1.0
            marker.id = i
            marker.ns = "texts"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.text = '{} ({:.2f})'.format(name, det.results[0].score)
            markers.markers.append(marker)

            # mesh marker
            try:
                marker = Marker()
                marker.header = detection_array.header
                marker.action = Marker.ADD
                marker.pose = det.bbox.center
                marker.color.r = color[0] / 255.0
                marker.color.g = color[1] / 255.0
                marker.color.b = color[2] / 255.0
                marker.color.a = 0.7
                marker.ns = "meshes"
                marker.id = i
                marker.type = Marker.MESH_RESOURCE
                marker.scale.x = self.mesh_scales[name]
                marker.scale.y = self.mesh_scales[name]
                marker.scale.z = self.mesh_scales[name]
                marker.mesh_resource = self.meshes[name]
                markers.markers.append(marker)
            except:
                # user didn't specify self.meshes[name], so don't publish marker
                pass

        for i in range(len(detection_array.detections), self.prev_num_detections):
            for ns in ["bboxes", "texts", "meshes"]:
                marker = Marker()
                marker.action = Marker.DELETE
                marker.ns = ns
                marker.id = i
                markers.markers.append(marker)
        self.prev_num_detections = len(detection_array.detections)

        self.pub_markers.publish(markers)


def rotate_vector(vector, quaternion):
    q_conj = qconjugate(quaternion)
    vector = np.array(vector, dtype='float64')
    vector = np.append(vector, [0.0])
    vector = qmult(q_conj, vector)
    vector = qmult(vector, quaternion)
    return vector[:3]

def main(args=None):
    """Main routine to run DOPE"""

    rclpy.init(args=args)
    node = DopeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()