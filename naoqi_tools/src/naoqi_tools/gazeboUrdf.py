#!/usr/bin/env python

# Copyright (C) 2014 Aldebaran Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# This file extends urdf parsing lib (urdf.py) to handle any kind of gazebo tag

import string
from xml.dom.minidom import Document
from xml.dom import minidom
import sys
from numpy import array, pi
import re
import copy

ZERO_THRESHOLD = 0.000000001


def reindent(s, numSpaces):
    """Reindent a string for tree structure pretty printing."""
    s = string.split(s, '\n')
    s = [(numSpaces * ' ') + line for line in s]
    s = string.join(s, '\n')
    return s


def add(doc, base, element):
    """Add an XML element for URDF export"""
    if element is not None:
        base.appendChild(element.to_xml(doc))


def add_openrave(doc, base, element):
    """Add an XML element for OpenRAVE XML export"""
    if element is not None:
        # TODO: copy this iterable test elsewhere
        newelements = element.to_openrave_xml(doc)
        if hasattr(newelements, '__iter__'):
            for e in newelements:
                base.appendChild(e)
        else:
            base.appendChild(newelements)


def pfloat(x):
    """Print float value as string"""
    return "{0}".format(x).rstrip('.')


def to_string(data=None):
    """Convert data fromvarious types to urdf string format"""
    if data is None:
        return None
    if hasattr(data, '__iter__'):
        outlist = []
        for a in data:
            try:
                if abs(a) > ZERO_THRESHOLD:
                    outlist.append(pfloat(a))
                else:
                    outlist.append("0")
            except TypeError:
                outlist.append(pfloat(a))
        return ' '.join(outlist)

    elif type(data) == type(0.0):
        return pfloat(data if abs(data) > ZERO_THRESHOLD else 0)
    elif not isinstance(data, str):
        return str(data)
    return data


def set_attribute(node, name, value):
    """Set an attribute on an XML node, converting data to string format"""
    node.setAttribute(name, to_string(value))


def set_content(doc, node, data):
    """Create a text node and add it to the current element"""
    if data is None:
        return
    node.appendChild(doc.createTextNode(to_string(data)))


def short(doc, name, key, value):
    element = doc.createElement(name)
    set_attribute(element, key, value)
    return element


def create_element(doc, name, contents=None, key=None, value=None):
    element = doc.createElement(name)
    if contents is not None:
        set_content(doc, element, contents)
    if key is not None:
        set_attribute(element, key, value)

    return element


def create_child(doc, name, contents=None, key=None, value=None):
    doc.appendChild(create_element(doc, name, contents=None, key=None,
                    value=None))


def children(node):
    children = []
    for child in node.childNodes:
        if child.nodeType is node.TEXT_NODE \
                or child.nodeType is node.COMMENT_NODE:
            continue
        else:
            children.append(child)
    return children

##################################
############# Gazebo #############
##################################


class Gazebo(object):
    """
    Represent any type of gazebo tag:
        - global gazebo tag inside a robot tag
        - gazebo tags referencing links
        - gazebo tags instantiating plugins and sensors
    """
    ATTRIBUTE_NAMES = ['material', 'turnGravityOff', 'dampingFactor',
                       'maxVel', 'minDepth', 'mu1', 'mu2', 'fdir1', 'kp',
                       'kd', 'selfCollide', 'maxContacts', 'laserRetro']

    def __init__(self, reference=None, material=None, gravity=None,
                 dampingFactor=None, maxVel=None, minDepth=None, mu1=None,
                 mu2=None, fdir1=None, kp=None, kd=None, selfCollide=None,
                 maxContacts=None, laserRetro=None, plugin=[], sensor=[]):
        self.reference = reference
        self.material = material
        self.gravity = gravity
        self.dampingFactor = dampingFactor
        self.maxVel = maxVel
        self.minDepth = minDepth
        self.mu1 = mu1
        self.mu2 = mu2
        self.fdir1 = fdir1
        self.kp = kp
        self.kd = kd
        self.selfCollide = selfCollide
        self.maxContacts = maxContacts
        self.laserRetro = laserRetro
        self.plugins = plugin
        self.sensors = sensor

    @staticmethod
    def parse(node, verbose=True):
        gaze = Gazebo()
        gaze.sensors = []
        gaze.plugins = []
        if node.hasAttribute('reference'):
            gaze.reference = node.getAttribute('reference')
            for child in children(node):
                for attribute in gaze.ATTRIBUTE_NAMES:
                    if child.localName == attribute:
                        if attribute == 'sensor':
                            gaze.sensors.append(Sensor.parse(child, verbose))
                        else:
                            setattr(gaze, attribute,
                                    str(child.childNodes[0].nodeValue))
        else:
            for child in children(node):
                if child.localName == 'plugin':
                    gaze.plugins.append(BasePlugin.parse(child, verbose))
        return gaze

    def to_xml(self, doc):
        xml = doc.createElement('gazebo')
        if self.reference is not None:
            set_attribute(xml, 'reference', self.reference)
            for attribute in self.ATTRIBUTE_NAMES:
                value = getattr(self, attribute)
                if value is not None:
                    xml.appendChild(createElement(doc, attribute, value))

            if len(self.sensors) != 0:
                for sensor in self.sensors:
                    add(doc, xml, sensor)
        else:
            if len(self.plugins) != 0:
                for plugin in self.plugins:
                    add(doc, xml, plugin)

        return xml

    def __str__(self):
        s = ""
        if self.reference is not None:
            s += 'Reference: {0}\n'.format(self.reference)
            for name in self.ATTRIBUTE_NAMES:
                value = getattr(self, name)
                if value is not None:
                    s += name.title() + ':\n'
                    s += reindent(str(value, 1)) + '\n'
            for sensor in self.sensors:
                s += 'Sensor'
                s += reindent(str(sensor), 1) + '\n'
        else:
            for plugin in self.plugins:
                s += 'Plugin'
                s += reindent(str(plugin), 1) + '\n'
        return s


###############################################################################
######################### PLUGINS CLASSES #####################################
###############################################################################
class BasePlugin(object):
    """
    Abstract class used to parse plugin tags and instantiating
    inherited plugin accordingly

    """
    ATTRIBUTE_NAMES = []

    def __init__(self, name=None, filename=None, attributes=None):
        self.name = name
        self.filename = filename
        self.attributes = attributes

    @classmethod
    def parse(cls, node, verbose=True):
        for child in children(node):
            if child.localName == 'cameraName':
                plug = CameraPlugin(node.getAttribute('name'),
                                    node.getAttribute('filename'))
                break
            elif child.localName == 'bumperTopicName':
                plug = BumperPlugin(node.getAttribute('name'),
                                    node.getAttribute('filename'))
                break
            elif child.localName == 'leftFrontJoint':
                plug = DrivePlugin(node.getAttribute('name'),
                                   node.getAttribute('filename'))
                break
            elif child.localName == 'odometryTopic':
                plug = OdometryPlugin(node.getAttribute('name'),
                                      node.getAttribute('filename'))
                break
            elif child.localName == 'height':
                plug = VideoPlugin(node.getAttribute('name'),
                                   node.getAttribute('filename'))
                break
            elif child.localName == 'mimicJoint':
                plug = MimicJointPlugin(node.getAttribute('name'),
                                        node.getAttribute('filename'))
                break
            elif child.localName == 'robotSimType':
                plug = SimuPlugin(node.getAttribute('name'),
                                  node.getAttribute('filename'))
            else:
                plug = ImuLaserPlugin()
        for child in children(node):
            if child.localName in cls.ATTRIBUTE_NAMES:
                setattr(plug, child.localName,
                        str(child.childNodes[0].nodeValue))
        return plug

    def to_xml(self, doc):
        xml = doc.createElement('plugin')
        if self.name is not None:
            set_attribute(xml, 'name', self.name)
        if self.filename is not None:
            set_attribute(xml, 'filename', self.filename)
        for attribute in self.ATTRIBUTE_NAMES:
            value = getattr(self, attribute)
            if value is not None:
                xml.appendChild(create_element(doc, attribute, value))
        return xml

    def __str__(self):
        if self.name is not None:
            s = 'Name: {0}\n'.format(self.name)
        if self.filename is not None:
            s += 'Filename: {0}\n'.format(self.filename)
        for attribute in self.ATTRIBUTE_NAMES:
            value = getattr(self, attribute)
            if value is not None:
                s += name.title() + ":\n"
                s += reindent(str(value), 1) + '\n'


class SimuPlugin(BasePlugin):
    """
    Define the simulation plugin used to initialize simulation parameters in
    gazebo plugin
        robotNamespace
        robotSimType
    """
    ATTRIBUTE_NAMES = ['robotNamespace', 'robotSimType']

    def __init__(self, name=None, filename=None, robot_namespace=None,
                 robot_sim_type=None):
        super(SimuPlugin, self).__init__(name, filename,
                                         {'robotNamespace': robot_namespace,
                                          'robotSimType': robot_sim_type})
#        self.filename = filename


class MimicJointPlugin(BasePlugin):
    """
    Define the plugin used to define mimic joints

    (joint not commanded directly by an actuator
     but depending on another joint position)
    plugin
        joint
        mimicjoint
        multiplier
        offset
    """
    ATTRIBUTE_NAMES = ['joint', 'mimicJoint', 'multiplier', 'offset']

    def __init__(self, name=None, filename=None, joint=None, mimic_joint=None,
                 multiplier=None, offset=None):
        super(MimicJointPlugin, self).__init__(name, filename,
                                               {'joint': joint,
                                                'mimicJoint': mimic_joint,
                                                'multiplier': multiplier,
                                                'offset': offset})


# Sensors Plugin
class BumperPlugin(BasePlugin):
    """
    Class defining the plugin used to simulate bumpers on a robot
    plugin
        bumperTopicName
        frameName
        alwaysOn
        updateRate
    """
    ATTRIBUTE_NAMES = ['bumperTopicName', 'frameName', 'alwaysOn',
                       'updateRate']

    def __init__(self, name=None, filename=None, bumper_topic_name=None,
                 frame_name=None, always_on=None, update_rate=None):
        super(BumperPlugin, self).__init__(name, filename,
              {'bumperTopicName': bumper_topic_name,
               'frameName': frame_name,
               'alwaysOn': always_on,
               'updateRate': update_rate})


class ImuLaserPlugin(BasePlugin):
    """
    Class defining the plugin used to IMU on a robot
    note: gazebo official IMU plugin has some weird behaviour

    plugin
        robotNamespace
        topicName
        gaussianNoise
        xyzOffset
        rpyOffset
        alwaysOn
        updateRate
        frameName
        hokuyoMinIntensity
        frameId
        accelGaussianNoise
        headingGaussianNoise
        rateGaussianNoise
        bodyName
    """
    ATTRIBUTE_NAMES = ['robotNamespace', 'topicName', 'gaussianNoise',
                       'xyzOffset', 'rpyOffset', 'alwaysOn', 'updateRate',
                       'frameName', 'hokuyoMinIntensity', 'frameId',
                       'accelGaussianNoise', 'headingGaussianNoise',
                      'rateGaussianNoise', 'bodyName']

    def __init__(self, name=None, filename=None, robot_namespace=None,
                 topic_name=None, gaussian_noise=None, xyz_offset=None,
                 rpy_offset=None, always_on=None, update_rate=None,
                 hokuyo_min_intensity=None, frame_id=None,
                 accel_gaussian_noise=None, heading_gaussian_noise=None,
                 rate_gaussian_noise=None, body_name=None):
        super(ImuLaserPlugin, self).__init__(name, filename,
            {'robotNamespace': robot_namespace,
            'topicName': topic_name,
            'gaussianNoise': gaussian_noise,
            'xyzOffset': xyz_offset,
            'rpyOffset': rpy_offset,
            'alwaysOn': always_on,
            'updateRate': update_rate,
            'hokuyoMinIntensity': hokuyo_min_intensity,
            'frameId': frame_id,
            'accelGaussianNoise': accel_gaussian_noise,
            'headingGaussianNoise': heading_gaussian_noise,
            'rateGaussianNoise': rate_gaussian_noise,
            'bodyName': body_name})


class VideoPlugin(BasePlugin):
    """
    Class defining the plugin used to simulate video on a robot
    plugin
        topicName
        height
        width
        alwaysOn
        updateRate
    """
    ATTRIBUTE_NAMES = ['topicName', 'height', 'width', 'alwaysOn',
                       'updateRate']

    def __init__(self, name=None, filename=None, topic_name=None, height=None,
                 width=None, always_on=None, update_rate=None):
        super(VideoPlugin, self).__init__(name, filename,
                                          {'topicName': topic_name,
                                          'height': height,
                                          'width': width,
                                          'alwaysOn': always_on,
                                          'updateRate': update_rate})


class OdometryPlugin(BasePlugin):
    """
    Class defining the plugin used to odometry on a robot
    plugin
        commandTopic
        odometryTopic
        odometryFrame
        odometryRate
        robotBaseFrame
        alwaysOn
        update_rate
    """
    ATTRIBUTE_NAMES = ['commandTopic', 'odometryTopic', 'odometryFrame',
                       'odometryRate', 'robotBaseFrame', 'alwaysOn',
                       'updateRate']

    def __init__(self, name=None, filename=None, command_topic=None,
                 odometry_topic=None, odometry_frame=None, odometry_rate=None,
                 robot_base_frame=None, always_on=None, update_rate=None):
        super(OdometryPlugin, self).__init__(name, filename,
            {'commandTopic': command_topic,
            'odometryTopic': odometry_topic,
            'odometryFrame': odometry_frame,
            'odometryRate': odometry_rate,
            'robotBaseFrame': robot_base_frame,
            'alwaysOn': always_on,
            'updateRate': update_rate})


#################################
###### SENSOR DECLARATION #######
#################################

class Sensor(object):
    """
    Class defining the sensors and the related plugins
    sensor
        type
        updateRate
        camera
            ...
        plugin
            ...
        pose
        visualize
        ray
    """
    def __init__(self, name=None, type=None, update_rate=None, camera=[],
                 plugin=[], pose=None, visualize=None, ray=None):
        self.name = name
        self.type = type
        self.update_rate = update_rate
        self.cameras = camera
        self.plugins = plugin
        self.pose = pose
        self.visualize = visualize
        self.ray = ray

    @staticmethod
    def parse(node, verbose=True):
        sens = Sensor(node.getAttribute('name'), node.getAttribute('type'))
        sens.plugins = []
        sens.cameras = []
        for child in children(node):
            if child.localName == 'ray':
                sens.ray = Ray.parse(child, verbose)
            if child.localName == 'update_rate':
                sens.update_rate = str(child.childNodes[0].nodeValue)
            if child.localName == 'camera':
                sens.cameras.append(CameraSensor.parse(child, verbose))
            if child.localName == 'plugin':
                sens.plugins.append(BasePlugin.parse(child, verbose))
            if child.localName == 'pose':
                sens.pose = array([float(x) for x in
                                  child.childNodes[0].nodeValue.split(' ')])
            if child.localName == 'visualize':
                sens.visualize = str(child.childNodes[0].nodeValue)
        return sens

    def to_xml(self, doc):
        xml = doc.createElement('sensor')
        set_attribute(xml, 'name', self.name)
        set_attribute(xml, 'type', self.type)
        if self.update_rate is not None:
            xml.appendChild(create_element(doc, 'update_rate',
                                           self.update_rate))
        for camera in self.cameras:
            add(doc, xml, camera)
        for plugin in self.plugins:
            add(doc, xml, plugin)
        if self.pose is not None:
            xml.appendChild(create_element(doc, 'pose', self.pose))
        # add(doc,xml,self.pose)
        if self.visualize is not None:
            xml.appendChild(create_element(doc, 'visualize', self.visualize))
        if self.ray is not None:
            add(doc, xml, self.ray)
        return xml

    def __str__(self):
        s = ""
        s += "Name: {0}\n".format(self.name)
        s += "Type: {0}\n".format(self.type)
        if self.update_rate is not None:
            s += "Update_rate:\n"
            s += reindent(str(self.update_rate), 1) + "\n"
        for camera in self.cameras:
            s += 'Camera'
            s += reindent(str(camera), 1) + "\n"
        for plugin in self.plugins:
            s += 'Plugin'
            s += reindent(str(plugin), 1) + "\n"

        if self.visualize is not None:
            s += "Visualize:\n"
            s += reindent(str(self.visualize), 1) + "\n"

        if self.ray is not None:
            s += "Ray:\n"
            s += reindent(str(self.ray), 1) + "\n"
        return s


class CameraSensor(object):
    """
    Class defining the camera sensor.

    This can instantiate specific plugin according to the type of camera
    (RGB,Stereo,Depth...) camera
        horizontal_fov
        image
        clip
        noise
    """
    def __init__(self, name=None, horizontal_fov=None, image=None, clip=None,
                 noise=None):
        self.name = name
        self.horizontal_fov = horizontal_fov
        self.image = image
        self.clip = clip
        self.noise = noise

    @staticmethod
    def parse(node, verbose=True):
        sens = CameraSensor()
        if node.hasAttribute('name'):
            sens.name = node.getAttribute('name')
        for child in children(node):
            if child.localName == 'horizontal_fov':
                sens.horizontal_fov = str(child.childNodes[0].nodeValue)
            elif child.localName == 'image':
                sens.image = Image.parse(child, verbose)
            elif child.localName == 'clip':
                sens.clip = Clip.parse(child, verbose)
            elif child.localName == 'noise':
                sens.noise = Noise.parse(child, verbose)
        return sens

    def to_xml(self, doc):
        xml = doc.createElement('camera')
        if self.name is not None:
            set_attribute(xml, 'name', self.name)
        if self.horizontal_fov is not None:
            xml.appendChild(create_element(doc, 'horizontal_fov',
                            self.horizontal_fov))
        if self.image is not None:
            add(doc, xml, self.image)
        if self.clip is not None:
            add(doc, xml, self.clip)
        if self.noise is not None:
            add(doc, xml, self.noise)
        return xml

    def __str__(self):
        s = ''
        if self.name is not None:
            s = "Name: {0}\n".format(self.name)
        if self.horizontal_fov is not None:
            s += "Horizontal_fov:\n"
            s += reindent(str(self.horizontal_fov), 1) + '\n'
        if self.image is not None:
            s += "Image:\n"
            s += reindent(str(self.image), 1) + '\n'
        if self.clip is not None:
            s += "Clip:\n"
            s += reindent(str(self.clip), 1) + '\n'
        if self.noise is not None:
            s += "Noise:\n"
            s += reindent(str(self.noise), 1) + '\n'
        return s


################################
##### Auxiliary Elements #######
################################

class Clip(object):
    """
    Class defining the clip element used in sensors

    (for range or beaming sensors)
    structure:
        clip
            near
            far
    """
    def __init__(self, near=None, far=None):
        self.near = near
        self.far = far

    @staticmethod
    def parse(node, verbose=True):
        clip = Clip()
        for child in children(node):
            if child.localName == 'near':
                clip.near = str(child.childNodes[0].nodeValue)
            elif child.localName == 'far':
                clip.far = str(child.childNodes[0].nodeValue)
        return clip

    def to_xml(self, doc):
        xml = doc.createElement('clip')
        if self.near is not None:
            xml.appendChild(create_element(doc, 'near', self.near))
        if self.far is not None:
            xml.appendChild(create_element(doc, 'far', self.far))
        return xml

    def __str__(self):
        s = ''
        if self.near is not None:
            s += "Near:\n"
            s += reindent(str(self.near), 1) + '\n'
        if self.far is not None:
            s += "Far:\n"
            s += reindent(str(self.far), 1) + '\n'
        return s


class Image(object):
    """
    Class defining the image element used in camera sensor
    image
        height
        width
        format
    """
    def __init__(self, height=None, width=None, format=None):
        self.height = height
        self.width = width
        self.format = format

    @staticmethod
    def parse(node, verbose=True):
        img = Image()
        for child in children(node):
            if child.localName == 'height':
                img.height = str(child.childNodes[0].nodeValue)
            elif child.localName == 'width':
                img.width = str(child.childNodes[0].nodeValue)
            elif child.localName == 'format':
                img.format = str(child.childNodes[0].nodeValue)
        return img

    def to_xml(self, doc):
        xml = doc.createElement('image')
        if self.height is not None:
            xml.appendChild(create_element(doc, 'height', self.height))
        if self.width is not None:
            xml.appendChild(create_element(doc, 'width', self.width))
        if self.format is not None:
            xml.appendChild(create_element(doc, 'format', self.format))
        return xml

    def __str__(self):
        s = ''
        if self.height is not None:
            s += "Height:\n"
            s += reindent(str(self.height), 1) + '\n'
        if self.width is not None:
            s += "Width:\n"
            s += reindent(str(self.width), 1) + '\n'
        if self.format is not None:
            s += "Format:\n"
            s += reindent(str(self.format), 1) + '\n'
        return s


class Noise(object):
    """
    Class defining the noise element used in most sensors definition
    noise
        type
        mean
        stddev
    """
    def __init__(self, type=None, mean=None, stddev=None):
        self.type = type
        self.mean = mean
        self.stddev = stddev

    @staticmethod
    def parse(node, verbose=True):
        noise = Noise()
        for child in children(node):
            if child.localName == 'type':
                noise.type = str(child.childNodes[0].nodeValue)
            elif child.localName == 'mean':
                noise.mean = str(child.childNodes[0].nodeValue)
            elif child.localName == 'stddev':
                noise.stddev = str(child.childNodes[0].nodeValue)
        return noise

    def to_xml(self, doc):
        xml = doc.createElement('noise')
        if self.type is not None:
            xml.appendChild(create_element(doc, 'type', self.type))
        if self.mean is not None:
            xml.appendChild(create_element(doc, 'mean', self.mean))
        if self.stddev is not None:
            xml.appendChild(create_element(doc, 'stddev', self.stddev))
        return xml

    def __str__(self):
        s = ''
        if self.type is not None:
            s += "Type:\n"
            s += reindent(str(self.type), 1) + '\n'
        if self.mean is not None:
            s += "Mean:\n"
            s += reindent(str(self.mean), 1) + '\n'
        if self.stddev is not None:
            s += "Stddev:\n"
            s += reindent(str(self.stddev), 1) + '\n'
        return s


class Ray(object):
    """
    Class defining the ray element used in most range sensors
    ray
        scan
        range
        noise
    """
    def __init__(self, scan=None, range=None, noise=None):
        self.scan = scan
        self.range = range
        self.noise = noise

    @staticmethod
    def parse(node, verbose=True):
        ray = Ray()
        for child in children(node):
            if child.localName == 'scan':
                ray.scan = Scan.parse(child, verbose)
                # str(child.childNodes[0].nodeValue)
            elif child.localName == 'range':
                ray.range = Range.parse(child, verbose)
                # str(child.childNodes[0].nodeValue)
            elif child.localName == 'noise':
                ray.noise = Noise.parse(child, verbose)
        return ray

    def to_xml(self, doc):
        xml = doc.createElement('ray')
        if self.scan is not None:
            add(doc, xml, self.scan)
        if self.range is not None:
            add(doc, xml, self.range)
        if self.noise is not None:
            add(doc, xml, self.noise)
        return xml

    def __str__(self):
        s = ''
        if self.scan is not None:
            s += "Scan:\n"
            s += reindent(str(self.scan), 1) + '\n'
        if self.range is not None:
            s += "Range:\n"
            s += reindent(str(self.range), 1) + '\n'
        if self.noise is not None:
            s += "Noise:\n"
            s += reindent(str(self.noise), 1) + '\n'
        return s


class Range(object):
    """
    Class defining the range element used in most range sensors (mostly LASERs)
    range:
        min
        max
        resolution
    """
    def __init__(self, min=None, max=None, resolution=None):
        self.min = min
        self.max = max
        self.resolution = resolution

    @staticmethod
    def parse(node, verbose=True):
        rang = Range()
        for child in children(node):
            if child.localName == 'min':
                rang.min = str(child.childNodes[0].nodeValue)
            elif child.localName == 'max':
                rang.max = str(child.childNodes[0].nodeValue)
            elif child.localName == 'resolution':
                rang.resolution = str(child.childNodes[0].nodeValue)
        return rang

    def to_xml(self, doc):
        xml = doc.createElement('range')
        if self.min is not None:
            xml.appendChild(create_element(doc, 'min', self.min))
        if self.max is not None:
            xml.appendChild(create_element(doc, 'max', self.max))
        if self.resolution is not None:
            xml.appendChild(create_element(doc, 'resolution', self.resolution))
        return xml

    def __str__(self):
        s = ''
        if self.min is not None:
            s += "Min:\n"
            s += reindent(str(self.min), 1) + '\n'
        if self.max is not None:
            s += "Max:\n"
            s += reindent(str(self.max), 1) + '\n'
        if self.resolution is not None:
            s += "Resolution:\n"
            s += reindent(str(self.resolution), 1) + '\n'
        return s


class Scan(object):
    """
    Class defining the scan element used in most range sensors (mostly LASERs)
    scan
        horizontal
    """
    def __init__(self, horizontal=None):
        self.horizontal = horizontal

    @staticmethod
    def parse(node, verbose=True):
        scan = Scan()
        for child in children(node):
            if child.localName == 'horizontal':
                scan.horizontal = Horizontal.parse(child, verbose)
                # str(child.childNodes[0].nodeValue)
        return scan

    def to_xml(self, doc):
        xml = doc.createElement('scan')
        if self.horizontal is not None:
            add(doc, xml, self.horizontal)
        return xml

    def __str__(self):
        s = ''
        if self.horizontal is not None:
            s += "Horizontal:\n"
            s += reindent(str(self.horizontal), 1) + '\n'
        return s


class Horizontal(object):
    """
    Class defining the horizontal element
    used in most range sensors(mostly LASERs)
    horizontal
        samples
        resolution
        min_angle
        max_angle
    """
    def __init__(self, samples=None, resolution=None, min_angle=None,
                 max_angle=None):
        self.samples = samples
        self.resolution = resolution
        self.min_angle = min_angle
        self.max_angle = max_angle

    @staticmethod
    def parse(node, verbose=True):
        hori = Horizontal()
        for child in children(node):
            if child.localName == 'samples':
                hori.samples = str(child.childNodes[0].nodeValue)
            elif child.localName == 'resolution':
                hori.resolution = str(child.childNodes[0].nodeValue)
            elif child.localName == 'min_angle':
                hori.min_angle = str(child.childNodes[0].nodeValue)
            # Noise.parse(child, verbose)
            elif child.localName == 'max_angle':
                hori.max_angle = str(child.childNodes[0].nodeValue)
        return hori

    def to_xml(self, doc):
        xml = doc.createElement('horizontal')
        if self.samples is not None:
            xml.appendChild(create_element(doc, 'samples', self.samples))
        if self.resolution is not None:
            xml.appendChild(create_element(doc, 'resolution', self.resolution))
        if self.min_angle is not None:
            xml.appendChild(create_element(doc, 'min_angle', self.min_angle))
        if self.max_angle is not None:
            xml.appendChild(create_element(doc, 'max_angle', self.samples))

        return xml

    def __str__(self):
        s = ''
        if self.samples is not None:
            s += "Samples:\n"
            s += reindent(str(self.samples), 1) + '\n'
        if self.resolution is not None:
            s += "Resolution:\n"
            s += reindent(str(self.resolution), 1) + '\n'
        if self.min_angle is not None:
            s += "Min_angle:\n"
            s += reindent(str(self.min_angle), 1) + '\n'
        if self.max_angle is not None:
            s += "Max_angle:\n"
            s += reindent(str(self.max_angle), 1) + '\n'
        return s
