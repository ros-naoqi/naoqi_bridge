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
#
# This file takes the official URDF aldebaran files and convert them
# to rep120 compliant urdf files. It also includes the meshes from nao_meshes
# package allowing to display the model in RVIZ
#
# authors: Mikael Arguedas [mikael DOT arguedas AT gmail DOT com]
# TODO Get motor information from documentation and generate transmission tags
# automatically
# TODO Generate automatically gazebo tags for every sensor
# TODO Add toe frames for romeo (not supported yet by NAOqi)

from __future__ import print_function
import sys
import argparse
from naoqi_tools.urdf import URDF
import copy
import naoqi_tools.gazeboUrdf
import naoqi_tools.urdf as ur
import naoqi_tools.nao_dictionaries as dico
import subprocess
import os
import math
from xml.dom.minidom import Document

NAO_XACRO_DICO = {
    'head': 'gaze',
    'legs': 'sole',
    'arms': 'gripper',
    'torso': 'torso',
    }

ROMEO_XACRO_DICO = {
    'head': 'gaze',
    'legs': 'sole',
    'arms': 'gripper',
    'torso': 'body',
    'eyes': 'Eye',
    }

PEPPER_XACRO_DICO = {
    'head': 'Head',
    'legs': 'base_footprint',
    'arms': 'gripper',
    'torso': 'torso',
    }

COLLISION_SUFFIX = '_0.10.stl'

parser = argparse.ArgumentParser(usage='Load an URDF file')
parser.add_argument('-i', '--input', default='', help='URDF file to load')
parser.add_argument('-r', '--REP120', choices=['true', 'false'],
    default='true', help='Rename the links to be REP120 compliant')
parser.add_argument('-x', '--xacro', choices=['urdf', 'robot'],
default='robot', help='Chose robot part to generate. choosing urdf create a'
    'single urdf file with the entire robot. robot will OUTPUT a xacro file '
    'for every kinematic chain on the robot')

####################
##### FUNCTIONS ####
####################


def define_materials():
    """Create a few materials.

    to display geometrical shapes in a given color.
    """
    global robot
    robot.add_material(ur.Material('Black', ur.Color(0.1, 0.1, 0.1, 1)))
    robot.add_material(ur.Material('LightGrey', ur.Color(0.9, 0.9, 0.9, 1)))
    robot.add_material(ur.Material('Grey', ur.Color(0.6, 0.6, 0.6, 1)))
    robot.add_material(ur.Material('DarkGrey', ur.Color(0.3, 0.3, 0.3, 1)))


def REP120_compatibility():
    """Add frames defined by ROS for humanoid robots.

    (REP120): http://www.ros.org/reps/rep-0120.html
    """
    # TODO Add toe frames for ROMEO (not supported by NAOqi yet)
    global robot, NAME, MESH_VERSION, VERSION, LINKS_DICO, OFFSETS_DICO
    print('creating and renaming joints & links to comply to REP120')

    # Rename links
    for joint in robot.joints:
        if robot.joints[joint].name.endswith('_joint'):
            robot.joints[joint].name = robot.joints[joint].name[0:-6]
        if robot.joints[joint].name.endswith('_actuator'):
            robot.joints[joint].name = robot.joints[joint].name[0:-9]
        if robot.joints[joint].mimic is not None:
            if robot.joints[joint].mimic.joint_name.endswith('_actuator'):
                robot.joints[joint].mimic.joint_name = \
                    robot.joints[joint].mimic.joint_name[0:-9]
            if robot.joints[joint].mimic.joint_name.endswith('_joint'):
                robot.joints[joint].mimic.joint_name = \
                    robot.joints[joint].mimic.joint_name[0:-6]
        try:
            robot.joints[joint].parent = LINKS_DICO[robot.joints[joint].parent]
        except KeyError:
            pass
        try:
            robot.joints[joint].child = LINKS_DICO[robot.joints[joint].child]
        except KeyError:
            pass
        for link in robot.links.keys():
            try:
                robot.rename_link(link, LINKS_DICO[link])
            except KeyError, ValueError:
                pass

    if NAME == 'romeo':
        robot.add_link(ur.Link('gaze'))
        robot.add_joint(ur.Joint('gaze_joint', 'HeadRoll_link',
            'gaze', 'fixed', None, ur.Pose(
                (OFFSETS_DICO['CameraLeftEyeOffsetX'], 0,
                OFFSETS_DICO['CameraLeftEyeOffsetZ']), (0, 0, 0))))
        MESH_VERSION = ''

    elif NAME == 'nao':
        robot.add_link(ur.Link('gaze'))
        robot.add_joint(ur.Joint('gaze_joint', 'Head',
            'gaze', 'fixed', None, ur.Pose(
                (OFFSETS_DICO['CameraTopV4OffsetX'], 0,
                OFFSETS_DICO['CameraTopV4OffsetZ']), (0, 0, 0))))
        if VERSION == 'V32':
            MESH_VERSION = VERSION
        elif VERSION == 'V33' or VERSION == 'V40' or VERSION == 'V50':
            MESH_VERSION = 'V40'

    elif NAME == 'pepper':
        MESH_VERSION = VERSION
        # add base_footprint frame
        robot.add_link(ur.Link('base_footprint'))
        robot.add_joint(ur.Joint('base_footprint_joint', 'Tibia',
            'base_footprint', 'fixed', None, ur.Pose(
                (OFFSETS_DICO['BaseFootprintOffsetX'],
                OFFSETS_DICO['BaseFootprintOffsetY'],
                OFFSETS_DICO['BaseFootprintOffsetZ']),
                (OFFSETS_DICO['BaseFootprintRotX'],
                OFFSETS_DICO['BaseFootprintRotY'],
                OFFSETS_DICO['BaseFootprintRotZ']))))

        # rename the laser frames to sensor frames
        # (they are actually not used for computation)
        laser_links = [c for c in robot.links.keys()
            if 'surrounding' in c.lower()]
        for joint in robot.joints.values():
            if joint.child in laser_links:
                laser_frame = joint.child
                laser_device_frame = laser_frame[:-5] + 'device_frame'
                # get the old joint to have the device frame as a child
                joint.child = laser_device_frame
                # but also create a joint with the projected frame as a child
                robot.add_link(ur.Link(laser_device_frame))
                joint_new = copy.deepcopy(joint)
                joint_new.name = joint.name[:-17] + \
                    'projected_sensor_fixedjoint'
                joint_new.child = laser_frame
                joint_new.origin.rotation[0] = 0
                joint_new.origin.rotation[1] = 0
                # set it on the ground
                joint_new.origin.position[2] = -0.334
                if 'left' in laser_frame.lower():
                    # the following line is a temporary fix
                    # that should be fixed upstream
                    joint_new.origin.rotation[2] = math.pi/2.0 + \
                        0.1864836732051034
                elif 'right' in laser_frame.lower():
                    # the following line is a temporary fix
                    # that should be fixed upstream
                    joint.origin.position[0] = -0.018
                    joint_new.origin.position[0] = -0.018
                    # the following line is a temporary fix
                    # that should be fixed upstream
                    joint_new.origin.rotation[2] = -math.pi/2.0 \
                        - 0.1864836732051034
                elif 'front' in laser_frame.lower():
                    joint_new.origin.rotation[2] = 0
                robot.add_joint(joint_new)

    # add an optical frame for each robot
    camera_frames = [c for c in robot.links.keys() if 'camera' in c.lower()]
    for camera_frame in camera_frames:
        camera_optical_frame = camera_frame[:-6] + '_optical_frame'
        robot.add_link(ur.Link(camera_optical_frame))
        robot.add_joint(ur.Joint('%s_fixedjoint' % camera_optical_frame,
            camera_frame, camera_optical_frame, 'fixed', None,
            ur.Pose((0, 0, 0), (-math.pi/2.0, 0, -math.pi/2.0))))

    # add dummy physics for gazebo simulation
    add_dummy_inertia(['Finger', 'Thumb', 'gripper', 'Fsr'])
    add_dummy_collision(['Fsr'])


def add_transmission_tags():
    """Should instanciate all transmission tags.

    - hardware interface
    - mechanical reduction ratio for each motor
    - joint and actuator to which the transmission tag reference
    for now naoTransmission.xacro has been done by hand based on the work
    of Konstantinos Chatzilygeroudis in his nao_dcm project
    https://github.com/costashatz/nao_dcm
    """
    global robot
    # TODO create all transmission elements : Cannot get them from the lib for now
    return


def add_gazebo_tags():
    """Should instanciate all gazebo tags.

        - sensor plugins
        - mimic joints plugins
        - ros_control plugin
        - disable_links plugins
        - gazebo reference for every link (ok)
    for now naoGazebo.xacro has been done by hand based on the work of
    Konstantinos Chatzilygeroudis in his nao_dcm project
    https://github.com/costashatz/nao_dcm
    """
    global robot
    # TODO instantiate plugins according to sensors present in input urdf
    return


def add_dummy_inertia(list):
    """Add a dummy Inertial tag to every links containing keyword in list."""
    global robot
    for string in list:
        for link in robot.links:
            if robot.links[link].name.find(string) != -1:
                robot.links[link].inertial = ur.Inertial(1.1e-9, 0.0, 0.0,
                    1.1e-9, 0.0, 1.1e-9, 2e-06)


def add_dummy_collision(list):
    """Add a Box collision tag to every links containing keyword in list."""
    global robot
    for string in list:
        for link in robot.links:
            if robot.links[link].name.find(string) != -1:
                robot.links[link].collision = ur.Collision(
                    ur.Box([0.01, 0.01, 0.005]),
                    ur.Pose((0, 0, 0), (0, 0, 0)))

##################
##### Meshes #####
##################


def create_visual_xacro():
    """Create a <ROBOT>_visual_collision.xacro file.

    with xacro macros for visual and collision tags of the urdf.
    This function creates xacro macros for visualisation and collisions.
    It checks if the meshes are on the computer and set the 'meshes_installed'
    property accordingly
    """
    global robot
    global OUTPUT
    global MESH_VERSION
    global NAME
    global LINKS_DICO
    global VISU_DICO
    global OFFSETS_DICO
    global MESHPKG
    prefix = 'insert_visu_'
    doc = Document()
    root = doc.createElement('robot')
    doc.appendChild(root)
    root.setAttribute("xmlns:xacro", "http://www.ros.org/wiki/xacro")

    cmd = 'rospack find ' + NAME + MESHPKG
    try:
        path_mesh_pkg = subprocess.check_output(cmd, stderr=subprocess.STDOUT,
            shell=True)[:-1]
    except:
        print('unable to find ' + NAME + MESHPKG + ' package')
        sys.exit(0)
    # Set Mesh path
    if NAME == 'nao':
        node = ur.short(doc, 'xacro:property', 'name', 'PI_2')
        node.setAttribute('value', str(math.pi/2.0))
        root.appendChild(node)
        node = ur.short(doc, 'xacro:property', 'name', 'meshes_installed')

        if os.path.isdir(os.path.join(path_mesh_pkg, 'meshes', MESH_VERSION)):
            node.setAttribute('value', 'true')
        else:
            node.setAttribute('value', 'false')
        root.appendChild(node)

    # Insert xacro macro
    for link in robot.links:
        (tempVisu, tempCol) = adjustMeshPath(path_mesh_pkg, link)
        if robot.links[link].visual is not None:
            robot.links[link].xacro = 'xacro:' + prefix + \
                robot.links[link].name
            node = ur.short(doc, 'xacro:macro', 'name', prefix +
                robot.links[link].name)
            if NAME == 'nao':
                # add xacro condition macro to handle the absence of meshes
                node2 = ur.short(doc, 'xacro:unless', 'value',
                    '${meshes_installed}')
                if tempVisu is not None:
                    node2.appendChild(tempVisu.to_xml(doc))
                if tempCol is not None:
                    node2.appendChild(tempCol.to_xml(doc))
                node.appendChild(node2)
                node3 = ur.short(doc, 'xacro:if', 'value', '${meshes_installed}')
                node3.appendChild(robot.links[link].visual.to_xml(doc))
                node3.appendChild(robot.links[link].collision.to_xml(doc))
                node.appendChild(node3)
            else:
                node.appendChild(robot.links[link].visual.to_xml(doc))
                node.appendChild(robot.links[link].collision.to_xml(doc))
            root.appendChild(node)
        robot.links[link].visual = None
        robot.links[link].collision = None
    filename = OUTPUT[0:OUTPUT.rfind('.')] + '_visual_collisions.xacro'
    write_comments_in_xacro(doc, filename)


#################################
######## XACRO FUNCTIONS ########
#################################


def export_robot_element(element):
    """
    Export the 'elements' related to the keyword 'element'.

    :param : element, string in ['Transmission', 'Gazebo', 'material']

    The output file is <ROBOT>_<element>.xacro
    """
    global robot, OUTPUT
    doc = Document()
    root = doc.createElement('robot')
    doc.appendChild(root)
    root.setAttribute("xmlns:xacro", "http://www.ros.org/wiki/xacro")
    for i in robot.elements:
        try:
            if element == 'Transmission':
                if i.name.find(element) != -1:
                    root.appendChild(i.to_xml(doc))
            elif element == 'Gazebo':
                if i.reference is not None:
                    root.appendChild(i.to_xml(doc))
                elif i.plugins != []:
                    root.appendChild(i.to_xml(doc))
            elif element == 'material':
                if type(i) == naoqi_tools.urdf.Material:
                    root.appendChild(i.to_xml(doc))
        except AttributeError:
            pass
    filename = OUTPUT[0:OUTPUT.rfind('.')] + '_' + str(element) + '.xacro'
    print('exporting ' + element + ' xacro')
    write_comments_in_xacro(doc, filename)


def export_robot_to_xacro_files():
    """
    Export the entire 'robot' in several xacro files.

    One xacro file per kinematic chain (<ROBOT>_legs.xacro,
    <ROBOT>_arms.xacro, <ROBOT>_torso.xacro...)
    Xacro file for specific parts of the robot (<ROBOT>_fingers.xacro,
    <ROBOT>_sensors.xacro)
    One xacro file for visual elements (<ROBOT>_visual_collision.xacro,
    <ROBOT>_material.xacro)
    One xacro file per type of element needed for gazebo simulation
    (<ROBOT>_Gazebo.xacro, <ROBOT>_Transmission.xacro)
    One generic robot file which includes all the other ones
    (<ROBOT>_robot.xacro)
    """
    global robot, OUTPUT, NAME
    doc = Document()
    root = doc.createElement('robot')
    doc.appendChild(root)
    root.setAttribute("xmlns:xacro", "http://www.ros.org/wiki/xacro")
    root.setAttribute("name", robot.name)
    root.appendChild(ur.short(doc, 'xacro:include', 'filename', NAME +
        '_visual_collisions.xacro'))
    create_visual_xacro()
    for i in XACRO_DICO.keys():
        print('exporting ' + NAME + '_' + i + '.xacro')
        if i.find('eye') != -1:
            export_kinematic_chain_to_xacro(i, 'HeadRoll_link',
                'HeadRoll_link')
        else:
            export_kinematic_chain_to_xacro(i)
        filenamerobot = NAME + '_' + i + '.xacro'
        root.appendChild(ur.short(doc, 'xacro:include', 'filename',
            filenamerobot))
    # Transmission elements not available from Aldebaran libraries yet
    export_robot_element('Transmission')
    root.appendChild(ur.short(doc, 'xacro:include', 'filename', NAME +
        '_Transmission.xacro'))
    # Gazebo Plugin not available from Aldebaran libraries yet
    export_robot_element('Gazebo')
    root.appendChild(ur.short(doc, 'xacro:include', 'filename', NAME +
        '_Gazebo.xacro'))
    root.appendChild(ur.short(doc, 'xacro:include', 'filename', NAME +
        '_sensors.xacro'))
    export_list_to_xacro(['_frame'], OUTPUT[0:OUTPUT.rfind('.')] +
        '_sensors.xacro')
    root.appendChild(ur.short(doc, 'xacro:include', 'filename', NAME +
        '_fingers.xacro'))
    export_list_to_xacro(['Finger', 'Thumb'], OUTPUT[0:OUTPUT.rfind('.')] +
        '_fingers.xacro')
    if NAME == 'pepper':
        root.appendChild(ur.short(doc, 'xacro:include', 'filename', NAME +
            '_wheels.xacro'))
        export_list_to_xacro(['Wheel'], OUTPUT[0:OUTPUT.rfind('.')] +
            '_wheels.xacro')
    if NAME == 'romeo':
        root.appendChild(ur.short(doc, 'xacro:include', 'filename',
            'romeo_cap.xacro'))

    filename = OUTPUT[0:OUTPUT.rfind('.')] + '_robot.xacro'
    write_comments_in_xacro(doc, filename)
    print('output directory : ' + OUTPUT[0:OUTPUT.rfind('/') + 1])


def export_kinematic_chain_to_xacro(keyword, baseChain='base_link',
                                    tipRefChain='default'):
    """Export a specific kinematic chain to a xacro file.

    :param : keyword, string defining kinematic chains to export
    (legs,arms,head,torso)
    :param : baseChain, string representing the name of the link where
    the reference chain starts
    :param : tipRefChain, string representing the name of the link where
    the reference chain ends
    """
    global robot, OUTPUT
    if tipRefChain == 'default':
        print('applying torso to end of ref chain')
        tipRefChain = XACRO_DICO['torso']
    chainRef = robot.get_chain(baseChain, tipRefChain)
    print(chainRef)
    doc = Document()
    root = doc.createElement('robot')
    doc.appendChild(root)
    root.setAttribute('xmlns:xacro', 'http://www.ros.org/wiki/xacro')
    chainNb = 0
    try:
        chain1 = robot.get_chain(baseChain, 'l_' + XACRO_DICO[keyword])
        chain2 = robot.get_chain(baseChain, 'r_' + XACRO_DICO[keyword])
        chainNb = 2
    except KeyError:
        try:
            chain1 = robot.get_chain(baseChain, 'L' + XACRO_DICO[keyword])
            chain2 = robot.get_chain(baseChain, 'R' + XACRO_DICO[keyword])
            chainNb = 2
        except KeyError:
            try:
                chain1 = robot.get_chain(baseChain, XACRO_DICO[keyword])
                chainNb = 1
            except KeyError:
                print('the chain ' + keyword + ' cannot be found')

    if chainNb != 0:
        duplicate = 0
        for i in range(len(chain1)):
            for j in range(len(chainRef)):
                if chain1[i] == chainRef[j]:
                    duplicate = 1
            if duplicate == 0 or keyword == 'torso':
                try:
                    root.appendChild(robot.links[chain1[i]].to_xml(doc))
                except KeyError:
                    try:
                        root.appendChild(robot.joints[chain1[i]].to_xml(doc))
                    except KeyError:
                        print('unknown element' + chain1[i])
            else:
                duplicate = 0
        if chainNb == 2:
            for i in range(len(chain2)):
                for j in range(len(chainRef)):
                    if chain2[i] == chainRef[j]:
                        duplicate = 1
                if duplicate == 0:
                    try:
                        root.appendChild(robot.links[chain2[i]].to_xml(doc))
                    except KeyError:
                        try:
                            root.appendChild(
                                robot.joints[chain2[i]].to_xml(doc))
                        except KeyError:
                            print('unknown element' + chain2[i])
                else:
                    duplicate = 0
        filename = OUTPUT[0:OUTPUT.rfind('.')] + '_' + keyword + str('.xacro')
        write_comments_in_xacro(doc, filename)


def write_comments_in_xacro(doc, filename):
    """
    Write the content of the XML Document doc to a file named filename.

    Also add comments at the beginning of the file

    :param : doc, minidom Document to write
    :param : filename, absolute path of the file to write to
    """
    if(not os.path.isdir(filename[0:filename.rfind('/') + 1])):
        os.makedirs(filename[0:filename.rfind('/')])

    file = open(filename, 'w+')
    file.write(doc.toprettyxml())
    file.close()
    file = open(filename, 'r')
    firstline, remaining = file.readline(), file.read()
    file.close()
    file = open(filename, 'w')
    file.write(firstline)
    file.write(
        '<!--**************************************************************\n'
        ' **** File automatically generated by generate_urdf.py script ****\n'
        ' **************************************************************-->\n')
    file.write(remaining)
    file.close()


def export_list_to_xacro(list, filename):
    """Export all links containing a string and its parent joint.

    :param : list, list of strings to look for
    :param : filename, absolute path of the file to write to
    """
    global robot, OUTPUT
    doc = Document()
    root = doc.createElement('robot')
    doc.appendChild(root)
    root.setAttribute("xmlns:xacro", "http://www.ros.org/wiki/xacro")
    print ('exporting ' + os.path.basename(filename))
    for string in list:
        for link in robot.links:
            if robot.links[link].name.find(string) != -1:
                root.appendChild(robot.links[link].to_xml(doc))
                for joint in robot.joints:
                    if robot.joints[joint].child == robot.links[link].name:
                        root.appendChild(robot.joints[joint].to_xml(doc))
    write_comments_in_xacro(doc, filename)


def adjustMeshPath(path_mesh_pkg, link):
    """
    Find the path of a mesh according to the link definition.

    Set the visual and collision element of the given link

    :param : path_mesh_pkg, absolute path of the package where the meshes
    should be located
    :param : link, dictionary key of the link we want to set
    visual and collision parameters

    :return : tempVisu, Visual element with the NAO visual geometrical shape
    for people who don't have the meshes
    :return : tempCol, Collision element with the NAO collision geometrical
    shape for people who don't have the meshes
    """
    global robot, SCALE, MESHPKG, MESH_VERSION
    tempVisu = None
    tempVisuMesh = None
    tempCol = None
    tempColMesh = None
    if robot.links[link].visual is not None:
        try:
            meshname = str(
                LINKS_DICO.keys()[list(LINKS_DICO.values()).index(link)])
            if meshname.endswith('_link'):
                meshfile = meshname[0:-5]
            else:
                meshfile = meshname
        except:
            meshname = link
            meshfile = link
            pass
        if meshfile.endswith('_link'):
            meshfile = meshfile[0:-5]
        tempVisuMesh = ur.Visual(ur.Mesh('', (SCALE, SCALE, SCALE)))
        tempColMesh = ur.Collision(ur.Mesh('', (SCALE, SCALE, SCALE)))
        tempVisuMesh.origin = robot.links[link].visual.origin
        tempColMesh.origin = robot.links[link].visual.origin
        if os.path.isfile(os.path.join(path_mesh_pkg, 'meshes', MESH_VERSION,
            robot.links[link].visual.geometry.filename[
                robot.links[link].visual.geometry.filename.rfind('/') + 1:])):
            tempVisuMesh.geometry.filename = os.path.join(
                'package://', NAME + MESHPKG, 'meshes', MESH_VERSION,
                robot.links[link].visual.geometry.filename[
                    robot.links[link].visual.geometry.filename.rfind('/')+1:])
            tempColMesh.geometry.filename = \
                tempVisuMesh.geometry.filename[0:-4] + COLLISION_SUFFIX
        else:
            tempVisuMesh.geometry.filename = os.path.join(
                'package://', NAME + MESHPKG, 'meshes', MESH_VERSION,
                meshfile + '.dae')
            tempColMesh.geometry.filename = \
                tempVisuMesh.geometry.filename[0:-4] + COLLISION_SUFFIX

        if NAME == 'nao':
            try:
                tempVisu = ur.Visual(
                    VISU_DICO[meshname], ur.Material('LightGrey'),
                    dico.Nao_orig[meshname])
                tempCol = ur.Collision(VISU_DICO[meshname],
                    dico.Nao_orig[meshname])
            except KeyError:
                tempVisu = None
                tempCol = None
        robot.links[link].visual = tempVisuMesh
        robot.links[link].collision = tempColMesh
    return (tempVisu, tempCol)

##############
#### Main ####
##############

args = parser.parse_args()
if args.input is '':
    robot = URDF.from_parameter_server()
else:
    robot = URDF.load_xml_file(args.input)

if robot.name.find('V') != -1:
    VERSION = robot.name[robot.name.find('V'):]
else:
    VERSION = args.input[args.input.find('V'):args.input.find('V') + 3]

if robot.name.lower().find('nao') != -1:
    NAME = 'nao'
    try:
        import naoqi_tools.nao_dictionaries as dico
        print('import nao dictionaries')
    except:
        print('unable to import nao dictionaries')
        sys.exit(0)
    LINKS_DICO = dico.Nao_links
    VISU_DICO = dico.Nao_visu
    OFFSETS_DICO = dico.Nao_offsets
    XACRO_DICO = NAO_XACRO_DICO
    MESHPKG = '_meshes'
    SCALE = 0.1
elif robot.name.lower().find('romeo') != -1:
    NAME = 'romeo'
    try:
        import naoqi_tools.romeo_dictionaries as dico
    except:
        print('unable to import romeo dictionaries')
        sys.exit(0)
    LINKS_DICO = dico.Romeo_links
    OFFSETS_DICO = dico.Romeo_offsets
    VISU_DICO = ''
    XACRO_DICO = ROMEO_XACRO_DICO
    MESHPKG = '_description'
    SCALE = 1

elif robot.name.lower().find('juliette') or robot.name.lower().find('pepper'):
    NAME = 'pepper'
    try:
        import naoqi_tools.pepper_dictionaries as dico
    except:
        print('unable to import pepper dictionaries')
        sys.exit(0)
    LINKS_DICO = dico.Pepper_links
    OFFSETS_DICO = dico.Pepper_offsets
    VISU_DICO = ''
    XACRO_DICO = PEPPER_XACRO_DICO
    print('PROCESSING PEPPER ROBOT')
    MESHPKG = '_meshes'
    SCALE = 0.1
    VERSION = '1.0'

for element in robot.elements:
    if type(element) == naoqi_tools.urdf.Material:
        robot.elements.remove(element)
cmd = 'rospack find ' + NAME + '_description'
try:
    pathdescription = subprocess.check_output(cmd,
        stderr=subprocess.STDOUT, shell=True)[:-1]
except:
    print('unable to find ' + NAME + '_description package')
    sys.exit(0)
OUTPUT = os.path.join(pathdescription, 'urdf', NAME + VERSION +
    '_generated_urdf', NAME + '.urdf')
print('processing ' + NAME + ' (' + VERSION + ") robot's urdf file in " +
    OUTPUT)
cmd = 'rospack find ' + NAME + MESHPKG
try:
    path_mesh_pkg = subprocess.check_output(cmd, stderr=subprocess.STDOUT,
        shell=True)[:-1]
except:
    print('unable to find ' + NAME + MESHPKG + ' package')
    sys.exit(0)

define_materials()
if args.REP120 == 'true':
    REP120_compatibility()

for link in robot.links:
    adjustMeshPath(path_mesh_pkg, link)

if args.xacro == 'robot':
    export_robot_element('material')
    export_robot_to_xacro_files()
elif args.xacro == 'urdf':
    robot.write_xml(OUTPUT)
else:
    export_kinematic_chain_to_xacro(args.xacro)
