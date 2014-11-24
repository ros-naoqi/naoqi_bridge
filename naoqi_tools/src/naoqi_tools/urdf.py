# Based on the python URDF implementation by Antonio El Khoury
# Available at https://github.com/laas/robot_model_py

import string
from naoqi_tools.gazeboUrdf import *
from xml.dom.minidom import Document
from xml.dom import minidom
import sys
from numpy import array,pi
import re, copy

HUBO_JOINT_SUFFIX_MASK=r'([HKASEWF][RPY12345])'

class Collision(object):
    """Collision node stores collision geometry for a link."""
    def __init__(self, geometry=None, origin=None):
        self.geometry = geometry
        self.origin = origin

    @staticmethod
    def parse(node, verbose=True):
        c = Collision()
        for child in children(node):
            if child.localName == 'geometry':
                c.geometry = Geometry.parse(child, verbose)
            elif child.localName == 'origin':
                c.origin = Pose.parse(child)
            else:
                print("Unknown collision element '%s'"%child.localName)
        return c

    def to_xml(self, doc):
        xml = doc.createElement("collision")
        add(doc, xml, self.geometry)
        add(doc, xml, self.origin)
        return xml

    def to_openrave_xml(self, doc):

        xml = self.geometry.to_openrave_xml(doc)
        add_openrave(doc,xml, self.origin)
        return xml

    def __str__(self):
        s =  "Origin:\n{0}\n".format(reindent(str(self.origin), 1))
        s += "Geometry:\n{0}\n".format(reindent(str(self.geometry), 1))
        return s

class Color(object):
    """Color node stores color definition for a material or a visual."""
    def __init__(self, r=0.0, g=0.0, b=0.0, a=0.0):
        self.rgba=(r,g,b,a)

    @staticmethod
    def parse(node):
        rgba = node.getAttribute("rgba").split()
        (r,g,b,a) = [ float(x) for x in rgba ]
        return Color(r,g,b,a)

    def to_xml(self, doc):
        xml = doc.createElement("color")
        set_attribute(xml, "rgba", self.rgba)
        return xml

    def to_openrave_xml(self,doc):
        return None

    def __str__(self):
        return "r: {0}, g: {1}, b: {2}, a: {3},".format(
            self.rgba[0], self.rgba[1], self.rgba[2], self.rgba[3])

class Dynamics(object):
    """Dynamics node stores coefficients that define the dynamics of a joint"""
    def __init__(self, damping=None, friction=None):
        self.damping = damping
        self.friction = friction

    @staticmethod
    def parse(node):
        d = Dynamics()
        if node.hasAttribute('damping'):
            d.damping = node.getAttribute('damping')
        if node.hasAttribute('friction'):
            d.friction = node.getAttribute('friction')
        return d

    def to_xml(self, doc):
        xml = doc.createElement('dynamics')
        set_attribute(xml, 'damping', self.damping)
        set_attribute(xml, 'friction', self.friction)
        return xml

    def to_openrave_xml(self,doc):
        return None

    def __str__(self):
        return "Damping: {0}\nFriction: {1}\n".format(self.damping,
                                                      self.friction)

class Geometry(object):
    """Geometry abstract class define the type of geomerical shape for a visual or collision element"""
    def __init__(self):
        None

    @staticmethod
    def parse(node, verbose=True):
        shape = children(node)[0]
        if shape.localName=='box':
            return Box.parse(shape)
        elif shape.localName=='cylinder':
            return Cylinder.parse(shape)
        elif shape.localName=='sphere':
            return Sphere.parse(shape)
        elif shape.localName=='mesh':
            return Mesh.parse(shape)
        else:
            if verbose:
                print("Unknown shape %s"%shape.localName)

    def __str__(self):
        return "Geometry abstract class"

class Box(Geometry):
    """Box node stores the dimensions for a rectangular geometry element"""
    def __init__(self, dims=None):
        if dims is None:
            self.dims = None
        else:
            self.dims = (dims[0], dims[1], dims[2])

    @staticmethod
    def parse(node):
        dims = node.getAttribute('size').split()
        return Box([float(a) for a in dims])

    def to_xml(self, doc):
        xml = doc.createElement("box")
        set_attribute(xml, "size", self.dims)
        geom = doc.createElement('geometry')
        geom.appendChild(xml)
        return geom

    def to_openrave_xml(self, doc):
        xml = short(doc, "geometry","type","box")
        xml.appendChild(create_element(xml, "extents", self.dims))
        return xml

    def __str__(self):
        return "Dimension: {0}".format(self.dims)


class Cylinder(Geometry):
    """Cylinder node stores the dimensions for a z-rotation invariant geometry element"""
    def __init__(self, radius=0.0, length=0.0):
        self.radius = radius
        self.length = length

    @staticmethod
    def parse(node):
        r = node.getAttribute('radius')
        l = node.getAttribute('length')
        return Cylinder(float(r), float(l))

    def to_xml(self, doc):
        xml = doc.createElement("cylinder")
        set_attribute(xml, "radius", self.radius)
        set_attribute(xml, "length", self.length)
        geom = doc.createElement('geometry')
        geom.appendChild(xml)
        return geom

    def to_openrave_xml(self, doc):
        xml = short(doc, "geometry","type","cylinder")
        xml.appendChild(create_element(doc, "height", self.length))
        xml.appendChild(create_element(doc, "radius", self.radius))
        return xml

    def __str__(self):
        return "Radius: {0}\nLength: {1}".format(self.radius,
                                                 self.length)

class Sphere(Geometry):
    """Sphere node stores the dimensions for a rotation invariant geometry element"""
    def __init__(self, radius=0.0):
        self.radius = radius

    @staticmethod
    def parse(node):
        r = node.getAttribute('radius')
        return Sphere(float(r))

    def to_xml(self, doc):
        xml = doc.createElement("sphere")
        set_attribute(xml, "radius", self.radius)
        geom = doc.createElement('geometry')
        geom.appendChild(xml)
        return geom

    def to_openrave_xml(self, doc):
        xml = short(doc, "geometry","type","sphere")
        xml.create_child(xml, "radius", self.radius)
        return xml

    def __str__(self):
        return "Radius: {0}".format(self.radius)


class Mesh(Geometry):
    """Mesh node stores the path of the mesh file to be displayed"""
    def __init__(self, filename=None, scale=1):
        self.filename = filename
        self.scale = scale

    @staticmethod
    def parse(node):
        fn = node.getAttribute('filename')
        s = node.getAttribute('scale')
        if s == "":
            s = 1
        else:
            xyz = node.getAttribute('scale').split()
            s = map(float, xyz)
        return Mesh(fn, s)

    def to_xml(self, doc):
        xml = doc.createElement("mesh")
        set_attribute(xml, "filename", self.filename)
        if self.scale != 1:
            set_attribute(xml, "scale", self.scale)
        geom = doc.createElement('geometry')
        geom.appendChild(xml)
        return geom

    def to_openrave_xml(self, doc):
        xml = short(doc, "geometry","type","trimesh")
        f='../meshes/'+self.filename.split('/')[-1]
        fhull=re.sub(r"Body","convhull",f)
        #xml.appendChild(create_element(doc, "render", [f, self.scale]))
        xml.appendChild(create_element(doc, "data", [fhull, self.scale]))
        #set_attribute(xml, "render","true")
        return xml

    def __str__(self):
        return "Filename: {0}\nScale: {1}".format(self.filename, self.scale)


class Inertial(object):
    """Inertial node stores mecanical information of a Link : mass, inertia matrix and origin"""
    def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0,
                 mass=0.0, origin=None):
        self.matrix = {}
        self.matrix['ixx'] = ixx
        self.matrix['ixy'] = ixy
        self.matrix['ixz'] = ixz
        self.matrix['iyy'] = iyy
        self.matrix['iyz'] = iyz
        self.matrix['izz'] = izz
        self.mass = mass
        self.origin = origin

    @staticmethod
    def parse(node):
        inert = Inertial()
        for child in children(node):
            if child.localName=='inertia':
                for v in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']:
                    inert.matrix[v] = float(child.getAttribute(v))
            elif child.localName=='mass':
                inert.mass = float(child.getAttribute('value'))
            elif child.localName == 'origin':
                inert.origin = Pose.parse(child)
        return inert


    def to_xml(self, doc):
        xml = doc.createElement("inertial")

        xml.appendChild(short(doc, "mass", "value", self.mass))

        inertia = doc.createElement("inertia")
        for (n,v) in self.matrix.items():
            set_attribute(inertia, n, v)
        xml.appendChild(inertia)

        add(doc, xml, self.origin)
        return xml

    def to_openrave_xml(self, doc):
        xml = doc.createElement("mass")
        set_attribute(xml,"type","custom")
        xml.appendChild(create_element(doc, "total", self.mass))
        text='{ixx} {ixy} {ixz}\n{ixy} {iyy} {iyz}\n{ixz} {iyz} {izz}'.format(**self.matrix)
        xml.appendChild(create_element(doc,"inertia",text))
        add_openrave(doc, xml, self.origin)
        xml.getElementsByTagName('translation')[0].tagName="com"
        return xml

    def __str__(self):
        s =  "Origin:\n{0}\n".format(reindent(str(self.origin), 1))
        s += "Mass: {0}\n".format(self.mass)
        s += "ixx: {0}\n".format(self.matrix['ixx'])
        s += "ixy: {0}\n".format(self.matrix['ixy'])
        s += "ixz: {0}\n".format(self.matrix['ixz'])
        s += "iyy: {0}\n".format(self.matrix['iyy'])
        s += "iyz: {0}\n".format(self.matrix['iyz'])
        s += "izz: {0}\n".format(self.matrix['izz'])
        return s


class Joint(object):
    """Joint node stores articulation information coupling two links"""
    UNKNOWN = 'unknown'
    REVOLUTE = 'revolute'
    CONTINUOUS = 'continuous'
    PRISMATIC = 'prismatic'
    FLOATING = 'floating'
    PLANAR = 'planar'
    FIXED = 'fixed'


    def __init__(self, name, parent, child, joint_type, axis=None, origin=None,
                 limits=None, dynamics=None, safety=None, calibration=None,
                 mimic=None):
        self.name = name
        self.parent = parent
        self.child = child
        self.joint_type = joint_type
        self.axis = axis
        self.origin = origin
        self.limits = limits
        self.dynamics = dynamics
        self.safety = safety
        self.calibration = calibration
        self.mimic = mimic

    @staticmethod
    def parse(node, verbose=True):
        joint = Joint(node.getAttribute('name'), None, None,
                      node.getAttribute('type'))
        for child in children(node):
            if child.localName == 'parent':
                joint.parent = child.getAttribute('link')
            elif child.localName == 'child':
                joint.child = child.getAttribute('link')
            elif child.localName == 'axis':
                joint.axis = array([float(x) for x in child.getAttribute('xyz').split(' ')])
            elif child.localName == 'origin':
                joint.origin = Pose.parse(child)
            elif child.localName == 'limit':
                joint.limits = JointLimit.parse(child)
            elif child.localName == 'dynamics':
                joint.dynamics = Dynamics.parse(child)
            elif child.localName == 'safety_controller':
                joint.safety = SafetyController.parse(child)
            elif child.localName == 'calibration':
                joint.calibration = JointCalibration.parse(child)
            elif child.localName == 'mimic':
                joint.mimic = JointMimic.parse(child)
            else:
                if verbose:
                   print("Unknown joint element '%s'"%child.localName)
        return joint

    def to_xml(self, doc):
        xml = doc.createElement("joint")
        set_attribute(xml, "name", self.name)
        set_attribute(xml, "type", self.joint_type)
        xml.appendChild( short(doc, "parent", "link", self.parent) )
        xml.appendChild( short(doc, "child" , "link", self.child ) )
        add(doc, xml, self.origin)
        if self.axis is not None:
            xml.appendChild( short(doc, "axis", "xyz", to_string(self.axis) ) )
        add(doc, xml, self.limits)
        add(doc, xml, self.dynamics)
        add(doc, xml, self.safety)
        add(doc, xml, self.calibration)
        add(doc, xml, self.mimic)

        return xml

    def to_openrave_xml(self, doc):
        xml = doc.createElement("joint")
        set_attribute(xml, "name", self.name)
        s=""
        if self.joint_type == Joint.UNKNOWN:
            s = "unknown"
        elif self.joint_type == Joint.REVOLUTE:
            s = "hinge"
        elif self.joint_type == Joint.CONTINUOUS:
            s = "hinge"
            set_attribute(xml, "circular", "true")

        elif self.joint_type == Joint.PRISMATIC:
            s = "slider"
        elif self.joint_type == Joint.FIXED:
            s = "hinge"
            set_attribute(xml, "enable", "false")
            xml.appendChild( create_element(doc, "limits", "0 0") )

        set_attribute(xml, "type", s)
        if self.mimic is not None:
            multiplier=self.mimic.multiplier if self.mimic.multiplier is not None else 1.0
            offset=self.mimic.offset if self.mimic.offset is not None else 0.0
            #1) Follow openrave mimic joint format, disable joint:
            set_attribute(xml,"enable","false")
            #2) Create the position equation
            set_attribute(xml,"mimic_pos","{0} * {1} + {2}".format(self.mimic.joint_name,multiplier,offset))
            set_attribute(xml,"mimic_vel","|{0} {1}".format(self.mimic.joint_name,multiplier))

        xml.appendChild( create_element(doc, "body", self.parent) )
        xml.appendChild( create_element(doc, "body" , self.child ) )
        xml.appendChild( create_element(doc, "offsetfrom" , self.parent) )
        add_openrave(doc, xml, self.origin)
        xml.getElementsByTagName('translation')[0].tagName="anchor"
        if self.axis is not None:
            xml.appendChild( create_element(doc, "axis", self.axis) )
        add_openrave(doc, xml, self.limits)
        return xml


    def __str__(self):
        s = "Name: {0}\n".format(self.name)

        s += "Child link name: {0}\n".format(self.child)
        s += "Parent link name: {0}\n".format(self.parent)

        if self.joint_type == Joint.UNKNOWN:
            s += "Type: unknown\n"
        elif self.joint_type == Joint.REVOLUTE:
            s += "Type: revolute\n"
        elif self.joint_type == Joint.CONTINUOUS:
            s += "Type: continuous\n"
        elif self.joint_type == Joint.PRISMATIC:
            s += "Type: prismatic\n"
        elif self.joint_type == Joint.FLOATING:
            s += "Type: floating\n"
        elif self.joint_type == Joint.PLANAR:
            s += "Type: planar\n"
        elif self.joint_type == Joint.FIXED:
            s += "Type: fixed\n"
        else:
            print("unknown joint type")

        s +=  "Axis: {0}\n".format(self.axis)
        s +=  "Origin:\n{0}\n".format(reindent(str(self.origin), 1))
        s += "Limits:\n"
        s += reindent(str(self.limits), 1) + "\n"
        s += "Dynamics:\n"
        s += reindent(str(self.dynamics), 1) + "\n"
        s += "Safety:\n"
        s += reindent(str(self.safety), 1) + "\n"
        s += "Calibration:\n"
        s += reindent(str(self.calibration), 1) + "\n"
        s += "Mimic:\n"
        s += reindent(str(self.mimic), 1) + "\n"
        return s


#FIXME: we are missing the reference position here.
class JointCalibration(object):
    def __init__(self, rising=None, falling=None):
        self.rising = rising
        self.falling = falling

    @staticmethod
    def parse(node):
        jc = JointCalibration()
        if node.hasAttribute('rising'):
            jc.rising = float( node.getAttribute('rising') )
        if node.hasAttribute('falling'):
            jc.falling = float( node.getAttribute('falling') )
        return jc

    def to_xml(self, doc):
        xml = doc.createElement('calibration')
        set_attribute(xml, 'rising', self.rising)
        set_attribute(xml, 'falling', self.falling)
        return xml

    def to_openrave_xml(self,doc):
        #No implementation
        return None

    def __str__(self):
        s = "Raising: {0}\n".format(self.rising)
        s += "Falling: {0}\n".format(self.falling)
        return s

class JointLimit(object):
    """JointLimit node stores mechanical limits of a given joint"""
    def __init__(self, effort, velocity, lower=None, upper=None):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper

    @staticmethod
    def parse(node):
        jl = JointLimit( float( node.getAttribute('effort') ) ,
                         float( node.getAttribute('velocity')))
        if node.hasAttribute('lower'):
            jl.lower = float( node.getAttribute('lower') )
        if node.hasAttribute('upper'):
            jl.upper = float( node.getAttribute('upper') )
        return jl

    def get_from_table(self,node):
        jl = JointLimit( float( node.getAttribute('effort') ) ,
                         float( node.getAttribute('velocity')))
        if node.hasAttribute('lower'):
            jl.lower = float( node.getAttribute('lower') )
        if node.hasAttribute('upper'):
            jl.upper = float( node.getAttribute('upper') )
        return jl

    def to_xml(self, doc):
        xml = doc.createElement('limit')
        set_attribute(xml, 'effort', self.effort)
        set_attribute(xml, 'velocity', self.velocity)
        set_attribute(xml, 'lower', self.lower)
        set_attribute(xml, 'upper', self.upper)
        return xml

    def to_openrave_xml(self,doc):
        limit = create_element(doc,'limitsdeg',[round(self.lower*180/pi,1),round(self.upper*180/pi,1)])
        maxvel = create_element(doc,'maxvel',self.velocity)
        maxtrq = create_element(doc,'maxtorque',self.effort)
        return [limit,maxvel,maxtrq]

    def __str__(self):
        s = "Effort: {0}\n".format(self.effort)
        s += "Lower: {0}\n".format(self.lower)
        s += "Upper: {0}\n".format(self.upper)
        s += "Velocity: {0}\n".format(self.velocity)
        return s

class JointMimic(object):
    """JointMimic node stores information coupling an actuated joint to a not controllable joint"""
    def __init__(self, joint_name, multiplier=None, offset=None):
        self.joint_name = joint_name
        self.multiplier = multiplier
        self.offset = offset

    @staticmethod
    def parse(node):
        mimic = JointMimic( node.getAttribute('joint') )
        if node.hasAttribute('multiplier'):
            mimic.multiplier = float( node.getAttribute('multiplier') )
        if node.hasAttribute('offset'):
            mimic.offset = float( node.getAttribute('offset') )
        return mimic

    def to_xml(self, doc):
        xml = doc.createElement('mimic')
        set_attribute(xml, 'joint', self.joint_name)
        set_attribute(xml, 'multiplier', self.multiplier)
        set_attribute(xml, 'offset', self.offset)
        return xml

    def __str__(self):
        s = "Joint: {0}\n".format(self.joint_name)
        s += "Multiplier: {0}\n".format(self.multiplier)
        s += "Offset: {0}\n".format(self.offset)
        return s


class Link(object):
    """Link node stores information defining the mechanics and the visualization of a link"""
    def __init__(self, name, visual=None, inertial=None, collision=None, xacro=None):
        self.name = name
        self.visual = visual
        self.inertial = inertial
        self.collision = collision
        self.xacro = xacro

    @staticmethod
    def parse(node, verbose=True):
        link = Link(node.getAttribute('name'))
        for child in children(node):
            if child.localName == 'visual':
                link.visual = Visual.parse(child, verbose)
            elif child.localName == 'collision':
                link.collision = Collision.parse(child, verbose)
            elif child.localName == 'inertial':
                link.inertial = Inertial.parse(child)
            elif child.localName.startswith('xacro'):
                link.xacro = xacro
            else:
                if verbose:
                    print("Unknown link element '%s'"%child.localName)
        return link

    def to_xml(self, doc):
        xml = doc.createElement("link")
        xml.setAttribute("name", self.name)
        add( doc, xml, self.visual)
        add( doc, xml, self.collision)
        add( doc, xml, self.inertial)
        if self.xacro is not None:
            text = doc.createElement(self.xacro)
            xml.appendChild(text)
        return xml

    def to_openrave_xml(self, doc):
        xml = doc.createElement("body")
        xml.setAttribute("name", self.name)
        add_openrave( doc, xml, self.collision)
        add_openrave( doc, xml, self.inertial)
        return xml

    def __str__(self):
        s = "Name: {0}\n".format(self.name)
        s += "Inertial:\n"
        s += reindent(str(self.inertial), 1) + "\n"
        s += "Visual:\n"
        s += reindent(str(self.visual), 1) + "\n"
        s += "Collision:\n"
        s += reindent(str(self.collision), 1) + "\n"
        s += "Xacro:\n"
        s += reindent(str(self.xacro),1) + "\n"
        return s

class Material(object):
    """Material node stores visual information of a mechanical material : color, texture"""
    def __init__(self, name=None, color=None, texture=None):
        self.name = name
        self.color = color
        self.texture = texture

    @staticmethod
    def parse(node, verbose=True):
        material = Material()
        if node.hasAttribute('name'):
            material.name = node.getAttribute('name')
        for child in children(node):
            if child.localName == 'color':
                material.color = Color.parse(child)
            elif child.localName == 'texture':
                material.texture = child.getAttribute('filename')
            else:
                if verbose:
                    print("Unknown material element '%s'"%child.localName)

        return material

    def to_xml(self, doc):
        xml = doc.createElement("material")
        set_attribute(xml, "name", self.name)
        add( doc, xml, self.color )

        if self.texture is not None:
            text = doc.createElement("texture")
            text.setAttribute('filename', self.texture)
            xml.appendChild(text)
        return xml

    def __str__(self):
        s = "Name: {0}\n".format(self.name)
        s += "Color: {0}\n".format(self.color)
        s += "Texture:\n"
        s += reindent(str(self.texture), 1)
        return s


class Pose(object):
    """Pose node stores a cartesian 6-D pose :
    xyz : array
    rpy : array
    """
    def __init__(self, position=None, rotation=None):
        self.position = array(position)
        self.rotation = array(rotation)

    @staticmethod
    def parse(node):
        pose = Pose()
        if node.hasAttribute("xyz"):
            xyz = node.getAttribute('xyz').split()
            pose.position = array(map(float, xyz))
        if node.hasAttribute("rpy"):
            rpy = node.getAttribute('rpy').split()
            pose.rotation = array(map(float, rpy))
        return pose

    def to_xml(self, doc):
        xml = doc.createElement("origin")
        set_attribute(xml, 'xyz', self.position)
        set_attribute(xml, 'rpy', self.rotation)
        return xml

    def to_openrave_xml(self, doc):
        xml = doc.createElement("translation")
        set_content(doc,xml,self.position)
        elements=[xml]
        if not self.rotation[0] == 0:
            rotr = doc.createElement("rotationaxis")
            set_content(doc,rotr,[1,0,0,self.rotation[0]*180.0/pi])
            elements.append(rotr)
        if not self.rotation[1] == 0:
            rotp = doc.createElement("rotationaxis")
            set_content(doc,rotp,[0,1,0,self.rotation[1]*180.0/pi])
            elements.append(rotp)
        if not self.rotation[2] == 0:
            roty = doc.createElement("rotationaxis")
            set_content(doc,roty,[0,0,1,self.rotation[2]*180.0/pi])
            elements.append(roty)

        return elements

    def __str__(self):
        return "Position: {0}\nRotation: {1}".format(self.position,
                                                     self.rotation)


class SafetyController(object):
    def __init__(self, velocity, position=None, lower=None, upper=None):
        self.velocity = velocity
        self.position = position
        self.lower = lower
        self.upper = upper

    @staticmethod
    def parse(node):
        sc = SafetyController( float(node.getAttribute('k_velocity')) )
        if node.hasAttribute('soft_lower_limit'):
            sc.lower = float( node.getAttribute('soft_lower_limit') )
        if node.hasAttribute('soft_upper_limit'):
            sc.upper = float( node.getAttribute('soft_upper_limit') )
        if node.hasAttribute('k_position'):
            sc.position = float( node.getAttribute('k_position') )
        return sc

    def to_xml(self, doc):
        xml = doc.createElement('safety_controller')
        set_attribute(xml, 'k_velocity', self.velocity)
        set_attribute(xml, 'k_position', self.position)
        set_attribute(xml, 'soft_upper_limit', self.upper)
        set_attribute(xml, 'soft_lower_limit', self.lower)
        return xml

    def __str__(self):
        s = "Safe lower limit: {0}\n".format(self.lower)
        s += "Safe upper limit: {0}\n".format(self.upper)
        s += "K position: {0}\n".format(self.position)
        s += "K velocity: {0}\n".format(self.velocity)
        return s


class Visual(object):
    """Visual node stores information defining shape and material of the link to be displayed"""
    def __init__(self, geometry=None, material=None, origin=None):
        self.geometry = geometry
        self.material = material
        self.origin = origin

    @staticmethod
    def parse(node, verbose=True):
        v = Visual()
        for child in children(node):
            if child.localName == 'geometry':
                v.geometry = Geometry.parse(child, verbose)
            elif child.localName == 'origin':
                v.origin = Pose.parse(child)
            elif child.localName == 'material':
                v.material = Material.parse(child, verbose)
            else:
                if verbose:
                    print("Unknown visual element '%s'"%child.localName)
        return v

    def to_xml(self, doc):
        xml = doc.createElement("visual")
        add( doc, xml, self.geometry )
        add( doc, xml, self.origin )
        add( doc, xml, self.material )
        return xml

    def __str__(self):
        s =  "Origin:\n{0}\n".format(reindent(str(self.origin), 1))
        s += "Geometry:\n"
        s += reindent(str(self.geometry), 1) + "\n"
        s += "Material:\n"
        s += reindent(str(self.material), 1) + "\n"
        return s


class Actuator(object):
    """Actuator node stores information about how a motor controls a joint : used in transmission tags"""
    def __init__(self,name=None,hardwareInterface=None,mechanicalReduction=1):
        self.name = name
        self.hardwareInterface = hardwareInterface
        self.mechanicalReduction = mechanicalReduction

    @staticmethod
    def parse(node, verbose=True):
        actuator = Actuator()
        if node.hasAttribute('name'):
            actuator.name = node.getAttribute('name')
        for child in children(node):
            if child.localName == 'hardwareInterface':
                actuator.hardwareInterface = str(child.childNodes[0].nodeValue)
            if child.localName == 'mechanicalReduction':
                actuator.mechanicalReduction = str(child.childNodes[0].nodeValue)
            else:
                print'Unknown actuator element ' + str(child.localName)
        return actuator

    def to_xml(self, doc):
        xml = doc.createElement('actuator')
        set_attribute(xml, 'name', self.name)
        xml.appendChild(create_element(doc, 'hardwareInterface', self.hardwareInterface))
        xml.appendChild(create_element(doc,"mechanicalReduction",str(self.mechanicalReduction)))
        return xml

    def __str__(self):
        s = "Name: {0}\n".format(self.name)
        s += "HardwareInterface : {0}\n".format(self.hardwareInterface)
        s += "Mechanical Reduction: {0}\n".format(self.mechanicalReduction)
        return s


class Transmission(object):
    """Transmission node stores information linking a joint to an actuator"""
    def __init__(self,name=None,type=None,joint=None,actuator=None):
        self.name = name
        self.joint = joint
        self.actuator = actuator
        self.type = type
    @staticmethod
    def parse(node, verbose=True):
        trans = Transmission()
        if node.hasAttribute('name'):
            trans.name = node.getAttribute('name')
        for child in children(node):
            if child.localName == 'joint':
                trans.joint = child.getAttribute('name')
            if child.localName == 'actuator':
                trans.actuator = Actuator.parse(child,verbose)
            if child.localName == 'type':
                trans.type = str(child.childNodes[0].nodeValue)#str(child.getAttribute('type'))
        return trans

    def to_xml(self, doc):
        xml = doc.createElement('transmission')
        set_attribute(xml, 'name', self.name)
        xml.appendChild( short(doc, "joint", "name", self.joint) )
        xml.appendChild(create_element(doc, 'type', self.type))
        add(doc, xml, self.actuator)

        return xml

    def __str__(self):
        s = "Name: {0}\n".format(self.name)
        s += "Type: {0}\n".format(self.type)
        s += "Joint: {0}\n".format(self.joint)
        s += "Actuator:\n"
        s += reindent(str(self.actuator), 1) + "\n"

        return s

########################################
######### URDF Global Class ############
########################################
class URDF(object):
    """URDF node stores every information about a robot's model"""
    ZERO_THRESHOLD=0.000000001
    def __init__(self, name=""):
        self.name = name
        self.elements = []
        self.gazebos = {}
        self.links = {}
        self.joints = {}
        self.materials = {}
        self.parent_map = {}
        self.child_map = {}

    @staticmethod
    def parse_xml_string(xml_string, verbose=True):
        """Parse a string to create a URDF robot structure."""
        urdf = URDF()
        base = minidom.parseString(xml_string)
        robot = children(base)[0]
        urdf.name = robot.getAttribute('name')

        for node in children(robot):
            if node.nodeType is node.TEXT_NODE:
                continue
            if node.localName == 'joint':
                urdf.add_joint( Joint.parse(node, verbose) )
            elif node.localName == 'link':
                urdf.add_link( Link.parse(node, verbose) )
            elif node.localName == 'material':
                urdf.add_material(Material.parse(node,verbose))
            elif node.localName == 'gazebo':
                urdf.add_gazebo(Gazebo.parse(node,verbose))
            elif node.localName == 'transmission':
                urdf.elements.append( Transmission.parse(node,verbose) )
            else:
                if verbose:
                    print("Unknown robot element '%s'"%node.localName)
        return urdf

    @staticmethod
    def load_xml_file(filename, verbose=True):
        """Parse a file to create a URDF robot structure."""
        f = open(filename, 'r')
        return URDF.parse_xml_string(f.read(), verbose)

    @staticmethod
    def load_from_parameter_server(key = 'robot_description', verbose=True):
        """
        Retrieve the robot model on the parameter server
        and parse it to create a URDF robot structure.

        Warning: this requires roscore to be running.
        """
        import rospy
        return URDF.parse_xml_string(rospy.get_param(key), verbose)

    def add_link(self, link):
        self.elements.append(link)
        self.links[link.name] = link

    def add_joint(self, joint):
        self.elements.append(joint)
        self.joints[joint.name] = joint

        self.parent_map[ joint.child ] = (joint.name, joint.parent)
        if joint.parent in self.child_map:
            self.child_map[joint.parent].append( (joint.name, joint.child) )
        else:
            self.child_map[joint.parent] = [ (joint.name, joint.child) ]

    def add_material(self, material):
        """
        Add a material to the URDF.elements list
        """
        self.elements.append(material)
        self.materials[material.name] = material

    def add_gazebo(self, gazebo):
        """
        Add gazebo data to the URDF.elements list
        """

        self.elements.append(gazebo)
        self.gazebos[gazebo.reference] = gazebo

    def get_chain(self, root, tip, joints=True, links=True, fixed=True):
        """Based on given link names root and tip, return either a joint or link chain as a list of names."""
        chain = []
        if links:
            chain.append(tip)
        link = tip
        while link != root:
            (joint, parent) = self.parent_map[link]
            if joints:
                if fixed or self.joints[joint].joint_type != 'fixed':
                    chain.append(joint)
            if links:
                chain.append(parent)
            link = parent
        chain.reverse()
        return chain

    def get_root(self):
        root = None
        for link in self.links:
            if link not in self.parent_map:
                assert root is None, "Multiple roots detected, invalid URDF."
                root = link
        assert root is not None, "No roots detected, invalid URDF."
        return root

    def to_xml(self,orderbytree=False,orderbytype=False):
        doc = Document()
        root = doc.createElement("robot")
        doc.appendChild(root)
        root.setAttribute("name", self.name)

        baselink=self.parent_map
        if orderbytree:
            #Walk child map sequentially and export
            pass
        if orderbytype:
            pass

        for element in self.elements:
            root.appendChild(element.to_xml(doc))

        return doc.toprettyxml()

    def write_xml(self,outfile=None):
        if outfile is None:
            outfile=self.name+'.urdf'

        self.write_reformatted(self.to_xml(),outfile)

    def make_openrave_kinbody(self,doc):
        kinbody = doc.createElement("kinbody")
        doc.appendChild(kinbody)
        kinbody.setAttribute("name", self.name)

        for element in self.elements:
            kinbody.appendChild(element.to_openrave_xml(doc))

        #Post-processing to add offsetfrom statements

        for j in self.joints.keys():
            for l in kinbody.getElementsByTagName('body'):
                if l.getAttribute('name')== self.joints[j].child:
                    #Add offsetfrom declarration and joint anchor as transform
                    l.appendChild( create_element(doc, "offsetfrom" , self.joints[j].parent) )
                    add_openrave(doc,l,self.joints[j].origin)
                    break

        #Add adjacencies
        for j in self.joints.values():
            kinbody.appendChild(create_element(doc,"adjacent",[j.parent, j.child]))

        #Add known extra adjacencies
        badjoints={'LAR':'LKP','LWR':'LWY','LSY':'LSP','LHY':'LHP',
                   'RAR':'RKP','RWR':'RWY','RSY':'RSP','RHY':'RHP',
                   'NK1':'Torso'}
        for k,v in badjoints.items():
            #TODO: search all bodies for above pairs and add extra adjacency tags
            b1=None
            b2=None
            for b in kinbody.getElementsByTagName('body'):
                if re.search(k,b.getAttribute('name')):
                    b1=b.getAttribute('name')
                if re.search(v,b.getAttribute('name')):
                    b2=b.getAttribute('name')
            if b1 and b2:
                kinbody.appendChild(create_element(doc,"adjacent",[b1, b2]))

        return kinbody

    def to_openrave_xml(self):
        doc = Document()
        root = doc.createElement("robot")
        doc.appendChild(root)
        root.setAttribute("name", self.name)
        root.appendChild(self.make_openrave_kinbody(doc))

        return doc.toprettyxml()

    def write_reformatted(self,data,name):
        if name is None:
            name=self.name
        outdata=re.sub(r'\t','    ',data)
        with open(name,'w+') as f:
            f.write(outdata)

    def write_openrave_files(self,outname=None,writerobot=False):
        if outname is None:
            outname=self.name

        kinfile=outname+'.kinbody.xml'

        if writerobot:
            robotfile=outname+'.robot.xml'
            doc = Document()
            root = doc.createElement("robot")
            doc.appendChild(root)
            root.setAttribute("name", outname)
            kinbody = doc.createElement("kinbody")
            root.appendChild(kinbody)
            kinbody.setAttribute("name", outname)
            kinbody.setAttribute("file", kinfile)
            self.write_reformatted(doc.toprettyxml(),robotfile)

        doc2 = Document()
        doc2.appendChild(self.make_openrave_kinbody(doc2))

        self.write_reformatted(doc2.toprettyxml(),kinfile)


    def __str__(self):
        s = "Name: {0}\n".format(self.name)
        s += "\n"
        s += "Links:\n"
        if len(self.links) == 0:
            s += "None\n"
        else:
            for k, v in self.links.iteritems():
                s += "- Link '{0}':\n{1}\n".format(k, reindent(str(v), 1))
        s += "\n"
        s += "Joints:\n"
        if len(self.joints) == 0:
            s += "None\n"
        else:
            for k, v in self.joints.iteritems():
                s += "- Joint '{0}':\n{1}\n".format(k, reindent(str(v), 1))
        s += "\n"
        s += "Materials:\n"
        if len(self.materials) == 0:
            s += "None\n"
        else:
            for k, v in self.materials.iteritems():
                s += "- Material '{0}':\n{1}\n".format(k, reindent(str(v), 1))

        return s

    def walk_chain(self,link,branchorder=None):
        """Walk along the first branch of urdf tree to find last link. Optionally specify which fork to take globally)."""
        child=link
        if branchorder is None:
            branchorder=0

        while self.child_map.has_key(child):
            children=self.child_map[link]
            l=len(children)
            child=children[min(branchorder,l-1)][1]

        return child

    def rename_link(self,link,newlink):
        """Rename a link, updating all internal references to the name, and
        removing the old name from the links dict."""
        self.links[link].name=newlink
        self.links[newlink]=self.links[link]
        self.links.pop(link)
        for k,v in self.parent_map.items():
            if k==link:
                self.parent_map[newlink]=v
                self.parent_map.pop(k)
                k=newlink
            if v[0]==link:
                new0=newlink
                v=(new0,v[1])
            if v[1]==link:
                new1=newlink
                v=(v[0],new1)
            self.parent_map[k]=v
        for k,v in self.child_map.items():
            if k==link:
                self.child_map[newlink]=v
                self.child_map.pop(k)
                k=newlink
            vnew=[]
            for el in v:
                if el[1]==link:
                    el=(el[0],newlink)
                vnew.append(el)
          #  print vnew
            self.child_map[k]=vnew
        for n,j in self.joints.items():
            if j.parent==link:
                j.parent=newlink
            if j.child==link:
                j.child=newlink
        #print self.child_map

    def rename_joint(self,joint,newjoint):
        """Find a joint and rename it to newjoint, updating all internal
        structures and mimic joints with the new name. Removes the old joint
        reference from the joints list."""
        self.joints[joint].name=newjoint
        self.joints[newjoint]=self.joints[joint]
        self.joints.pop(joint)
        for k,v in self.child_map.items():
            vnew=[]
            for el in v:
                if el[0]==joint:
                    el=(newjoint,el[1])
                print el
                vnew.append(el)
            self.child_map[k]=vnew
        for k,v in self.parent_map.items():
            if v[0]==joint:
                v=(newjoint,v[1])
            print el
            self.parent_map[k]=v
        for n,j in self.joints.items():
            if j.mimic is not None:
                j.mimic.joint_name=newjoint if j.mimic.joint_name==joint else j.mimic.joint_name

    def copy_joint(self,joint,f,r):
        """Copy and rename a joint and it's parent/child by the f / r strings. Assumes links exist.
        Note that it renames the parent and child, so use this with copy_link"""
        newjoint=copy.deepcopy(self.joints[joint])
        newjoint.name=re.sub(f,r,newjoint.name)
        newjoint.parent=re.sub(f,r,newjoint.parent)
        newjoint.child=re.sub(f,r,newjoint.child)

        self.add_joint(newjoint)
        return newjoint

    def move_chain_with_rottrans(self,root,tip,rpy,xyz,f,r):
        """Find and rename a kinematic chain based on the given root and top
        names. Renames all links and joints according to find and replace
        terms.
        """
        linkchain=self.get_chain(root,tip,links=True,joints=False)
        jointchain=self.get_chain(root,tip,joints=True,links=False)
        print linkchain
        print jointchain

        for l in linkchain[1:]:
            newlink=re.sub(f,r,l)
            self.rename_link(l,newlink)
        for j in jointchain:
            newname=re.sub(f,r,j)
            self.rename_joint(j,newname)
            self.joints[newname].origin.position+=array(xyz)
            self.joints[newname].origin.rotation+=array(rpy)
            if self.joints[newname].mimic is not None:
                self.joints[newname].mimic.joint_name=re.sub(f,r,self.joints[newname].mimic.joint_name)

    def copy_chain_with_rottrans(self,root,tip,rpy,xyz,f,r,mir_ax=None):
        """Copy a kinematic chain, renaming joints and links according to a regular expression.

        Note that this is designed to work with the Hubo joint / body
        convention, which has an easy pattern for joint and body names. If your
        model has joints and links that are not systematically named, this
        function won't be much use.
        """

        linkchain=self.get_chain(root,tip,links=True,joints=False)
        jointchain=self.get_chain(root,tip,joints=True,links=False)
        print linkchain
        print jointchain
        newjoints=[]
        newlinks=[]
        for l in linkchain[1:]:
            newlink=copy.deepcopy(self.links[l])
            newlink.name=re.sub(f,r,newlink.name)
            if newlink.collision is not None:
                newlink.collision.geometry.filename=re.sub(f,r,newlink.collision.geometry.filename)
                newlink.visual.geometry.filename=re.sub(f,r,newlink.visual.geometry.filename)
            self.add_link(newlink)
            newlinks.append(newlink)
            if mir_ax == 'x':
                newlink.inertial.matrix['ixy']*=-1.
                newlink.inertial.matrix['ixz']*=-1.
                newlink.inertial.origin.position[0]*=-1.
            if mir_ax == 'y':
                newlink.inertial.matrix['ixy']*=-1.
                newlink.inertial.matrix['iyz']*=-1.
                newlink.inertial.origin.position[1]*=-1.
            if mir_ax == 'z':
                newlink.inertial.matrix['ixz']*=-1.
                newlink.inertial.matrix['iyz']*=-1.
                newlink.inertial.origin.position[2]*=-1.
        #Hack to rotate just first joint
        for j in jointchain:
            newjoints.append(self.copy_joint(j,f,r))
            if mir_ax == 'x':
                newjoints[-1].origin.position[0]*=-1.0
                newjoints[-1].origin.rotation[1]*=-1.0
                newjoints[-1].origin.rotation[2]*=-1.0
            if mir_ax == 'y':
                newjoints[-1].origin.position[1]*=-1.0
                newjoints[-1].origin.rotation[0]*=-1.0
                newjoints[-1].origin.rotation[2]*=-1.0
            if mir_ax == 'z':
                newjoints[-1].origin.position[2]*=-1.0
                newjoints[-1].origin.rotation[0]*=-1.0
                newjoints[-1].origin.rotation[1]*=-1.0

        if mir_ax =='rotx':
            newjoints[0].origin.position[1]*=-1.0

        for j in newjoints:
            if j.mimic is not None:
                j.mimic.joint_name=re.sub(f,r,j.mimic.joint_name)

        newjoints[0].origin.position+=array(xyz)
        newjoints[0].origin.rotation+=array(rpy)

    def fix_mesh_case(self):
        for l in self.links:
            fname=l.collision.geometry.filename
            l.collision.geometry.filename=re.sub(r'\.[Ss][Tt][Ll]','.stl',fname)
    #TODO: merge function to tie two chains together from disparate models

    def update_mesh_paths(self,package_name):
        """Search and replace package paths in urdf with chosen package name"""
        for n,l in self.links.items():
            #TODO: check if mesh
            for g in [l.collision.geometry,l.visual.geometry]:
                #save STL file name only
                meshfile=g.filename.split('/')[-1]
                newpath=[package_name,'meshes',meshfile]
                cleanpath=re.sub('/+','/','/'.join(newpath))
                g.filename='package://'+cleanpath
            l.collision.geometry.filename=re.sub('Body','convhull',l.collision.geometry.filename)

    def apply_default_limits(self,effort,vel,lower,upper,mask=None):
        """Apply default limits to all joints and convert continous joints to revolute. Ignores fixed and other joint types."""
        for n,j in self.joints.items():
            if mask is None or re.search(mask,n):
                if j.joint_type==Joint.CONTINUOUS or j.joint_type==Joint.REVOLUTE:
                    j.limits=JointLimit(effort,vel,lower,upper)
                    j.joint_type=Joint.REVOLUTE

if __name__ == '__main__':
    #try:
        #from openhubo import startup
    #except ImportError:
        #pass

    try:
        filename=sys.argv[1]
    except IndexError:
        print "Please supply a URDF filename to convert!"

    try:
        outname=sys.argv[2]
    except IndexError:
        outname=None

    model=URDF.load_xml_file(filename)

    model.write_openrave_files(outname)

