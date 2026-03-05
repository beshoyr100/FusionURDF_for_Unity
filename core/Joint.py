# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: syuntoku
"""

import adsk, re
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils

class Joint:
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
        """
        Attributes
        ----------
        name: str
            name of the joint
        type: str
            type of the joint(ex: rev)
        xyz: [x, y, z]
            coordinate of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'
        
    def make_joint_xml(self):
        """
        Generate the joint_xml and hold it by self.joint_xml
        """
        joint = Element('joint')
        joint.attrib = {'name':self.name, 'type':self.type}
        
        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link':self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link':self.child}
        if self.type == 'revolute' or self.type == 'continuous' or self.type == 'prismatic':        
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        if self.type == 'revolute' or self.type == 'prismatic':
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': '100', 'velocity': '100'}
            
        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_transmission_xml(self):
        """
        Generate the tran_xml and hold it by self.tran_xml
        
        
        Notes
        -----------
        mechanicalTransmission: 1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface        
        """        
        
        tran = Element('transmission')
        tran.attrib = {'name':self.name + '_tran'}
        
        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'
        
        joint = SubElement(tran, 'joint')
        joint.attrib = {'name':self.name}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'hardware_interface/EffortJointInterface'
        
        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name':self.name + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'hardware_interface/EffortJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'
        
        self.tran_xml = "\n".join(utils.prettify(tran).split("\n")[1:])


def make_joints_dict(root, msg):
    """Build the joints dictionary from all joints defined on the root component.

    Parameters
    ----------
    root : adsk.fusion.Component
        Root component of the active design.
    msg : str
        Pass-through status message (unchanged on success).

    Returns
    -------
    joints_dict : dict
        {joint_name: {type, axis, upper_limit, lower_limit, parent, child, xyz}}
    msg : str
        Updated status message.
    """

    joint_type_list = [
        'fixed', 'revolute', 'prismatic', 'Cylinderical',
        'PinSlot', 'Planner', 'Ball']  # these are the names in urdf

    if root.joints.count == 0:
        raise RuntimeError(
            'No joints found in the root component.\n\n'
            'Your design needs at least one joint connecting the components.\n'
            'Check the Joints folder in the browser panel.'
        )

    joints_dict = {}

    # Coordinate transformation: applies 4×4 matrix M to point a
    def trans(M, a):
        ex = [M[0], M[4], M[8]]
        ey = [M[1], M[5], M[9]]
        ez = [M[2], M[6], M[10]]
        oo = [M[3], M[7], M[11]]
        return [a[0]*ex[i] + a[1]*ey[i] + a[2]*ez[i] + oo[i] for i in range(3)]

    # Element-wise equality within a tolerance
    def allclose(v1, v2, tol=1e-6):
        return max(abs(a - b) for a, b in zip(v1, v2)) < tol

    # Identity 4×4 as 16-element array (used when occurrence IS the root)
    _identity = [1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1]
    _zero3    = [0, 0, 0]

    for joint in root.joints:
        joint_name = getattr(joint, 'name', '<unnamed joint>')
        try:
            joint_dict = {}

            # ── Joint type ─────────────────────────────────────────────────
            try:
                joint_type = joint_type_list[joint.jointMotion.jointType]
            except (IndexError, Exception) as e:
                raise RuntimeError(
                    'Could not read joint type.\n'
                    'Supported types: Fixed, Revolute, Prismatic.\n\n'
                    'Error detail: {}'.format(e)
                )
            joint_dict['type'] = joint_type
            joint_dict['axis'] = [0, 0, 0]
            joint_dict['upper_limit'] = 0.0
            joint_dict['lower_limit'] = 0.0

            # ── Axis and limits ────────────────────────────────────────────
            if joint_type == 'revolute':
                try:
                    joint_dict['axis'] = [round(i, 6) for i in
                        joint.jointMotion.rotationAxisVector.asArray()]
                except Exception as e:
                    raise RuntimeError(
                        'Could not read rotation axis vector.\n\n'
                        'Error detail: {}'.format(e)
                    )
                limits = joint.jointMotion.rotationLimits
                max_en = limits.isMaximumValueEnabled
                min_en = limits.isMinimumValueEnabled
                if max_en and min_en:
                    joint_dict['upper_limit'] = round(limits.maximumValue, 6)
                    joint_dict['lower_limit'] = round(limits.minimumValue, 6)
                elif max_en and not min_en:
                    raise RuntimeError(
                        'Upper rotation limit is set but the LOWER limit is missing.\n\n'
                        'Fix: open the joint in Fusion and set both rotation limits,\n'
                        'or remove the upper limit to make it a continuous joint.'
                    )
                elif not max_en and min_en:
                    raise RuntimeError(
                        'Lower rotation limit is set but the UPPER limit is missing.\n\n'
                        'Fix: open the joint in Fusion and set both rotation limits,\n'
                        'or remove the lower limit to make it a continuous joint.'
                    )
                else:
                    joint_dict['type'] = 'continuous'

            elif joint_type == 'prismatic':
                try:
                    joint_dict['axis'] = [round(i, 6) for i in
                        joint.jointMotion.slideDirectionVector.asArray()]
                except Exception as e:
                    raise RuntimeError(
                        'Could not read slide direction vector.\n\n'
                        'Error detail: {}'.format(e)
                    )
                limits = joint.jointMotion.slideLimits
                max_en = limits.isMaximumValueEnabled
                min_en = limits.isMinimumValueEnabled
                if max_en and min_en:
                    joint_dict['upper_limit'] = round(limits.maximumValue / 100, 6)
                    joint_dict['lower_limit'] = round(limits.minimumValue / 100, 6)
                elif max_en and not min_en:
                    raise RuntimeError(
                        'Upper slide limit is set but the LOWER limit is missing.\n\n'
                        'Fix: open the joint and set both slide limits.'
                    )
                elif not max_en and min_en:
                    raise RuntimeError(
                        'Lower slide limit is set but the UPPER limit is missing.\n\n'
                        'Fix: open the joint and set both slide limits.'
                    )
            elif joint_type == 'fixed':
                pass
            else:
                raise RuntimeError(
                    'Unsupported joint type: "{}". \n\n'
                    'Only Fixed, Revolute, and Prismatic joints are supported.'.format(joint_type)
                )

            # ── Parent / child occurrences ─────────────────────────────────
            # occurrenceTwo = parent; occurrenceOne = child.
            # Either can be None when the body lives directly in the root
            # component rather than in a sub-occurrence.
            occ_two = joint.occurrenceTwo
            occ_one = joint.occurrenceOne

            if occ_two is None or occ_two.component.name == 'base_link':
                joint_dict['parent'] = 'base_link'
            else:
                joint_dict['parent'] = re.sub('[ :()]', '_', occ_two.name)

            if occ_one is None or occ_one.component.name == 'base_link':
                joint_dict['child'] = 'base_link'
            else:
                joint_dict['child'] = re.sub('[ :()]', '_', occ_one.name)

            # ── Joint origin (xyz) ─────────────────────────────────────────
            # Reference:
            # https://forums.autodesk.com/t5/fusion-360-api-and-scripts/
            # difference-of-geometryororiginone-and-geometryororiginonetwo/m-p/9837767
            # Thanks to Masaki Yamamoto!
            try:
                xyz_from_one_to_joint = joint.geometryOrOriginOne.origin.asArray()
                xyz_from_two_to_joint = joint.geometryOrOriginTwo.origin.asArray()

                xyz_of_one = occ_one.transform.translation.asArray() if occ_one else _zero3
                xyz_of_two = occ_two.transform.translation.asArray() if occ_two else _zero3
                M_two      = occ_two.transform.asArray()              if occ_two else _identity

                case1 = allclose(xyz_from_two_to_joint, xyz_from_one_to_joint)
                case2 = allclose(xyz_from_two_to_joint, xyz_of_one)
                xyz_of_joint = (xyz_from_two_to_joint if (case1 or case2)
                                else trans(M_two, xyz_from_two_to_joint))

                joint_dict['xyz'] = [round(i / 100.0, 6) for i in xyz_of_joint]

            except Exception:
                # Fallback: read origin directly from geometryOrOriginTwo
                try:
                    origin_two = joint.geometryOrOriginTwo
                    if isinstance(origin_two, adsk.fusion.JointOrigin):
                        data = origin_two.geometry.origin.asArray()
                    else:
                        data = origin_two.origin.asArray()
                    joint_dict['xyz'] = [round(i / 100.0, 6) for i in data]
                except Exception as e2:
                    raise RuntimeError(
                        'Could not determine the joint origin position.\n\n'
                        'Fix: open the joint in Fusion, switch to the "Motion" tab,\n'
                        'and confirm that a valid joint origin is set for both sides.\n\n'
                        'Error detail: {}'.format(e2)
                    )

            joints_dict[joint.name] = joint_dict

        except RuntimeError:
            raise   # already has a good message — let it propagate

        except Exception as e:
            raise RuntimeError(
                'Unexpected error while processing joint "{}".\n\n'
                'What to check:\n'
                '  • Occurrence 1 (child):  {}\n'
                '  • Occurrence 2 (parent): {}\n'
                '  • The joint has a valid origin set on both sides.\n'
                '  • Both connected components exist and are not suppressed.\n\n'
                'Error detail: {}'.format(
                    joint_name,
                    getattr(joint.occurrenceOne, 'name', 'root component'),
                    getattr(joint.occurrenceTwo, 'name', 'root component'),
                    e,
                )
            )

    return joints_dict, msg

