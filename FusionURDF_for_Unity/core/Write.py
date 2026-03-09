# -*- coding: utf-8 -*-
"""
Write.py
--------
Generates the URDF file from the joints and inertial dictionaries
produced by Joint.make_joints_dict() and Link.make_inertial_dict().

Output format
-------------
A single, self-contained ``{robot_name}.urdf`` placed at::

    {save_dir}/{robot_name}/{robot_name}.urdf

Mesh references use the ROS package:// URI convention so the file can
be loaded directly by Unity's URDF Importer or any ROS-aware tool::

    <mesh filename="package://{robot_name}/meshes/{link_name}.obj"
          scale="0.01 0.01 0.01"/>

No xacro, no launch files, no Gazebo/ROS 2 specific files are generated.
"""

import adsk
import os
from xml.etree.ElementTree import Element, SubElement

from . import Link, Joint, utils


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

def _write_link_section(joints_dict, robot_name, links_xyz_dict, file_name, inertial_dict):
    """Write every <link> element into *file_name*.

    Also populates *links_xyz_dict* with the world-frame xyz of each link,
    which is consumed later by :func:`_write_joint_section`.

    Parameters
    ----------
    joints_dict : dict
        Joint information from :func:`Joint.make_joints_dict`.
    robot_name : str
        Used to build the ``package://`` mesh URI.
    links_xyz_dict : dict
        Empty dict that will be filled: ``{link_name: [x, y, z]}``.
    file_name : str
        Absolute path to the open URDF file (appended to).
    inertial_dict : dict
        Inertial information from :func:`Link.make_inertial_dict`.

    Notes
    -----
    For child links the centre-of-mass is expressed relative to the
    *link's own origin*, which is located at the joint position.  So::

        com_in_link_frame = com_world - joint_xyz_world
    """
    with open(file_name, mode='a') as f:
        # base_link — its origin is the world origin, so xyz = [0,0,0]
        com = inertial_dict['base_link']['center_of_mass']
        link = Link.Link(
            name='base_link',
            xyz=[0, 0, 0],
            center_of_mass=com,
            robot_name=robot_name,
            mass=inertial_dict['base_link']['mass'],
            inertia_tensor=inertial_dict['base_link']['inertia'],
        )
        links_xyz_dict[link.name] = link.xyz
        link.make_link_xml()
        f.write(link.link_xml)
        f.write('\n')

        # All other links (children of some joint)
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            joint_xyz = joints_dict[joint]['xyz']
            # Centre-of-mass relative to the link's own frame
            com = [i - j for i, j in zip(inertial_dict[name]['center_of_mass'], joint_xyz)]
            link = Link.Link(
                name=name,
                xyz=joint_xyz,
                center_of_mass=com,
                robot_name=robot_name,
                mass=inertial_dict[name]['mass'],
                inertia_tensor=inertial_dict[name]['inertia'],
            )
            links_xyz_dict[link.name] = link.xyz
            link.make_link_xml()
            f.write(link.link_xml)
            f.write('\n')


def _write_joint_section(joints_dict, links_xyz_dict, file_name):
    """Write every <joint> element into *file_name*.

    The joint's xyz in the URDF is the *offset from the parent link's
    origin to the child link's origin*::

        joint_xyz_urdf = parent_xyz_world - child_xyz_world

    Parameters
    ----------
    joints_dict : dict
        Joint information from :func:`Joint.make_joints_dict`.
    links_xyz_dict : dict
        World-frame xyz of each link, populated by :func:`_write_link_section`.
    file_name : str
        Absolute path to the open URDF file (appended to).
    """
    with open(file_name, mode='a') as f:
        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']

            try:
                # xyz = parent_origin - child_origin  (in world frame)
                xyz = [round(p - c, 6)
                       for p, c in zip(links_xyz_dict[parent], links_xyz_dict[child])]
            except KeyError:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox(
                    'Connection error between\n\n{}\nand\n{}\n\n'
                    'Check whether parent=component2={} and child=component1={} '
                    'are correct, or swap component1 ↔ component2.'.format(
                        parent, child, parent, child),
                    'URDF Export Error',
                )
                raise

            joint = Joint.Joint(
                name=j,
                joint_type=joint_type,
                xyz=xyz,
                axis=joints_dict[j]['axis'],
                parent=parent,
                child=child,
                upper_limit=upper_limit,
                lower_limit=lower_limit,
            )
            joint.make_joint_xml()
            f.write(joint.joint_xml)
            f.write('\n')


def _write_transmission_section(joints_dict, links_xyz_dict, file_name):
    """Write <transmission> blocks for every non-fixed joint.

    Transmissions are included for compatibility with tools that read them
    (they are harmless when unused, e.g. in Unity).
    """
    with open(file_name, mode='a') as f:
        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']

            try:
                xyz = [round(p - c, 6)
                       for p, c in zip(links_xyz_dict[parent], links_xyz_dict[child])]
            except KeyError:
                continue  # already reported by _write_joint_section

            joint = Joint.Joint(
                name=j,
                joint_type=joint_type,
                xyz=xyz,
                axis=joints_dict[j]['axis'],
                parent=parent,
                child=child,
                upper_limit=upper_limit,
                lower_limit=lower_limit,
            )
            if joint_type != 'fixed':
                joint.make_transmission_xml()
                f.write(joint.tran_xml)
                f.write('\n')




# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def write_urdf(joints_dict, links_xyz_dict, inertial_dict, robot_name, save_dir):
    """Generate the complete URDF file.

    Creates the directory structure::

        {save_dir}/{robot_name}/
            {robot_name}.urdf
            meshes/           (populated separately by utils.export_stl)

    Parameters
    ----------
    joints_dict : dict
        From :func:`Joint.make_joints_dict`.
    links_xyz_dict : dict
        Empty dict; filled internally and reused across sub-writers.
    inertial_dict : dict
        From :func:`Link.make_inertial_dict`.
    robot_name : str
        Name of the robot (= root component name, first word).
    save_dir : str
        User-selected output directory.  The robot folder is created here.
    """
    robot_dir  = os.path.join(save_dir, robot_name)           # outer folder
    inner_dir  = os.path.join(robot_dir, robot_name)           # inner same-named folder
    meshes_dir = os.path.join(inner_dir, 'meshes')             # meshes live here

    os.makedirs(robot_dir,  exist_ok=True)
    os.makedirs(meshes_dir, exist_ok=True)

    file_name = os.path.join(robot_dir, robot_name + '.urdf')

    # ---- File header -------------------------------------------------------
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}">\n\n'.format(robot_name))
        # No global material override — let Unity use the mesh's own material

    # ---- Body sections -----------------------------------------------------
    _write_link_section(joints_dict, robot_name, links_xyz_dict, file_name, inertial_dict)
    _write_joint_section(joints_dict, links_xyz_dict, file_name)
    _write_transmission_section(joints_dict, links_xyz_dict, file_name)

    # ---- Closing tag -------------------------------------------------------
    with open(file_name, mode='a') as f:
        f.write('</robot>\n')

    return robot_dir, inner_dir
