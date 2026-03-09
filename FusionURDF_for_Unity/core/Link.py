# -*- coding: utf-8 -*-
"""
Link.py
-------
Defines the Link class and the make_inertial_dict() helper.

Each Link represents a single URDF <link> element.  It carries the
inertial properties (mass, inertia tensor, centre-of-mass) and generates
the XML for the visual and collision geometry, referencing the exported
OBJ mesh via a ROS-style package:// URI.

Mathematical conventions are documented in docs/MESH_AND_MATH.md.
"""

import adsk
import re
from xml.etree.ElementTree import Element, SubElement
from . import utils


class Link:
    """Represents a single URDF <link> element.

    Parameters
    ----------
    name : str
        Name of the link (must match the exported STL filename, without .stl).
    xyz : list of float [x, y, z]
        Origin of the link in the *parent* joint frame (metres).
        The sign is negated internally to obtain the visual/collision origin.
    center_of_mass : list of float [x, y, z]
        Centre-of-mass position relative to the *link* frame (metres).
    robot_name : str
        Top-level robot / package name used to build the package:// URI.
    mass : float
        Mass in kilograms.
    inertia_tensor : list of float [ixx, iyy, izz, ixy, iyz, ixz]
        Inertia tensor about the centre of mass (kg·m²).
    """

    def __init__(self, name, xyz, center_of_mass, robot_name, mass, inertia_tensor):
        self.name = name
        # Visual / collision origin = negated joint-frame offset (see MESH_AND_MATH.md §5)
        self.xyz = [-_ for _ in xyz]
        self.center_of_mass = center_of_mass
        self.robot_name = robot_name
        self.mass = mass
        self.inertia_tensor = inertia_tensor
        self.link_xml = None

    # ------------------------------------------------------------------
    def make_link_xml(self):
        """Build the URDF <link> XML string and store it in self.link_xml."""

        link = Element('link')
        link.attrib = {'name': self.name}

        # --- inertial ---------------------------------------------------
        inertial = SubElement(link, 'inertial')

        origin_i = SubElement(inertial, 'origin')
        origin_i.attrib = {
            'xyz': ' '.join(str(v) for v in self.center_of_mass),
            'rpy': '0 0 0',
        }

        mass_el = SubElement(inertial, 'mass')
        mass_el.attrib = {'value': str(self.mass)}

        inertia_el = SubElement(inertial, 'inertia')
        inertia_el.attrib = {
            'ixx': str(self.inertia_tensor[0]),
            'iyy': str(self.inertia_tensor[1]),
            'izz': str(self.inertia_tensor[2]),
            'ixy': str(self.inertia_tensor[3]),
            'iyz': str(self.inertia_tensor[4]),
            'ixz': str(self.inertia_tensor[5]),
        }

        # Mesh URI: package://{robot_name}/meshes/{link_name}.obj
        mesh_uri = 'package://{}/meshes/{}.obj'.format(self.robot_name, self.name)

        # --- visual -----------------------------------------------------
        visual = SubElement(link, 'visual')
        origin_v = SubElement(visual, 'origin')
        origin_v.attrib = {
            'xyz': ' '.join(str(v) for v in self.xyz),
            'rpy': '0 0 0',
        }
        geometry_v = SubElement(visual, 'geometry')
        mesh_v = SubElement(geometry_v, 'mesh')
        mesh_v.attrib = {'filename': mesh_uri, 'scale': '0.01 0.01 0.01'}
        # No <material> tag — let Unity render using the mesh's own material

        # --- collision --------------------------------------------------
        collision = SubElement(link, 'collision')
        origin_c = SubElement(collision, 'origin')
        origin_c.attrib = {
            'xyz': ' '.join(str(v) for v in self.xyz),
            'rpy': '0 0 0',
        }
        geometry_c = SubElement(collision, 'geometry')
        mesh_c = SubElement(geometry_c, 'mesh')
        mesh_c.attrib = {'filename': mesh_uri, 'scale': '0.01 0.01 0.01'}

        self.link_xml = '\n'.join(utils.prettify(link).split('\n')[1:])


# ---------------------------------------------------------------------------
# Module-level helper
# ---------------------------------------------------------------------------

def make_inertial_dict(root, msg):
    """Collect physical properties for every occurrence in the design.

    Parameters
    ----------
    root : adsk.fusion.Component
        The root component of the active design.
    msg : str
        Pass-through status message (unchanged on success).

    Returns
    -------
    inertial_dict : dict
        Keyed by link name.  Each value is a dict with keys:
        ``name``, ``mass`` (kg), ``center_of_mass`` ([x,y,z] metres),
        ``inertia`` ([ixx,iyy,izz,ixy,iyz,ixz] kg·m²).
    msg : str
        Updated status message.

    Notes
    -----
    * Fusion 360 reports distances in **centimetres** and moments of inertia
      in **kg·cm²**.  Both are converted to SI (metres / kg·m²) here.
    * ``origin2center_of_mass`` applies the parallel-axis theorem to shift the
      inertia tensor from the world origin to the centre of mass.
      (See docs/MESH_AND_MATH.md §3 for the full derivation.)
    """
    allOccs = root.occurrences

    if allOccs.count == 0:
        raise RuntimeError(
            'No occurrences found in the root component.\n\n'
            'Make sure your design has at least one body placed as a component\n'
            'with a "base_link" component named correctly.'
        )

    inertial_dict = {}

    for occs in allOccs:
        occ_label = getattr(occs, 'name', '<unknown occurrence>')
        try:
            occs_dict = {}

            prop = occs.getPhysicalProperties(
                adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy
            )
            if prop is None:
                raise RuntimeError('getPhysicalProperties() returned None.')

            occs_dict['name'] = re.sub('[ :()]', '_', occs.name)

            # Mass: kg (Fusion already returns kg)
            mass = prop.mass
            if mass is None or mass <= 0:
                raise RuntimeError(
                    'Mass is zero or could not be read.\n'
                    'Check that the component has physical material assigned.'
                )
            occs_dict['mass'] = mass

            # Centre of mass: cm → m
            com_raw = prop.centerOfMass
            if com_raw is None:
                raise RuntimeError('centerOfMass could not be computed.')
            center_of_mass = [v / 100.0 for v in com_raw.asArray()]
            occs_dict['center_of_mass'] = center_of_mass

            # Inertia tensor about world origin: kg·cm² → kg·m²
            # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-ce341ee6-4490-11e5-b25b-f8b156d7cd97
            inertia_result = prop.getXYZMomentsOfInertia()
            if not inertia_result or len(inertia_result) < 7:
                raise RuntimeError('getXYZMomentsOfInertia() returned unexpected data.')
            (_, xx, yy, zz, xy, yz, xz) = inertia_result
            moment_inertia_world = [v / 10000.0 for v in [xx, yy, zz, xy, yz, xz]]

            # Shift to centre-of-mass frame via parallel-axis theorem
            occs_dict['inertia'] = utils.origin2center_of_mass(
                moment_inertia_world, center_of_mass, mass
            )

            if occs.component.name == 'base_link':
                inertial_dict['base_link'] = occs_dict
            else:
                inertial_dict[re.sub('[ :()]', '_', occs.name)] = occs_dict

        except Exception as e:
            raise RuntimeError(
                'Failed to read physical properties for occurrence "{}".\n\n'
                'What to check:\n'
                '  • The component has a valid material assigned (not "Generic").\n'
                '  • The component contains at least one solid body.\n'
                '  • The component is not suppressed or empty.\n\n'
                'Error detail: {}'.format(occ_label, e)
            )

    return inertial_dict, msg

