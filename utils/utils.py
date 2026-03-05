# -*- coding: utf-8 -*-
"""
utils.py
--------
Shared utility functions for the Fusion 360 URDF exporter.

Responsibilities
----------------
* :func:`copy_occs`           — Duplicate Fusion occurrences before STL export
* :func:`export_stl`          — Export each occurrence as an STL binary file
* :func:`file_dialog`         — Show the OS folder-picker dialog
* :func:`origin2center_of_mass` — Parallel-axis theorem (inertia shift)
* :func:`prettify`            — Pretty-print an XML element to a string

All mathematics are documented in docs/MESH_AND_MATH.md.
"""

import adsk
import adsk.core
import adsk.fusion
import os
import re
from xml.etree import ElementTree
from xml.dom import minidom


# ---------------------------------------------------------------------------
# Mesh helpers
# ---------------------------------------------------------------------------

def copy_occs(root):
    """Duplicate every occurrence so each link becomes its own component.

    Fusion 360 exports STL per-component.  When multiple occurrences share the
    same component, only one STL would be produced.  This function resolves
    that by creating a *new* component for each occurrence and copying all
    B-Rep bodies into it.

    The original occurrences are renamed to ``old_component`` so they can be
    filtered out during export (see :func:`export_stl`).

    .. warning::
        This function **modifies** the active Fusion design.  The caller is
        responsible for rolling back these changes.
        See :func:`run` in ``FusionURDF_for_Unity.py``.

    Parameters
    ----------
    root : adsk.fusion.Component
        The root component of the active design.
    """
    def _copy_body(all_occs, occ):
        """Copy all B-Rep bodies from *occ* into a newly created component."""
        occ_label = getattr(occ, 'name', '<unknown>')
        bodies = occ.bRepBodies

        try:
            transform = adsk.core.Matrix3D.create()
            new_occ = all_occs.addNewComponent(transform)
        except Exception as e:
            raise RuntimeError(
                'Could not create a new component for occurrence "{}". \n\n'
                'What to check:\n'
                '  • The occurrence is not locked or external.\n'
                '  • The design is not read-only.\n\n'
                'Error detail: {}'.format(occ_label, e)
            )

        # Name the new component after the occurrence (sanitise special chars)
        try:
            if occ.component.name == 'base_link':
                occ.component.name = 'old_component'
                new_occ.component.name = 'base_link'
            else:
                new_occ.component.name = re.sub('[ :()]', '_', occ.name)
        except Exception as e:
            raise RuntimeError(
                'Could not rename component for occurrence "{}". \n\n'
                'Error detail: {}'.format(occ_label, e)
            )

        # Refresh reference — addNewComponent appends to the end
        new_occ = all_occs.item(all_occs.count - 1)

        for i in range(bodies.count):
            body = bodies.item(i)
            body_label = getattr(body, 'name', 'body #{}'.format(i))
            try:
                body.copyToComponent(new_occ)
            except Exception as e:
                raise RuntimeError(
                    'Could not copy body "{}" from occurrence "{}" to new component.\n\n'
                    'What to check:\n'
                    '  • The body is a valid solid (not a surface or mesh body).\n'
                    '  • The body is not suppressed.\n\n'
                    'Error detail: {}'.format(body_label, occ_label, e)
                )

    all_occs = root.occurrences

    if all_occs.count == 0:
        raise RuntimeError(
            'No occurrences found in the root component.\n\n'
            'The design must contain at least one component occurrence with solid bodies.'
        )

    old_occs = []
    copy_list = list(all_occs)  # snapshot before we start mutating

    for occ in copy_list:
        if occ.bRepBodies.count > 0:
            _copy_body(all_occs, occ)
            old_occs.append(occ)

    # Mark originals so export_stl can skip them
    for occ in old_occs:
        try:
            occ.component.name = 'old_component'
        except Exception as e:
            raise RuntimeError(
                'Could not mark original occurrence "{}" as "old_component".\n\n'
                'Error detail: {}'.format(getattr(occ, 'name', '?'), e)
            )


def export_stl(design, save_dir, robot_name, components):
    """Export each occurrence as a binary STL file into *meshes/*.

    Only occurrences whose component name does **not** contain
    ``old_component`` are exported (the duplicates created by
    :func:`copy_occs` are the ones we want; the originals are skipped).

    STL files are written to::

        {save_dir}/{robot_name}/meshes/{component_name}.stl

    Parameters
    ----------
    design : adsk.fusion.Design
        The active Fusion design.
    save_dir : str
        User-selected root output directory.
    robot_name : str
        Robot name (used to locate the correct output sub-folder).
    components : adsk.fusion.ComponentList
        ``design.allComponents``.
    """
    export_mgr = design.exportManager
    # Meshes go in the inner same-named subfolder:
    # {save_dir}/{robot_name}/{robot_name}/meshes/
    meshes_dir = os.path.join(save_dir, robot_name, robot_name, 'meshes')

    try:
        os.makedirs(meshes_dir, exist_ok=True)
    except Exception as e:
        raise RuntimeError(
            'Could not create meshes directory at:\n  {}\n\n'
            'Check that the output path is writable.\n\n'
            'Error detail: {}'.format(meshes_dir, e)
        )

    exported = []
    failed   = []

    for component in components:
        for occ in component.allOccurrences:
            if 'old_component' in occ.component.name:
                continue

            comp_name = occ.component.name
            file_path = os.path.join(meshes_dir, comp_name)

            try:
                opts = export_mgr.createSTLExportOptions(occ, file_path)
                opts.sendToPrintUtility = False
                opts.isBinaryFormat = True
                # Low refinement keeps file sizes manageable
                opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                export_mgr.execute(opts)
                exported.append(comp_name)
            except Exception as e:
                failed.append('  • {} — {}'.format(comp_name, e))

    if failed:
        raise RuntimeError(
            'STL export completed with errors.\n\n'
            'The following mesh(es) could not be exported:\n'
            '{}\n\n'
            'What to check for each failed mesh:\n'
            '  • The component contains at least one solid body.\n'
            '  • Bodies are not intersecting in a way that prevents meshing.\n'
            '  • The component is visible and not suppressed.\n\n'
            'Successfully exported {} mesh(es): {}'.format(
                '\n'.join(failed),
                len(exported),
                ', '.join(exported) or 'none',
            )
        )




# ---------------------------------------------------------------------------
# UI helper
# ---------------------------------------------------------------------------

def file_dialog(ui):
    """Show an OS folder-picker and return the selected path.

    Parameters
    ----------
    ui : adsk.core.UserInterface

    Returns
    -------
    str or False
        Absolute path to the chosen folder, or ``False`` if cancelled.
    """
    dlg = ui.createFolderDialog()
    dlg.title = 'Select output folder for URDF export'
    result = dlg.showDialog()
    if result == adsk.core.DialogResults.DialogOK:
        return dlg.folder
    return False


# ---------------------------------------------------------------------------
# Mathematics
# ---------------------------------------------------------------------------

def origin2center_of_mass(inertia, center_of_mass, mass):
    """Shift the inertia tensor from the world origin to the centre of mass.

    Uses the *parallel-axis theorem* (Steiner's theorem)::

        I_cm = I_world - m * (|r|² δᵢⱼ - rᵢ rⱼ)

    where **r** = [x, y, z] is the vector from the world origin to the
    centre of mass.

    In component form the correction matrix D is::

        D_xx = y² + z²     D_xy = -xy
        D_yy = x² + z²     D_yz = -yz
        D_zz = x² + y²     D_xz = -xz

    So::

        [ixx, iyy, izz, ixy, iyz, ixz]_cm =
        [ixx, iyy, izz, ixy, iyz, ixz]_world - mass * [D_xx, D_yy, D_zz, D_xy, D_yz, D_xz]

    Parameters
    ----------
    inertia : list of float [ixx, iyy, izz, ixy, iyz, ixz]
        Inertia tensor about the **world** origin (kg·m²).
    center_of_mass : list of float [x, y, z]
        Centre-of-mass position in the world frame (metres).
    mass : float
        Mass in kilograms.

    Returns
    -------
    list of float [ixx, iyy, izz, ixy, iyz, ixz]
        Inertia tensor about the **centre of mass** (kg·m²), rounded to 6 dp.
    """
    x, y, z = center_of_mass
    D = [y**2 + z**2, x**2 + z**2, x**2 + y**2, -x*y, -y*z, -x*z]
    return [round(i - mass * d, 6) for i, d in zip(inertia, D)]


# ---------------------------------------------------------------------------
# XML helper
# ---------------------------------------------------------------------------

def prettify(elem):
    """Return a pretty-printed, indented XML string for *elem*.

    The leading XML declaration line (``<?xml version="1.0" ?>``) is kept;
    callers that want to strip it use ``split('\\n')[1:]``.

    Parameters
    ----------
    elem : xml.etree.ElementTree.Element

    Returns
    -------
    str
    """
    raw = ElementTree.tostring(elem, encoding='utf-8')
    reparsed = minidom.parseString(raw)
    return reparsed.toprettyxml(indent='  ')
