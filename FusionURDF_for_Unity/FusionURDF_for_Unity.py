# Author: Bishoy Labib
# Inspired by work of: syuntoku14, Dheena2k2, Lentin Joseph
#
# Description
# -----------
# Fusion 360 Add-in entry point.
#
# Exports the active design as a self-contained URDF package suitable for
# Unity's URDF Importer:
#
#   {output_dir}/{robot_name}/
#       {robot_name}.urdf
#       meshes/
#           base_link.obj
#           <component_name>.obj
#           ...
#
# Fusion 360 workspace restoration
# ---------------------------------
# copy_occs() temporarily renames and duplicates components inside the active
# design.  After the OBJ export is done we restore the design by rolling the
# timeline back to the marker position recorded just before the changes were
# made.  This is equivalent to pressing Ctrl+Z for every operation that
# copy_occs performed, but is 100 % reliable regardless of how many steps
# were involved.
#
# The files written to disk (URDF + OBJ) are NOT rolled back — only the
# in-memory Fusion model is restored.
#
# Supported joint types: Revolute, Rigid (fixed), Prismatic, Continuous

import adsk
import adsk.core
import adsk.fusion
import traceback

from .core import Link, Joint, Write, utils


def run(context):
    ui = None
    title = 'Fusion 360 → URDF (Unity)'

    try:
        # ------------------------------------------------------------------ #
        # 1. Initialise                                                        #
        # ------------------------------------------------------------------ #
        app = adsk.core.Application.get()
        ui = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        if not design:
            ui.messageBox('No active Fusion design found.', title)
            return

        root = design.rootComponent
        components = design.allComponents
        robot_name = root.name.split()[0]

        # ------------------------------------------------------------------ #
        # 2. Welcome dialog                                                    #
        # ------------------------------------------------------------------ #
        welcome = (
            "Welcome to the Fusion 360 → URDF (Unity) exporter.\n\n"
            "This tool generates a self-contained URDF file and OBJ meshes\n"
            "in the format expected by Unity's URDF Importer.\n\n"
            "Output structure:\n"
            "  {robot_name}/\n"
            "    {robot_name}.urdf\n"
            "    meshes/  ← one .obj per link\n\n"
            "Press OK to continue or Cancel to quit."
        )
        btn = adsk.core.MessageBoxButtonTypes.OKCancelButtonType
        ok = adsk.core.DialogResults.DialogOK
        if ui.messageBox(welcome, title, btn) != ok:
            return

        # ------------------------------------------------------------------ #
        # 3. Browse for output folder                                          #
        # ------------------------------------------------------------------ #
        browse_msg = "Press OK to choose the output folder, or Cancel to quit."
        if ui.messageBox(browse_msg, title, btn) != ok:
            return

        save_dir = utils.file_dialog(ui)
        if save_dir is False:
            ui.messageBox('Export cancelled.', title)
            return

        # ------------------------------------------------------------------ #
        # 4. Build data dictionaries (read-only Fusion API calls)             #
        # ------------------------------------------------------------------ #
        success_msg = 'Successfully created URDF file'

        joints_dict, msg = Joint.make_joints_dict(root, success_msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return

        inertial_dict, msg = Link.make_inertial_dict(root, success_msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return

        if 'base_link' not in inertial_dict:
            ui.messageBox(
                'No base_link found.\n'
                'Please name your root body component "base_link" and run again.',
                title,
            )
            return

        # ------------------------------------------------------------------ #
        # 5. Write URDF to disk (no Fusion changes yet)                       #
        # ------------------------------------------------------------------ #
        links_xyz_dict = {}
        robot_dir, _ = Write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, robot_name, save_dir)

        # ------------------------------------------------------------------ #
        # 6. Export OBJ meshes — restore all Fusion design changes afterwards #
        # ------------------------------------------------------------------ #
        # copy_occs() creates new components and renames existing occurrences.
        # We roll these back using the best method for the current design type:
        #
        #   Parametric design → timeline marker rollback (one atomic rewind)
        #   Direct Modelling  → entity-token snapshot + manual delete/rename

        is_parametric = (
            design.designType == adsk.fusion.DesignTypes.ParametricDesignType
        )

        if is_parametric:
            # ── Parametric: snapshot timeline marker ──────────────────────
            timeline = design.timeline
            marker_before = timeline.markerPosition
            try:
                utils.copy_occs(root)
                utils.export_obj(design, save_dir, robot_name, components)
            finally:
                # Rewind to the pre-export position — atomically undoes
                # every step copy_occs performed.
                timeline.markerPosition = marker_before

        else:
            # ── Direct Modelling: entity-token snapshot ───────────────────
            # Record what exists before copy_occs touches anything.
            components_before = {c.entityToken for c in design.allComponents}
            occ_names_before = {
                occ.entityToken: occ.component.name
                for occ in root.occurrences
            }
            try:
                utils.copy_occs(root)
                utils.export_obj(design, save_dir, robot_name, components)
            finally:
                # 1. Delete every new component copy_occs created
                for comp in list(design.allComponents):
                    if comp.entityToken not in components_before:
                        try:
                            comp.deleteMe()
                        except Exception:
                            pass  # already gone, or read-only — safe to skip

                # 2. Restore original component names on pre-existing occurrences
                for occ in root.occurrences:
                    token = occ.entityToken
                    if token in occ_names_before:
                        original_name = occ_names_before[token]
                        if occ.component.name != original_name:
                            occ.component.name = original_name

        # ------------------------------------------------------------------ #
        # 7. Done                                                              #
        # ------------------------------------------------------------------ #
        ui.messageBox(
            'Export complete!\n\n'
            'Output: {}/{}/'.format(save_dir, robot_name),
            title,
        )

    except Exception:
        if ui:
            ui.messageBox('Export failed:\n{}'.format(traceback.format_exc()), title)

