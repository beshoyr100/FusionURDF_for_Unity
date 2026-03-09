# Mesh Pipeline & Mathematics Reference

This document explains every step that the **Fusion 360 → URDF (Unity)** exporter
performs on the mesh geometry and all the mathematics embedded in the core classes.

---

## 1. Overview — What the Plugin Does

```
Fusion 360 design
      │
      ▼
make_joints_dict()   ─── reads joint type, axis, limits, and world-frame XYZ
make_inertial_dict() ─── reads mass, centre-of-mass, inertia tensor (per occurrence)
      │
      ▼
Write.write_urdf()   ─── produces {robot_name}/{robot_name}.urdf on disk
      │
      ▼
utils.copy_occs()    ─── duplicates occurrences so each link has a unique component
utils.export_stl()   ─── exports each component as a binary STL into meshes/
      │
      ▼
transaction.abort()  ─── rolls back ALL Fusion design changes (copy_occs effects)
```

---

## 2. Mesh Pipeline

### 2.1 Why copy_occs() is Necessary

In Fusion 360 a *component* can be instantiated many times (multiple
*occurrences*).  The STL export API operates on an *occurrence*, but two
occurrences that share the same component would produce identically-named
STL files, overwriting each other.

`copy_occs()` resolves this by:

1. Iterating every occurrence in the root component that has at least one
   B-Rep body.
2. For each such occurrence, creating a **new, empty component** with
   `allOccurrences.addNewComponent(identity_matrix)`.
3. Copying all B-Rep bodies from the original occurrence into the new one.
4. Naming the new component after the **occurrence** name (not the component
   name), sanitised with `re.sub('[ :()]', '_', occ.name)` so the name is a
   valid file-system identifier.
5. Renaming all original occurrences to `old_component` so they are
   excluded from the STL export step.

### 2.2 Mesh Export (export_stl)

`export_stl()` walks `design.allComponents`, iterates their occurrences, and
skips any whose component name contains `"old_component"`.  The remaining
occurrences are the duplicates created in step 2.1.

For each kept occurrence an STL is written to:

```
{save_dir}/{robot_name}/meshes/{component_name}.stl
```

Export settings:

| Setting | Value |
|---|---|
| Format | Binary STL |
| Mesh refinement | **Low** (keeps file sizes small) |
| `sendToPrintUtility` | False (silent export) |

### 2.3 Fusion Workspace Restoration

`copy_occs()` has permanent side-effects on the Fusion design (new components,
renamed originals).  The script wraps the `copy_occs + export_stl` block in a
**Fusion 360 UndoManager Transaction**:

```python
transaction = design.undoManager.createTransaction()
try:
    utils.copy_occs(root)
    utils.export_stl(design, save_dir, robot_name, components)
finally:
    transaction.abort()   # atomically reverts ALL in-memory changes
```

`transaction.abort()` is equivalent to pressing Ctrl+Z enough times to undo
every operation that occurred inside the transaction — but it does it
atomically and reliably regardless of how many operations were performed.

> **Important:** `transaction.abort()` only reverts the **Fusion design in
> memory**.  Files already written to disk (STLs, URDF) are **not** deleted.

---

## 3. Coordinate System & Unit Conversions

| Quantity | Fusion 360 unit | URDF unit | Conversion |
|---|---|---|---|
| Length / position | centimetres (cm) | metres (m) | ÷ 100 |
| Mass | kilograms (kg) | kilograms (kg) | × 1 |
| Moment of inertia | kg·cm² | kg·m² | ÷ 10 000 |
| Rotation angles | radians | radians | × 1 |
| Mesh vertex positions | millimetres (mm) | metres via scale | `scale="0.001 0.001 0.001"` |

STL files exported from Fusion store vertex coordinates in **millimetres**.
The URDF `<mesh scale="0.001 0.001 0.001"/>` attribute instructs the loader
to scale each vertex by 0.001, converting mm → m automatically.

---

## 4. Inertia Tensor Mathematics

### 4.1 What Fusion Returns

`prop.getXYZMomentsOfInertia()` returns the inertia tensor components
**about the world-frame origin** (not the centre of mass):

```
(_, Ixx_w, Iyy_w, Izz_w, Ixy_w, Iyz_w, Ixz_w)   [kg·cm²]
```

### 4.2 Parallel-Axis Theorem (Steiner's Theorem)

To shift from the world origin **O** to the centre of mass **C** we use:

```
I_C = I_O - m · D
```

where **D** is the "correction matrix".  In the [ixx, iyy, izz, ixy, iyz, ixz]
representation, with **r** = [x, y, z] = position of **C** relative to **O**:

```
D_xx = y² + z²
D_yy = x² + z²
D_zz = x² + y²
D_xy = -x·y
D_yz = -y·z
D_xz = -x·z
```

Implemented in `utils.origin2center_of_mass()`:

```python
x, y, z = center_of_mass                          # metres
D = [y**2+z**2, x**2+z**2, x**2+y**2, -x*y, -y*z, -x*z]
result = [round(I - m*d, 6) for I, d in zip(inertia_world, D)]
```

### 4.3 Why This Matters

URDF requires the inertia tensor to be expressed **about the centre of mass**.
Skipping this step would produce physically incorrect simulation behaviour
(wrong rotation dynamics).

---

## 5. Joint Origin Mathematics

### 5.1 Joint Position in the World Frame

Each Fusion joint stores two origin points:

- `geometryOrOriginOne.origin` — joint position relative to **occurrence 1** (child)
- `geometryOrOriginTwo.origin` — joint position relative to **occurrence 2** (parent)

Fusion's API has known inconsistencies between these two.  The original code
(preserved verbatim) uses the following heuristic:

```python
def trans(M, a):
    # Applies the 4×4 transformation matrix M to point a
    ex = [M[0], M[4], M[8]]
    ey = [M[1], M[5], M[9]]
    ez = [M[2], M[6], M[10]]
    oo = [M[3], M[7], M[11]]
    return [a[0]*ex[i] + a[1]*ey[i] + a[2]*ez[i] + oo[i] for i in range(3)]

def allclose(v1, v2, tol=1e-6):
    return max(abs(a-b) for a, b in zip(v1, v2)) < tol

xyz_from_one_to_joint = joint.geometryOrOriginOne.origin.asArray()
xyz_from_two_to_joint = joint.geometryOrOriginTwo.origin.asArray()
xyz_of_one            = joint.occurrenceOne.transform.translation.asArray()
M_two                 = joint.occurrenceTwo.transform.asArray()  # 4×4 → 16 elements

case1 = allclose(xyz_from_two_to_joint, xyz_from_one_to_joint)
case2 = allclose(xyz_from_two_to_joint, xyz_of_one)

if case1 or case2:
    xyz_of_joint = xyz_from_two_to_joint        # already in world frame
else:
    xyz_of_joint = trans(M_two, xyz_from_two_to_joint)  # transform to world
```

The result is converted from cm to m: `joint_dict['xyz'] = [v / 100.0 for v in xyz_of_joint]`.

### 5.2 URDF Joint Origin

The URDF `<joint><origin xyz="…"/>` is the offset from the **parent link's
local origin** to the **child link's local origin**, expressed in the parent's
frame.

In `Write._write_joint_section()`:

```python
xyz_urdf = parent_link_xyz_world - child_link_xyz_world
```

Both values come from `links_xyz_dict`, which stores the *negated* joint xyz
(see §5.3 below).

### 5.3 Link Visual/Collision Origin — Sign Negation

In `Link.__init__()`:

```python
self.xyz = [-v for v in xyz]   # visual/collision origin = -joint_xyz
```

This sign flip places the mesh at the correct position in the link's own
frame.  Because the joint xyz is the *world position* of the link's origin, the
visual/collision geometry must be offset by its negative to sit at that
world position when the parent transform is applied.

---

## 6. Axis Vector Normalisation

Fusion returns joint rotation / slide axis vectors that are **already
normalised** (unit length).  They are stored directly in `joint_dict['axis']`
after rounding to 6 decimal places:

```python
joint_dict['axis'] = [round(v, 6) for v in joint.jointMotion.rotationAxisVector.asArray()]
```

The URDF `<axis xyz="…"/>` element expects a unit vector, so no further
normalisation is required.

---

## 7. Joint Limit Conversions

| Joint type | Fusion unit | URDF unit | Conversion |
|---|---|---|---|
| Revolute | radians | radians | × 1 |
| Prismatic | centimetres (cm) | metres (m) | ÷ 100 |

```python
# Prismatic only:
joint_dict['upper_limit'] = round(joint.jointMotion.slideLimits.maximumValue / 100, 6)
joint_dict['lower_limit'] = round(joint.jointMotion.slideLimits.minimumValue / 100, 6)
```

---

## 8. Mesh URI Format

All `<mesh>` tags in the generated URDF use the ROS `package://` URI scheme:

```xml
<mesh filename="package://{robot_name}/meshes/{link_name}.stl"
      scale="0.001 0.001 0.001"/>
```

- **`{robot_name}`** — first word of the Fusion root component name.
- **`{link_name}`** — occurrence name, sanitised (`re.sub('[ :()]', '_', name)`).
- **`scale`** — converts mm (STL native) → m (URDF standard).

Unity's URDF Importer resolves `package://` by looking for a folder named
`{robot_name}` adjacent to the `.urdf` file.  The exported directory structure
satisfies this automatically.
