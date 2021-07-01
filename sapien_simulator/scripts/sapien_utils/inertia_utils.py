import numpy as np


def compute_area(vertices):
    v1, v2, v3 = vertices
    return np.linalg.norm(np.cross(v2 - v1, v3 - v1)) / 2


def compute_cm(vertices):
    return np.mean(vertices, 0)


def compute_integral_mult(vertices, index1, index2):
    v1, v2, v3 = vertices
    x1 = v1[index1]
    x2 = v2[index1]
    x3 = v3[index1]
    y1 = v1[index2]
    y2 = v2[index2]
    y3 = v3[index2]
    return (
        x1 * y1 / 12
        + x1 * y2 / 24
        + x1 * y3 / 24
        + x2 * y1 / 24
        + x2 * y2 / 12
        + x2 * y3 / 24
        + x3 * y1 / 24
        + x3 * y2 / 24
        + x3 * y3 / 12
    )


def compute_integral_square(vertices, index):
    v1, v2, v3 = vertices
    x1 = v1[index]
    x2 = v2[index]
    x3 = v3[index]
    return (
        x1 ** 2 / 12
        + x1 * x2 / 12
        + x1 * x3 / 12
        + x2 ** 2 / 12
        + x2 * x3 / 12
        + x3 ** 2 / 12
    )


import trimesh


def compute_shell_inertial(mesh: trimesh.Trimesh):
    """Compute the mass and inertia of a given mesh, assuming the mesh is a thin
    shell with surface mass density 1
    Args:
        mesh: trimesh.Trimesh, mesh to compute mass properties
    Returns:
        mass: float, mass of the object
        inertia: shape `(3)`, principal inertia
        cm: shape `(3)` center of mass position
        rotation: rotation matrix, principal inertia directions
    """
    vertices = np.array(mesh.vertices)
    CM = []
    S = []
    for face in mesh.faces:
        v = vertices[face]
        S.append(compute_area(v))
        CM.append(compute_cm(v))
    S = np.array(S)
    CM = np.array(CM)

    mass = S.sum()
    cm = (CM * S[:, None]).sum(0) / S.sum()

    vertices = vertices - cm
    inertia = np.zeros((3, 3))
    for face, s in zip(mesh.faces, S):
        v = vertices[face]
        xy = compute_integral_mult(v, 0, 1) * s * 2
        xz = compute_integral_mult(v, 0, 2) * s * 2
        yz = compute_integral_mult(v, 1, 2) * s * 2
        xx = compute_integral_square(v, 0) * s * 2
        yy = compute_integral_square(v, 1) * s * 2
        zz = compute_integral_square(v, 2) * s * 2

        inertia[0, 0] += yy + zz
        inertia[1, 1] += xx + zz
        inertia[2, 2] += xx + yy
        inertia[1, 0] -= xy
        inertia[2, 0] -= xz
        inertia[2, 1] -= yz
        inertia[0, 1] = inertia[1, 0]
        inertia[0, 2] = inertia[2, 0]
        inertia[1, 2] = inertia[2, 1]

    inertia, rotation = np.linalg.eigh(inertia)

    if np.linalg.det(rotation) < 0:
        rotation[2] *= -1

    return mass, inertia, cm, rotation


# import numpy as np

# np.random.seed(0)
# r1 = np.random.rand(3)
# r1 /= np.linalg.norm(r1)
# r2 = np.random.rand(3)
# r3 = np.cross(r1, r2)
# r3 /= np.linalg.norm(r3)
# r2 = np.cross(r3, r1)

# R = np.array([r1, r2, r3]).T
# print(R)

# mesh = trimesh.creation.box([1, 1, 1])
# # mesh.vertices = mesh.vertices @ R.T
# print(compute_shell_inertial(mesh))

# # sphere = trimesh.creation.icosphere(4)
# # sphere.vertices = sphere.vertices + np.array([1, 2, 3])
# # print(compute_inertial(sphere))
