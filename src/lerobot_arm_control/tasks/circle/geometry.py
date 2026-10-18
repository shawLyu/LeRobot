"""Validated circle-plane coordinates, independent of hardware and NumPy."""

import math


def circle_frame(plane, basis=None):
    if plane in ('XY', 'XZ'):
        if basis is not None:
            raise ValueError('axis-aligned circle must not override its plane basis')
        basis = [[1., 0., 0.], [0., 1., 0.] if plane == 'XY' else [0., 0., 1.]]
    elif plane != 'inclined':
        raise ValueError('unknown circle plane')
    if (not isinstance(basis, (list, tuple)) or len(basis) != 2
            or any(not isinstance(row, (list, tuple)) or len(row) != 3
                   or any(type(v) not in (int, float) or not math.isfinite(v) for v in row)
                   for row in basis)):
        raise ValueError('invalid circle plane basis')
    u, v = (tuple(row) for row in basis)
    if (abs(sum(x*x for x in u)-1) > 1e-9 or abs(sum(x*x for x in v)-1) > 1e-9
            or abs(sum(a*b for a, b in zip(u, v))) > 1e-9):
        raise ValueError('circle plane basis must be orthonormal')
    normal = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
    return u, v, normal
