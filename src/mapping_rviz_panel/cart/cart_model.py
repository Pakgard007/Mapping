"""A golf cart the size of the real one, as a COLLADA mesh and a one-link URDF.

mapping_panel.launch.py builds it from cart.yaml every time RViz is opened.
The URDF has a single link, lidar_link, which is the frame LIO-SAM moves: RViz
draws the cart straight from the file, so there is no robot_state_publisher
and nothing is added to TF.

The cart is modelled in its own frame: origin on the ground under the middle
of the rear axle, x forward, y left, z up.
"""
import math
import os

import numpy as np

# RViz lights the scene from the camera only, so a white roof seen from above
# comes out grey. GLOW is the share of its colour a part shows whatever the light.
GLOW = {'body': 0.35, 'roof': 0.5, 'metal': 0.2, 'lamp': 0.8, 'tail': 0.6}
COLORS = {
    'body': (0.93, 0.94, 0.95, 1.0),
    'roof': (0.96, 0.96, 0.94, 1.0),
    'seat': (0.17, 0.18, 0.20, 1.0),
    'trim': (0.07, 0.075, 0.085, 1.0),
    'tire': (0.035, 0.035, 0.04, 1.0),
    'metal': (0.72, 0.74, 0.77, 1.0),
    'glass': (0.62, 0.80, 0.92, 0.28),
    'lamp': (1.0, 0.95, 0.75, 1.0),
    'tail': (0.85, 0.12, 0.10, 1.0),
}


def _outward(tris):
    """A closed surface whose triangles all wind the same way, facing out."""
    volume = np.einsum('ij,ij->i', tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum()
    return tris if volume > 0 else tris[:, ::-1]


def _loft(rings):
    """The surface through rings of equal point count, capped at both ends."""
    rings = np.asarray(rings, dtype=float)
    a, b = rings[:-1], rings[1:]
    a2, b2 = np.roll(a, -1, axis=1), np.roll(b, -1, axis=1)
    parts = [np.stack([a, a2, b2], axis=2).reshape(-1, 3, 3),
             np.stack([a, b2, b], axis=2).reshape(-1, 3, 3)]
    for ring, first in ((rings[0], True), (rings[-1], False)):
        fan = np.stack(
            [np.broadcast_to(ring.mean(axis=0), ring.shape), ring, np.roll(ring, -1, axis=0)],
            axis=1)
        parts.append(fan[:, ::-1] if first else fan)
    return _outward(np.concatenate(parts))


def _rounded_rect(hv, w0, w1, r, n):
    """Outline (v, w) of the rectangle |v| <= hv, w0 <= w <= w1, corners of radius r."""
    r = max(min(r, hv, (w1 - w0) / 2), 1e-4)
    corners = ((hv - r, w1 - r, 0.0), (r - hv, w1 - r, 0.5), (r - hv, w0 + r, 1.0), (hv - r, w0 + r, 1.5))
    arcs = []
    for cv, cw, start in corners:
        t = (start + np.linspace(0.0, 0.5, n + 1)) * math.pi
        arcs.append(np.stack([cv + r * np.cos(t), cw + r * np.sin(t)], axis=1))
    return np.concatenate(arcs)


def _place(axis, a, v, w):
    """Points of a cross-section (v, w) at station a along the axis."""
    a = np.full_like(v, a)
    return np.stack({'x': (a, v, w), 'y': (v, a, w), 'z': (w, v, a)}[axis], axis=1)


def _block(axis, a0, a1, hv, w0, w1, r=0.03, r_end=None, n=3, v=0.0):
    """A box with rounded edges: the cross-section (v, w) swept from a0 to a1.

    For axis x the section is (y, z), for y it is (x, z), for z it is (y, x);
    r rounds the edges along the axis, r_end the ones around its two ends.
    """
    r_end = r if r_end is None else r_end
    r_end = min(r_end, (a1 - a0) / 2, hv, (w1 - w0) / 2)
    if r_end > 1e-4:
        t = np.linspace(0.0, math.pi / 2, n + 1)
        along = np.concatenate([a0 + r_end * (1 - np.cos(t)), a1 - r_end * (1 - np.cos(t[::-1]))])
        inset = np.concatenate([r_end * (1 - np.sin(t)), r_end * (1 - np.sin(t[::-1]))])
    else:
        along, inset = np.array([a0, a1]), np.zeros(2)
    rings = []
    for a, d in zip(along, inset):
        ring = _rounded_rect(hv - d, w0 + d, w1 - d, r - d, n)
        rings.append(_place(axis, a, ring[:, 0] + v, ring[:, 1]))
    return _loft(rings)


def _basis(u):
    u = np.asarray(u, dtype=float)
    u = u / np.linalg.norm(u)
    e = np.cross(u, (0.0, 0.0, 1.0))
    if np.linalg.norm(e) < 1e-6:
        e = np.cross(u, (1.0, 0.0, 0.0))
    e = e / np.linalg.norm(e)
    return u, e, np.cross(u, e)


def _tube(p0, p1, radius, n=12):
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    _, e, f = _basis(p1 - p0)
    t = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    circle = radius * (np.outer(np.cos(t), e) + np.outer(np.sin(t), f))
    return _loft([p0 + circle, p1 + circle])


def _revolve(profile, center, axis, n=32):
    """A closed outline of (radius, along-axis) points turned around the axis."""
    u, e, f = _basis(axis)
    rings = np.array([
        np.asarray(center, dtype=float) +
        np.outer(profile[:, 0], math.cos(t) * e + math.sin(t) * f) + np.outer(profile[:, 1], u)
        for t in np.linspace(0.0, 2 * math.pi, n, endpoint=False)])
    a, b = rings, np.roll(rings, -1, axis=0)
    a2, b2 = np.roll(a, -1, axis=1), np.roll(b, -1, axis=1)
    return _outward(np.concatenate([
        np.stack([a, a2, b2], axis=2).reshape(-1, 3, 3),
        np.stack([a, b2, b], axis=2).reshape(-1, 3, 3)]))


def _tilt(tris, degrees, pivot):
    """Turned about the y axis through the pivot; positive leans the top forward."""
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    rotation = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    pivot = np.asarray(pivot, dtype=float)
    return (tris - pivot) @ rotation.T + pivot


def _cowl(x0, x1, hw0, hw1, z0, top0, top1, nose_r, n=14):
    """The front body: full width over the wheels, sloping down to a rounded nose."""
    rings = []
    for x in np.concatenate([np.linspace(x0, x1 - nose_r, n, endpoint=False),
                             x1 - nose_r * (1 - np.sin(np.linspace(0.0, math.pi / 2, n)))]):
        t = (x - x0) / (x1 - x0)
        u = max(0.0, (x - (x1 - nose_r)) / nose_r)
        d = nose_r * (1 - math.sqrt(max(0.0, 1 - u * u)))
        ring = _rounded_rect(
            hw0 + (hw1 - hw0) * t - 0.9 * d, z0 + 0.1 * d,
            top0 + (top1 - top0) * t ** 1.5 - 0.35 * d, 0.10, 4)
        rings.append(_place('x', x, ring[:, 0], ring[:, 1]))
    return _loft(rings)


def build_parts(dims):
    """{material: [triangles]} of the cart in its own frame."""
    wb = float(dims['wheelbase'])
    nose = wb + float(dims['front_overhang'])
    tail = nose - float(dims['length'])
    tire_r = float(dims['tire_diameter']) / 2
    tire_w = float(dims['tire_width'])
    wheel_y = float(dims['width']) / 2 - tire_w / 2
    hb = wheel_y + tire_w / 2 - 0.06          # body half-width: the tyres stand a little proud
    inner = wheel_y - tire_w / 2 - 0.035      # half-width of what sits between the wheels
    lidar_x = float(dims['lidar']['ahead_of_rear_axle'])
    lidar_z = float(dims['lidar']['height'])
    sensor_base = lidar_z - 0.038             # os_lidar is 38 mm above the foot of an OS1
    roof_top = sensor_base - 0.006            # the sensor's plate lies on it, a little proud
    roof_bottom = roof_top - 0.05
    roof_front = float(dims['roof_front'])
    roof_rear = roof_front - float(dims['roof_length'])
    fender = 2 * tire_r + 0.045               # underside of the body over the wheels
    deck = fender + 0.16                      # top of the body tub the seats sit on

    parts = {name: [] for name in COLORS}

    # Wheels: tyre, rim and hub cap.
    tyre = _rounded_rect(tire_w / 2, tire_r * 0.47, tire_r, 0.05, 4)[:, ::-1]
    for x in (0.0, wb):
        for side in (1.0, -1.0):
            y = side * wheel_y
            parts['tire'].append(_revolve(tyre, (x, y, tire_r), (0, 1, 0)))
            parts['metal'].append(_tube((x, y - 0.075, tire_r), (x, y + 0.075, tire_r), tire_r * 0.48, 20))
            parts['trim'].append(
                _tube((x, y + side * 0.07, tire_r), (x, y + side * 0.092, tire_r), tire_r * 0.2, 12))

    # Body: nose and cowl, floor, and the tub under the seats.
    parts['body'] += [
        _cowl(wb - 0.50, nose - 0.03, hb, hb - 0.05, fender, deck + 0.27, deck + 0.02, 0.22),
        _block('x', wb - 0.45, nose - 0.07, inner, 0.19, fender + 0.02, 0.06),
        _block('x', 0.50, wb - 0.41, hb, 0.19, 0.30, 0.03),
        _block('x', tail + 0.23, 0.62, hb, fender, deck, 0.05),
        _block('x', tail + 0.30, 0.62, inner, 0.19, fender + 0.02, 0.05),
    ]
    parts['trim'] += [
        _block('x', nose - 0.12, nose, inner + 0.06, 0.26, 0.36, 0.04),            # front bumper
        _block('x', 0.55, wb - 0.45, hb - 0.09, 0.30, 0.312, 0.005),               # floor mat
        _block('x', tail, tail + 0.28, hb - 0.09, 0.30, 0.34, 0.015),              # rear footrest
        _block('x', wb - 0.57, wb - 0.47, hb - 0.03, deck + 0.10, deck + 0.255, 0.03),  # dashboard
    ]
    for side in (1.0, -1.0):
        parts['lamp'].append(_block(
            'x', nose - 0.05, nose - 0.02, 0.07, deck - 0.105, deck - 0.055, 0.012, v=side * 0.24))
        parts['tail'].append(_block(
            'x', tail + 0.215, tail + 0.24, 0.06, fender + 0.04, fender + 0.12, 0.012,
            v=side * (hb - 0.12)))
        parts['trim'].append(_tube(                                                 # footrest stay
            (tail + 0.02, side * (hb - 0.12), 0.32), (tail + 0.32, side * (inner - 0.03), 0.30), 0.014, 8))

    # Seats: a bench facing forward and one facing back, sharing the middle.
    parts['seat'] += [
        _block('x', 0.07, 0.60, hb - 0.04, deck, deck + 0.12, 0.045),
        _tilt(_block('x', 0.02, 0.11, hb - 0.04, deck + 0.14, deck + 0.56, 0.04), -8, (0.065, 0, deck + 0.14)),
        _block('x', tail + 0.26, -0.11, hb - 0.04, deck, deck + 0.12, 0.045),
        _tilt(_block('x', -0.19, -0.10, hb - 0.04, deck + 0.14, deck + 0.52, 0.04), 8, (-0.145, 0, deck + 0.14)),
    ]
    for side in (1.0, -1.0):                                                        # hip rails
        y = side * (hb - 0.02)
        for x0, x1 in ((0.10, 0.52), (tail + 0.30, -0.14)):
            parts['trim'] += [
                _tube((x0, y, deck + 0.02), (x0, y, deck + 0.24), 0.012, 8),
                _tube((x0, y, deck + 0.24), (x1, y, deck + 0.24), 0.012, 8),
                _tube((x1, y, deck + 0.24), (x1, y, deck + 0.02), 0.012, 8),
            ]

    # Steering wheel, on the left.
    wheel_at = np.array((wb - 0.75, 0.28, deck + 0.34))
    column = np.array((wb - 0.54, 0.28, deck + 0.20))
    spin = wheel_at - column
    t = np.linspace(0.0, 2 * math.pi, 8, endpoint=False)
    rim = np.stack([0.17 + 0.015 * np.cos(t), 0.015 * np.sin(t)], axis=1)
    parts['trim'] += [
        _tube(column, wheel_at, 0.018, 10),
        _revolve(rim, wheel_at, spin, 28),
        _tube(wheel_at - 0.02 * spin / np.linalg.norm(spin), wheel_at, 0.045, 12),
    ]
    _, e, f = _basis(spin)
    for direction in (e, -0.5 * e + 0.87 * f, -0.5 * e - 0.87 * f):
        parts['trim'].append(_tube(wheel_at, wheel_at + 0.17 * direction, 0.009, 6))

    # Roof on four struts, with a windscreen between the front pair.
    strut_foot = np.array((wb - 0.49, hb - 0.05, deck + 0.24))
    strut_top = np.array((wb - 0.60, hb - 0.07, roof_bottom + 0.01))
    rear_foot = np.array((tail + 0.29, hb - 0.04, deck))
    rear_top = np.array((max(roof_rear + 0.10, tail + 0.23), hb - 0.07, roof_bottom + 0.01))
    mirror = np.array((1.0, -1.0, 1.0))
    for flip in (np.ones(3), mirror):
        parts['trim'] += [
            _tube(strut_foot * flip, strut_top * flip, 0.017, 10),
            _tube(rear_foot * flip, rear_top * flip, 0.017, 10),
        ]
    lean = math.degrees(math.atan2(strut_foot[0] - strut_top[0], strut_top[2] - strut_foot[2]))
    height = float(np.linalg.norm(strut_top - strut_foot)) - 0.04
    foot = (strut_foot[0], 0.0, strut_foot[2] + 0.02)
    parts['glass'].append(_tilt(_block(
        'x', foot[0] - 0.004, foot[0] + 0.004, hb - 0.08, foot[2], foot[2] + height, 0.003), -lean, foot))
    for share in (0.0, 0.55, 1.0):                                                  # its frame
        bar = strut_foot + share * (strut_top - strut_foot)
        parts['trim'].append(_tube(bar, bar * mirror, 0.011, 8))
    parts['roof'].append(_block('z', roof_bottom, roof_top, hb - 0.02, roof_rear, roof_front, 0.16, 0.02, 6))

    # The lidar where it really is, on a plate off the front edge of the roof.
    parts['trim'] += [
        _block('x', min(roof_front - 0.10, lidar_x - 0.07), lidar_x + 0.07, 0.07, sensor_base - 0.014, sensor_base, 0.005),
        _tube((lidar_x, 0, sensor_base + 0.018), (lidar_x, 0, sensor_base + 0.056), 0.040, 24),
    ]
    parts['metal'] += [
        _tube((lidar_x, 0, sensor_base), (lidar_x, 0, sensor_base + 0.018), 0.0435, 24),
        _tube((lidar_x, 0, sensor_base + 0.056), (lidar_x, 0, sensor_base + 0.0735), 0.0435, 24),
    ]
    return parts


def cart_in_lidar_link(dims):
    """(xyz, rpy) of the cart's frame in lidar_link, as a URDF origin."""
    lidar = dims['lidar']
    roll, pitch = math.radians(float(lidar['roll_deg'])), math.radians(float(lidar['pitch_deg']))
    yaw = {'+x': 0.0, 'x': 0.0, '-x': math.pi}[str(lidar['forward']).strip()]

    def rot(axis, angle):
        c, s = math.cos(angle), math.sin(angle)
        i, j = {'x': (1, 2), 'y': (2, 0), 'z': (0, 1)}[axis]
        m = np.eye(3)
        m[i, i], m[i, j], m[j, i], m[j, j] = c, -s, s, c
        return m

    # A level cart heading where lidar_link's forward axis points, seen from a
    # lidar_link that is itself rolled and pitched by the mount.
    rotation = rot('x', -roll) @ rot('y', -pitch) @ rot('z', yaw)
    position = rotation @ -np.array((float(lidar['ahead_of_rear_axle']), 0.0, float(lidar['height'])))
    rpy = (math.atan2(rotation[2, 1], rotation[2, 2]), -math.asin(rotation[2, 0]),
           math.atan2(rotation[1, 0], rotation[0, 0]))
    return position, rpy, rotation


def _smooth_normals(tris, crease_degrees=50.0):
    """A normal for every corner: its faces averaged, except across sharp edges."""
    edges = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    area = np.linalg.norm(edges, axis=1)
    tris, edges, area = tris[area > 1e-12], edges[area > 1e-12], area[area > 1e-12]
    face = edges / area[:, None]
    _, vertex = np.unique(np.round(tris.reshape(-1, 3) * 1e4).astype(np.int64), axis=0, return_inverse=True)
    vertex = vertex.reshape(-1)
    order = np.argsort(vertex, kind='stable')
    starts = np.flatnonzero(np.r_[True, np.diff(vertex[order]) != 0, True])
    normals = np.repeat(face, 3, axis=0)
    limit = math.cos(math.radians(crease_degrees))
    for a, b in zip(starts[:-1], starts[1:]):
        corners = order[a:b]                      # the corners that meet at one point
        if len(corners) == 1:
            continue
        f = face[corners // 3]
        together = (f @ f.T) > limit
        summed = (together * area[corners // 3]) @ f
        normals[corners] = summed / np.linalg.norm(summed, axis=1, keepdims=True)
    return tris, normals.reshape(-1, 3, 3)


def _floats(values):
    return ' '.join('%.5g' % v for v in np.asarray(values, dtype=float).ravel())


def _write_dae(path, parts):
    """One COLLADA file, a geometry per colour. RViz gives an STL no colour of
    its own choosing: it shows the importer's white."""
    effects, materials, geometries, nodes = [], [], [], []
    for name, tris in parts.items():
        if not tris:
            continue
        tris, normals = _smooth_normals(np.concatenate(tris))
        r, g, b, a = COLORS[name]
        glass = (
            '<transparent opaque="A_ONE"><color>1 1 1 1</color></transparent>'
            '<transparency><float>%g</float></transparency>' % a) if a < 1 else ''
        effects.append(
            '<effect id="%s-fx"><profile_COMMON><technique sid="common"><phong>'
            '<emission><color>%g %g %g 1</color></emission>'
            '<ambient><color>%g %g %g 1</color></ambient><diffuse><color>%g %g %g 1</color></diffuse>'
            '<specular><color>0.08 0.08 0.08 1</color></specular><shininess><float>12</float></shininess>'
            '%s</phong></technique></profile_COMMON></effect>' % (
                name, *(GLOW.get(name, 0.0) * c for c in (r, g, b)), r, g, b, r, g, b, glass))
        materials.append(
            '<material id="%s-mat" name="%s"><instance_effect url="#%s-fx"/></material>' % (name, name, name))
        source = (
            '<source id="{n}-{k}"><float_array id="{n}-{k}-a" count="{c}">{v}</float_array><technique_common>'
            '<accessor source="#{n}-{k}-a" count="{m}" stride="3"><param name="X" type="float"/>'
            '<param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>')
        count = len(tris) * 3
        geometries.append(
            '<geometry id="%s-geo" name="%s"><mesh>%s%s<vertices id="%s-vtx">'
            '<input semantic="POSITION" source="#%s-pos"/></vertices>'
            '<triangles count="%d" material="%s-mat"><input semantic="VERTEX" source="#%s-vtx" offset="0"/>'
            '<input semantic="NORMAL" source="#%s-nrm" offset="0"/><p>%s</p></triangles></mesh></geometry>' % (
                name, name,
                source.format(n=name, k='pos', c=count * 3, m=count, v=_floats(tris)),
                source.format(n=name, k='nrm', c=count * 3, m=count, v=_floats(normals)),
                name, name, len(tris), name, name, name, ' '.join(map(str, range(count)))))
        nodes.append(
            '<instance_geometry url="#%s-geo"><bind_material><technique_common>'
            '<instance_material symbol="%s-mat" target="#%s-mat"/></technique_common></bind_material>'
            '</instance_geometry>' % (name, name, name))
    with open(path, 'w', encoding='utf-8') as f:
        f.write(
            '<?xml version="1.0" encoding="utf-8"?>\n'
            '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">\n'
            '<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>\n'
            '<library_effects>%s</library_effects>\n<library_materials>%s</library_materials>\n'
            '<library_geometries>%s</library_geometries>\n'
            '<library_visual_scenes><visual_scene id="scene"><node id="cart" name="cart">%s</node>'
            '</visual_scene></library_visual_scenes>\n'
            '<scene><instance_visual_scene url="#scene"/></scene>\n</COLLADA>\n' % (
                '\n'.join(effects), '\n'.join(materials), '\n'.join(geometries), '\n'.join(nodes)))


def build(dims, out_dir, link='lidar_link'):
    """Writes the mesh and the URDF into out_dir; returns the URDF's path."""
    os.makedirs(out_dir, exist_ok=True)
    mesh = os.path.join(out_dir, 'golf_cart.dae')
    _write_dae(mesh, build_parts(dims))
    xyz, rpy, _ = cart_in_lidar_link(dims)
    urdf = os.path.join(out_dir, 'golf_cart.urdf')
    with open(urdf, 'w', encoding='utf-8') as f:
        # No <material>: with one, RViz would paint the whole cart in it.
        f.write(
            '<?xml version="1.0"?>\n<robot name="golf_cart">\n  <link name="%s">\n    <visual>\n'
            '      <origin xyz="%.4f %.4f %.4f" rpy="%.5f %.5f %.5f"/>\n'
            '      <geometry><mesh filename="file://%s"/></geometry>\n'
            '    </visual>\n  </link>\n</robot>\n' % (link, *xyz, *rpy, mesh))
    return urdf
