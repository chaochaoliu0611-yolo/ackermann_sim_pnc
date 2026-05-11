#!/usr/bin/env python3
"""
sdf_to_urdf.py — Convert SAYE model.sdf to URDF for robot_state_publisher / RViz.

Usage (from launch file):
    from sdf_to_urdf import sdf_to_urdf_string
    urdf_str = sdf_to_urdf_string(sdf_str)
"""

import xml.etree.ElementTree as ET


def _text(el, default=""):
    return (el.text or "").strip() if el is not None else default


def _pose_to_origin(pose_text):
    """'x y z r p y' → ('x y z', 'r p y')"""
    parts = pose_text.strip().split()
    if len(parts) >= 6:
        x, y, z, r, p, yaw = parts[:6]
        return f"{x} {y} {z}", f"{r} {p} {yaw}"
    elif len(parts) >= 3:
        return f"{parts[0]} {parts[1]} {parts[2]}", "0 0 0"
    return "0 0 0", "0 0 0"


def _elem_to_str(elem):
    """Serialize an Element to XML string (without XML declaration)."""
    return ET.tostring(elem, encoding='unicode', method='xml')


def _geo_to_urdf(geo_elem):
    """Convert SDF <geometry> to URDF <geometry> string."""
    if geo_elem is None:
        return ""
    mesh = geo_elem.find('mesh')
    if mesh is not None:
        uri = mesh.find('uri')
        if uri is not None and uri.text and uri.text.strip():
            filename = uri.text.strip()
            return f'<geometry>\n        <mesh filename="{filename}"/>\n      </geometry>'
        return ""
    box = geo_elem.find('box')
    if box is not None:
        size = box.find('size')
        if size is not None and size.text:
            return f'<geometry>\n        <box size="{size.text.strip()}"/>\n      </geometry>'
        return ""
    cyl = geo_elem.find('cylinder')
    if cyl is not None:
        radius = cyl.find('radius')
        length = cyl.find('length')
        r = _text(radius, "0")
        l = _text(length, "0")
        return f'<geometry>\n        <cylinder radius="{r}" length="{l}"/>\n      </geometry>'
    return ""


def sdf_to_urdf_string(sdf_str):
    """Convert SDF XML string to URDF XML string for robot_state_publisher."""

    sdf_root = ET.fromstring(sdf_str)
    model = sdf_root.find('model')
    if model is None:
        raise ValueError("No <model> element in SDF")

    # ── rename map ──────────────────────────────────────────────
    rename = {'base_link': 'saye'}

    # ── Collect links & joints ───────────────────────────────────
    links = {}
    joints = []
    for child in model:
        tag = child.tag
        if tag == 'link':
            name = child.get('name', '')
            links[name] = child
        elif tag == 'joint':
            joints.append(child)

    def rlink(name):
        """Rename link name (base_link → saye)."""
        return rename.get(name, name)

    # ── Build URDF parts ─────────────────────────────────────────
    parts = ['<?xml version="1.0"?>']
    parts.append('<robot name="saye">')

    for orig_name, link_elem in links.items():
        name = rlink(orig_name)
        parts.append(f'  <link name="{name}">')

        # inertial
        inertial = link_elem.find('inertial')
        if inertial is not None:
            parts.append('    <inertial>')
            pose = inertial.find('pose')
            if pose is not None:
                xyz, rpy = _pose_to_origin(_text(pose))
                parts.append(f'      <origin xyz="{xyz}" rpy="{rpy}"/>')
            mass = inertial.find('mass')
            if mass is not None:
                parts.append(f'      <mass value="{_text(mass, "1.0")}"/>')
            inertia = inertial.find('inertia')
            if inertia is not None:
                vals = {}
                for a in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']:
                    e = inertia.find(a)
                    vals[a] = _text(e, '0') if e is not None else '0'
                parts.append(f'      <inertia ixx="{vals["ixx"]}" ixy="{vals["ixy"]}" ixz="{vals["ixz"]}" iyy="{vals["iyy"]}" iyz="{vals["iyz"]}" izz="{vals["izz"]}"/>')
            parts.append('    </inertial>')

        # visuals
        for vis in link_elem.findall('visual'):
            vname = vis.get('name', '')
            parts.append(f'    <visual name="{vname}">')
            pose = vis.find('pose')
            if pose is not None:
                xyz, rpy = _pose_to_origin(_text(pose))
                parts.append(f'      <origin xyz="{xyz}" rpy="{rpy}"/>')
            geo = vis.find('geometry')
            if geo is not None:
                parts.append(f'      {_geo_to_urdf(geo).strip()}')
            parts.append('    </visual>')

        # collisions
        for col in link_elem.findall('collision'):
            cname = col.get('name', '')
            parts.append(f'    <collision name="{cname}">')
            pose = col.find('pose')
            if pose is not None:
                xyz, rpy = _pose_to_origin(_text(pose))
                parts.append(f'      <origin xyz="{xyz}" rpy="{rpy}"/>')
            geo = col.find('geometry')
            if geo is not None:
                parts.append(f'      {_geo_to_urdf(geo).strip()}')
            parts.append('    </collision>')

        parts.append(f'  </link>')

    # joints
    for jnt in joints:
        jname = jnt.get('name', '')
        jtype = jnt.get('type', 'fixed')
        parts.append(f'  <joint name="{jname}" type="{jtype}">')

        parent = jnt.find('parent')
        child = jnt.find('child')
        if parent is not None:
            parts.append(f'    <parent link="{rlink(_text(parent))}"/>')
        if child is not None:
            parts.append(f'    <child link="{rlink(_text(child))}"/>')

        # origin from pose
        pose = jnt.find('pose')
        pose_text = _text(pose, '0 0 0 0 0 0') if pose is not None else '0 0 0 0 0 0'
        xyz, rpy = _pose_to_origin(pose_text)
        parts.append(f'    <origin xyz="{xyz}" rpy="{rpy}"/>')

        # axis
        axis = jnt.find('axis')
        if axis is not None:
            xyz_e = axis.find('xyz')
            if xyz_e is not None:
                parts.append(f'    <axis xyz="{_text(xyz_e)}"/>')
            limit_e = axis.find('limit')
            if limit_e is not None:
                lo = _text(limit_e.find('lower'), '0')
                hi = _text(limit_e.find('upper'), '0')
                parts.append(f'    <limit lower="{lo}" upper="{hi}" effort="0" velocity="0"/>')

        parts.append(f'  </joint>')

    parts.append('</robot>')
    return '\n'.join(parts)
