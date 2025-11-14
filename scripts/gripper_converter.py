import xml.etree.ElementTree as ET
import re

"""
    Converts the URDF model of the Rover2025 gripper into MJCF XML format for RoboSuite.
    Be sure to modify INPUT_URDF and OUTPUT_XML as needed.
"""
INPUT_URDF = "Arm 2024-2025/Arm URDF/mujoco/End-Test/urdf/End-Test.urdf"
OUTPUT_XML = "gripper.xml"

# ---------------------------------------------------------------------------

def parse_urdf(path):
    tree = ET.parse(path)
    return tree.getroot()

def extract_inertial(link):
    inert = link.find("inertial")
    origin = inert.find("origin")
    mass = inert.find("mass")
    inertia = inert.find("inertia")

    pos = origin.attrib.get("xyz", "0 0 0")
    m = mass.attrib.get("value", "1")
    diag = " ".join([
        inertia.attrib.get("ixx", "0"),
        inertia.attrib.get("iyy", "0"),
        inertia.attrib.get("izz", "0"),
    ])

    return pos, m, diag

def build_mjcf(urdf):

    mj = ET.Element("mujoco", attrib={"model": "rover2025_gripper_model"})

    # ------------------------------------------------------------------
    # <default>
    d = ET.SubElement(mj, "default")
    ET.SubElement(d, "joint", attrib={"limited": "false", "damping": "0.01", "armature": "0.01", "frictionloss": "0.01"})
    ET.SubElement(d, "geom", attrib={"condim": "4", "contype": "1", "conaffinity": "1"})

    d_pris = ET.SubElement(d, "default", attrib={"class": "prismatic"})
    ET.SubElement(d_pris, "joint", attrib={"type": "slide"})

    d_col = ET.SubElement(d, "default", attrib={"class": "collision"})
    ET.SubElement(d_col, "geom", attrib={"group": "0", "rgba": "0.7 0.7 0.7 1", "type": "mesh"})

    d_vis = ET.SubElement(d, "default", attrib={"class": "visual"})
    ET.SubElement(d_vis, "geom", attrib={"group": "1", "material": "default_material", "type": "mesh"})

    # ------------------------------------------------------------------
    mj.append(ET.Comment(" compiler set to radians and use rpy with euler "))
    ET.SubElement(mj, "compiler", attrib={"angle": "radian", "eulerseq": "xyz"})

    # ------------------------------------------------------------------
    mj.append(ET.Comment(" mesh names "))
    asset = ET.SubElement(mj, "asset")
    ET.SubElement(asset, "material", attrib={"name": "default_visual", "rgba": "0.7 0.7 0.7 1"})
    ET.SubElement(asset, "material", attrib={"name": "default_material", "rgba": "0.7 0.7 0.7 1"})

    # Mesh names are normalized according to your manually created XML
    ET.SubElement(asset, "mesh", attrib={"name": "base_ee", "file": "meshes/rover2025_gripper/Base-EE.stl"})
    ET.SubElement(asset, "mesh", attrib={"name": "gripper1", "file": "meshes/rover2025_gripper/Gripper-1.stl"})
    ET.SubElement(asset, "mesh", attrib={"name": "gripper2", "file": "meshes/rover2025_gripper/Gripper-2.stl"})

    # ------------------------------------------------------------------
    tendon = ET.SubElement(mj, "tendon")
    fixed = ET.SubElement(tendon, "fixed", attrib={"name": "gripper_coupling"})
    ET.SubElement(fixed, "joint", attrib={"joint": "gripper1_joint", "coef": "1"})
    ET.SubElement(fixed, "joint", attrib={"joint": "gripper2_joint", "coef": "-1"})

    # ------------------------------------------------------------------
    mj.append(ET.Comment(" actuator torques "))
    actuator = ET.SubElement(mj, "actuator")
    ET.SubElement(actuator, "motor", attrib={
        "name": "grip_motor",
        "joint": "gripper1_joint",
        "gear": "1",
    })

    # ------------------------------------------------------------------
    world = ET.SubElement(mj, "worldbody")

    # Base link (Base-EE)
    base = urdf.find(".//link[@name='Base-EE']")
    base_pos = "0 0 0"
    base_rpy = "0 0 0"

    body_base = ET.SubElement(world, "body",
        attrib={"name": "base_ee", "pos": base_pos, "euler": base_rpy})

    # Sites
    ET.SubElement(body_base, "site", attrib={
        "name": "ft_frame", "pos": "0 0 0",
        "size": "0.01 0.01 0.01", "rgba": "1 0 0 1",
        "type": "sphere", "group": "1"
    })

    # eef body
    eef = ET.SubElement(body_base, "body", attrib={
        "name": "eef",
        "pos": "0 0 0.145",
        "quat": "0.707105 0 0 -0.707105"
    })

    # eef sites
    ET.SubElement(eef, "site", attrib={
        "name": "grip_site", "pos": "0 0 0",
        "size": "0.01 0.01 0.01", "rgba": "1 0 0 0.5",
        "type": "sphere", "group": "1"
    })
    ET.SubElement(eef, "site", attrib={
        "name": "ee_x", "pos": "0.1 0 0",
        "size": "0.005 .1",
        "quat": "0.707105 0 0.707108 0",
        "rgba": "1 0 0 0",
        "type": "cylinder", "group": "1"
    })
    ET.SubElement(eef, "site", attrib={
        "name": "ee_y", "pos": "0 0.1 0",
        "size": "0.005 .1",
        "quat": "0.707105 0.707108 0 0",
        "rgba": "0 1 0 0",
        "type": "cylinder", "group": "1"
    })
    ET.SubElement(eef, "site", attrib={
        "name": "ee_z", "pos": "0 0 0.1",
        "size": "0.005 .1",
        "quat": "1 0 0 0",
        "rgba": "0 0 1 0",
        "type": "cylinder", "group": "1"
    })
    ET.SubElement(eef, "site", attrib={
        "name": "grip_site_cylinder",
        "pos": "0 0 0",
        "size": "0.005 10",
        "rgba": "0 1 0 0.3",
        "type": "cylinder", "group": "1"
    })

    # Base geoms + inertia
    ET.SubElement(body_base, "geom", attrib={"class": "collision", "mesh": "base_ee"})
    ET.SubElement(body_base, "geom", attrib={"class": "visual", "mesh": "base_ee"})

    pos, mass, diag = extract_inertial(base)
    ET.SubElement(body_base, "inertial", attrib={
        "pos": pos,
        "mass": mass,
        "diaginertia": diag
    })

    # ------------------------------------------------------------------
    # Gripper-1
    g1_link = urdf.find(".//link[@name='Gripper-1']")
    g1 = ET.SubElement(body_base, "body",
                       attrib={"name": "gripper1", "pos": "0 0 0", "quat": "1 0 0 0"})
    ET.SubElement(g1, "joint", attrib={
        "name": "gripper1_joint",
        "class": "prismatic",
        "axis": "0 1 0",
        "range": "-0.2 0.2"
    })
    ET.SubElement(g1, "geom", attrib={"class": "collision", "mesh": "gripper1"})
    ET.SubElement(g1, "geom", attrib={"class": "visual", "mesh": "gripper1"})

    g1_pos, g1_m, g1_diag = extract_inertial(g1_link)
    ET.SubElement(g1, "inertial", attrib={
        "pos": g1_pos, "mass": g1_m, "diaginertia": g1_diag
    })

    # ------------------------------------------------------------------
    # Gripper-2
    g2_link = urdf.find(".//link[@name='Gripper-2']")
    g2 = ET.SubElement(body_base, "body",
                       attrib={"name": "gripper2", "pos": "0 0 0", "quat": "1 0 0 0"})
    ET.SubElement(g2, "joint", attrib={
        "name": "gripper2_joint",
        "class": "prismatic",
        "axis": "0 1 0",
        "range": "-0.2 0.2"
    })
    ET.SubElement(g2, "geom", attrib={"class": "collision", "mesh": "gripper2"})
    ET.SubElement(g2, "geom", attrib={"class": "visual", "mesh": "gripper2"})

    # Reuse same inertial as Gripper-1 because your XML duplicates it
    ET.SubElement(g2, "inertial", attrib={
        "pos": g1_pos, "mass": g1_m, "diaginertia": g1_diag
    })

    # ------------------------------------------------------------------
    # Sensors
    sensor = ET.SubElement(mj, "sensor")
    ET.SubElement(sensor, "force", attrib={"name": "force_ee", "site": "ft_frame"})
    ET.SubElement(sensor, "torque", attrib={"name": "torque_ee", "site": "ft_frame"})

    return mj

# ---------------------------------------------------------------------------

def indent(elem, level=0):
    i = "\n" + "    " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "    "
        for c in elem:
            indent(c, level + 1)
            if not c.tail or not c.tail.strip():
                c.tail = i + "    "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if not elem.tail or not elem.tail.strip():
            elem.tail = i

def format_output(root):
    indent(root)
    xml_body = ET.tostring(root, encoding="unicode")

    # inertial multi-line formatting
    pattern = re.compile(
        r'<inertial\s+pos="([^"]+)"\s+mass="([^"]+)"\s+diaginertia="([^"]+)"\s*/>'
    )
    xml_body = pattern.sub(
        lambda m: (
            "\n            <inertial\n"
            f'                pos="{m.group(1)}"\n'
            f'                mass="{m.group(2)}"\n'
            f'                diaginertia="{m.group(3)}"\n'
            "            />"
        ),
        xml_body
    )

    return "<!-- Automatically Generated using /scripts/gripper_conv.py -->\n" + xml_body.strip() + "\n"

# ---------------------------------------------------------------------------

def urdf_to_mjcf():
    urdf = parse_urdf(INPUT_URDF)
    mj = build_mjcf(urdf)
    out = format_output(mj)

    with open(OUTPUT_XML, "w") as f:
        f.write(out)

    print(f"Wrote {OUTPUT_XML}")

# ---------------------------------------------------------------------------

if __name__ == "__main__":
    urdf_to_mjcf()
