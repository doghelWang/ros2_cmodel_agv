#!/usr/bin/env python3
import os
import sys
import json
import zipfile
import tempfile
import math
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
BACKEND_PATH = REPO_ROOT / "src" / "backend"
if BACKEND_PATH.exists() and str(BACKEND_PATH) not in sys.path:
    sys.path.insert(0, str(BACKEND_PATH))

try:
    from app.infrastructure.protobuf.cmodel_decoder import decode_cmodel
except ImportError:
    decode_cmodel = None


def parse_cmodel_file(cmodel_path: str) -> dict:
    cmodel_path = os.path.abspath(cmodel_path)
    if not os.path.exists(cmodel_path):
        raise FileNotFoundError(f"cmodel file not found: {cmodel_path}")

    with tempfile.TemporaryDirectory() as tmpdir:
        if decode_cmodel:
            decode_cmodel(cmodel_path, tmpdir)
            comp_path = os.path.join(tmpdir, "CompDesc.json")
        else:
            with zipfile.ZipFile(cmodel_path, "r") as z:
                z.extractall(tmpdir)
            comp_path = os.path.join(tmpdir, "CompDesc.json")

        if not os.path.exists(comp_path):
            raise FileNotFoundError("CompDesc.json not found in decoded model")

        with open(comp_path, "r", encoding="utf-8") as f:
            comp_desc = json.load(f)

    return extract_robot_spec(comp_desc, os.path.basename(cmodel_path))


def extract_robot_spec(comp_desc: dict, model_filename: str) -> dict:
    spec = {
        "model_file": model_filename,
        "robot_name": "cmodel_agv",
        "chassis": {
            "type": "diff",
            "length_m": 1.2,
            "width_m": 0.8,
            "height_m": 0.35,
            "head_offset_m": 0.6,
            "tail_offset_m": 0.6,
            "left_offset_m": 0.4,
            "right_offset_m": 0.4,
            "max_speed_mps": 1.5,
            "max_accel_mps2": 1.0,
            "max_ang_speed_radps": 2.0,
        },
        "drive_wheels": {
            "radius_m": 0.1,
            "track_width_m": 0.6,
            "wheel_width_m": 0.05,
            "x_offset_m": 0.0,
        },
        "lidars": [],
        "imu": {"x": 0.0, "y": 0.0, "z": 0.1}
    }

    modules = comp_desc.get("moreModuleInfo", [])
    for g in modules:
        comps = g.get("moduleComponets") or g.get("module_componets") or []
        for c in comps:
            gen = c.get("generalAttr", {})
            mname = gen.get("moduleName", {}).get("stringValue", "")
            mtype = gen.get("mainModuleType", {}).get("comboType", {}).get("typeKey", "")

            # Chassis
            if mtype == "chassis" or "chassis" in mname.lower():
                pa_list = (c.get("privateAttr") or c.get("privateAttrs") or {}).get("privateAttrs", [])
                for pa in pa_list:
                    k = pa.get("key", "")
                    if k == "motionCenterAttr":
                        for ele in pa.get("arrayBaseEle", []):
                            ek = ele.get("key")
                            ev = ele.get("doubleValue")
                            if ev is not None and ev > 0:
                                if "headOffset" in ek:
                                    spec["chassis"]["head_offset_m"] = ev / 1000.0
                                elif "tailOffset" in ek:
                                    spec["chassis"]["tail_offset_m"] = ev / 1000.0
                                elif "leftOffset" in ek:
                                    spec["chassis"]["left_offset_m"] = ev / 1000.0
                                elif "rightOffset" in ek:
                                    spec["chassis"]["right_offset_m"] = ev / 1000.0
                    elif k == "chassisAttr":
                        for ele in pa.get("arrayBaseEle", []):
                            ek = ele.get("key")
                            ev = ele.get("doubleValue")
                            if ev is not None:
                                if "maxSpeed" in ek:
                                    spec["chassis"]["max_speed_mps"] = ev / 1000.0
                                elif "maxAcceleration" in ek:
                                    spec["chassis"]["max_accel_mps2"] = ev / 1000.0
                                elif "rotateMaxAngSpeed" in ek:
                                    spec["chassis"]["max_ang_speed_radps"] = math.radians(ev)

                spec["chassis"]["length_m"] = spec["chassis"]["head_offset_m"] + spec["chassis"]["tail_offset_m"]
                spec["chassis"]["width_m"] = spec["chassis"]["left_offset_m"] + spec["chassis"]["right_offset_m"]

            # Drive wheel
            elif mtype == "driveWheel" or "wheel" in mname.lower():
                pa_list = (c.get("privateAttr") or c.get("privateAttrs") or {}).get("privateAttrs", [])
                for pa in pa_list:
                    if pa.get("key") == "wheelAttr":
                        for ele in pa.get("arrayBaseEle", []):
                            if ele.get("key") == "wheelRadius" and ele.get("doubleValue"):
                                spec["drive_wheels"]["radius_m"] = ele.get("doubleValue") / 1000.0

                sp_list = (c.get("structParam") or {}).get("extendParams", [])
                for p in sp_list:
                    if p.get("key") == "locCoordX" and p.get("doubleValue"):
                        spec["drive_wheels"]["x_offset_m"] = p.get("doubleValue") / 1000.0

            # Lidar
            elif "laser" in mname.lower() or "lidar" in mname.lower():
                lidar_info = {
                    "name": mname,
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.3,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "min_range": 0.05,
                    "max_range": 25.0,
                    "fov_deg": 270.0
                }
                sp_list = (c.get("structParam") or {}).get("extendParams", [])
                for p in sp_list:
                    k = p.get("key")
                    v = p.get("doubleValue")
                    if v is not None:
                        if k == "locCoordX":
                            lidar_info["x"] = v / 1000.0
                        elif k == "locCoordY":
                            lidar_info["y"] = v / 1000.0
                        elif k == "locCoordZ":
                            lidar_info["z"] = v / 1000.0
                        elif k == "locCoordROLL":
                            lidar_info["roll"] = math.radians(v)
                        elif k == "locCoordPITCH":
                            lidar_info["pitch"] = math.radians(v)
                        elif k == "locCoordYAW":
                            lidar_info["yaw"] = math.radians(v)

                spec["lidars"].append(lidar_info)

    if not spec["lidars"]:
        spec["lidars"].append({
            "name": "laser_front",
            "x": spec["chassis"]["head_offset_m"] * 0.9,
            "y": 0.0,
            "z": 0.25,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "min_range": 0.05,
            "max_range": 25.0,
            "fov_deg": 270.0
        })

    spec["drive_wheels"]["track_width_m"] = spec["chassis"]["width_m"] * 0.8
    return spec


def generate_urdf(spec: dict) -> str:
    chassis = spec["chassis"]
    wheels = spec["drive_wheels"]
    length = chassis["length_m"]
    width = chassis["width_m"]
    height = chassis["height_m"]
    r_wheel = wheels["radius_m"]
    w_wheel = wheels["wheel_width_m"]
    track = wheels["track_width_m"]
    x_body = (chassis["head_offset_m"] - chassis["tail_offset_m"]) / 2.0
    z_body = r_wheel + height / 2.0

    xml = []
    xml.append('<?xml version="1.0"?>')
    xml.append(f'<robot name="{spec["robot_name"]}">')
    xml.append('  <link name="base_footprint"/>')
    xml.append('  <link name="base_link">')
    xml.append('    <visual>')
    xml.append(f'      <origin xyz="{x_body:.4f} 0 {z_body:.4f}" rpy="0 0 0"/>')
    xml.append('      <geometry>')
    xml.append(f'        <box size="{length:.4f} {width:.4f} {height:.4f}"/>')
    xml.append('      </geometry>')
    xml.append('      <material name="industrial_orange">')
    xml.append('        <color rgba="0.95 0.5 0.1 1.0"/>')
    xml.append('      </material>')
    xml.append('    </visual>')
    xml.append('    <collision>')
    xml.append(f'      <origin xyz="{x_body:.4f} 0 {z_body:.4f}" rpy="0 0 0"/>')
    xml.append('      <geometry>')
    xml.append(f'        <box size="{length:.4f} {width:.4f} {height:.4f}"/>')
    xml.append('      </geometry>')
    xml.append('    </collision>')
    xml.append('    <inertial>')
    xml.append(f'      <origin xyz="{x_body:.4f} 0 {z_body:.4f}" rpy="0 0 0"/>')
    xml.append('      <mass value="60.0"/>')
    xml.append('      <inertia ixx="1.5" ixy="0.0" ixz="0.0" iyy="2.5" iyz="0.0" izz="3.0"/>')
    xml.append('    </inertial>')
    xml.append('  </link>')
    xml.append('  <joint name="base_footprint_joint" type="fixed">')
    xml.append('    <parent link="base_footprint"/>')
    xml.append('    <child link="base_link"/>')
    xml.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
    xml.append('  </joint>')

    # Left Wheel
    xml.append('  <link name="left_wheel_link">')
    xml.append('    <visual>')
    xml.append('      <origin xyz="0 0 0" rpy="1.570796 0 0"/>')
    xml.append('      <geometry>')
    xml.append(f'        <cylinder radius="{r_wheel:.4f}" length="{w_wheel:.4f}"/>')
    xml.append('      </geometry>')
    xml.append('      <material name="dark_grey">')
    xml.append('        <color rgba="0.2 0.2 0.2 1.0"/>')
    xml.append('      </material>')
    xml.append('    </visual>')
    xml.append('  </link>')
    xml.append('  <joint name="left_wheel_joint" type="continuous">')
    xml.append('    <parent link="base_link"/>')
    xml.append('    <child link="left_wheel_link"/>')
    xml.append(f'    <origin xyz="0 {track/2:.4f} {r_wheel:.4f}" rpy="0 0 0"/>')
    xml.append('    <axis xyz="0 1 0"/>')
    xml.append('  </joint>')

    # Right Wheel
    xml.append('  <link name="right_wheel_link">')
    xml.append('    <visual>')
    xml.append('      <origin xyz="0 0 0" rpy="1.570796 0 0"/>')
    xml.append('      <geometry>')
    xml.append(f'        <cylinder radius="{r_wheel:.4f}" length="{w_wheel:.4f}"/>')
    xml.append('      </geometry>')
    xml.append('      <material name="dark_grey">')
    xml.append('        <color rgba="0.2 0.2 0.2 1.0"/>')
    xml.append('      </material>')
    xml.append('    </visual>')
    xml.append('  </link>')
    xml.append('  <joint name="right_wheel_joint" type="continuous">')
    xml.append('    <parent link="base_link"/>')
    xml.append('    <child link="right_wheel_link"/>')
    xml.append(f'    <origin xyz="0 {-track/2:.4f} {r_wheel:.4f}" rpy="0 0 0"/>')
    xml.append('    <axis xyz="0 1 0"/>')
    xml.append('  </joint>')

    # IMU
    xml.append('  <link name="imu_link"/>')
    xml.append('  <joint name="imu_joint" type="fixed">')
    xml.append('    <parent link="base_link"/>')
    xml.append('    <child link="imu_link"/>')
    xml.append(f'    <origin xyz="{spec["imu"]["x"]} {spec["imu"]["y"]} {spec["imu"]["z"]}" rpy="0 0 0"/>')
    xml.append('  </joint>')

    # Lidars
    for idx, lidar in enumerate(spec["lidars"]):
        lname = lidar["name"].replace(" ", "_").replace("-", "_")
        xml.append(f'  <link name="{lname}_link">')
        xml.append('    <visual>')
        xml.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        xml.append('      <geometry>')
        xml.append('        <cylinder radius="0.04" length="0.06"/>')
        xml.append('      </geometry>')
        xml.append('      <material name="cyan">')
        xml.append('        <color rgba="0.0 0.85 0.95 1.0"/>')
        xml.append('      </material>')
        xml.append('    </visual>')
        xml.append('  </link>')
        xml.append(f'  <joint name="{lname}_joint" type="fixed">')
        xml.append('    <parent link="base_link"/>')
        xml.append(f'    <child link="{lname}_link"/>')
        xml.append(f'    <origin xyz="{lidar["x"]:.4f} {lidar["y"]:.4f} {lidar["z"]:.4f}" rpy="{lidar["roll"]:.4f} {lidar["pitch"]:.4f} {lidar["yaw"]:.4f}"/>')
        xml.append('  </joint>')

    xml.append('</robot>')
    return '\n'.join(xml) + '\n'


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 cmodel_parser.py <path_to_cmodel> [output_dir]")
        sys.exit(1)

    cmodel_path = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(__file__))
    os.makedirs(out_dir, exist_ok=True)

    spec = parse_cmodel_file(cmodel_path)
    urdf = generate_urdf(spec)

    spec_path = os.path.join(out_dir, "robot_config.json")
    urdf_path = os.path.join(out_dir, "robot.urdf")

    with open(spec_path, "w", encoding="utf-8") as f:
        json.dump(spec, f, indent=2, ensure_ascii=False)
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(urdf)

    print(f"Successfully generated:")
    print(f"  - Config: {spec_path}")
    print(f"  - URDF:   {urdf_path}")
