import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


CONFIG = Path(__file__).parents[2] / "agibot_x2_moveit_config" / "config"
XACRO = CONFIG / "x2_ultra.urdf.xacro"


def expand(**mappings):
    command = ["xacro", str(XACRO)]
    command.extend(f"{name}:={value}" for name, value in mappings.items())
    return ET.fromstring(subprocess.check_output(command, text=True))


def test_real_control_description_has_31_states_and_14_arm_commands():
    root = expand(use_fake_hardware="false", command_transport="ros_topic")
    control = root.find("ros2_control")
    assert control is not None
    assert control.findtext("hardware/plugin") == "agibot_x2_ros2_control/X2SystemHardware"
    joints = control.findall("joint")
    assert len(joints) == 31
    assert all({item.get("name") for item in joint.findall("state_interface")} ==
               {"position", "velocity", "effort"} for joint in joints)
    commanded = [joint for joint in joints if joint.findall("command_interface")]
    assert len(commanded) == 14
    assert all(joint.find("command_interface").get("name") == "position" for joint in commanded)


def test_fake_default_and_locomanipulation_gains():
    root = expand()
    control = root.find("ros2_control")
    assert control.findtext("hardware/plugin") == "mock_components/GenericSystem"
    joints = {joint.get("name"): joint for joint in control.findall("joint")}
    shoulder = joints["left_shoulder_pitch_joint"]
    wrist = joints["right_wrist_roll_joint"]
    shoulder_parameters = {
        item.get("name"): float(item.text) for item in shoulder.findall("param")
    }
    wrist_parameters = {item.get("name"): float(item.text) for item in wrist.findall("param")}
    assert shoulder_parameters == {"stiffness": 50.0, "damping": 3.0}
    assert wrist_parameters == {"stiffness": 20.0, "damping": 2.0}


def test_zmq_transport_is_selected_exclusively():
    root = expand(use_fake_hardware="false", command_transport="zmq")
    parameters = {
        item.get("name"): item.text for item in root.findall("ros2_control/hardware/param")
    }
    assert parameters["command_transport"] == "zmq"
    assert parameters["zmq_endpoint"] == "tcp://*:8559"
