"""
xml_manager.py - Manages config.xml for FIC controller tuning parameters.

Handles adding new robots with default tuning values based on joint count.
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
from typing import Optional


def get_default_joint_params(joint_id: int, num_joints: int) -> dict:
    """
    Get default joint parameters based on joint position.
    Uses Franka FR3-like defaults scaled for joint position.
    
    Base joints (0-1): Higher stiffness
    Mid joints (2-3): Medium stiffness  
    Wrist joints (4+): Lower stiffness
    """
    # Base defaults (Franka-like)
    if joint_id <= 1:
        return {
            'stiffness': 200.0,
            'diff_effort_max': 100.0,
            'error_max': '0.05,0.5',
            'force_max': '10.0,20.0',
            'saturation_ratios': 0.9,
            'velocity': 1.3963,
            'planner': {
                'stiffness': 157800.0,
                'vel_desired': 3.14,
                'acc_max': 5000.0,
                'damping_ratio': 0.005,
                'frequency': 150.0,
                'wrapped': 0,
                'pos_limits': (-2.8973, 2.8973)
            },
            'admittance': {
                'InvM': 1.0,
                'Damping': 0.0,
                'dFlow': '-5.2,5.2',
                'Flow': '-2.615,2.615',
                'Pose': '-0.05,0.05'
            }
        }
    elif joint_id <= 3:
        return {
            'stiffness': 160.0,
            'diff_effort_max': 100.0,
            'error_max': '0.05,0.5',
            'force_max': '8.0,15.0',
            'saturation_ratios': 0.9,
            'velocity': 1.3963,
            'planner': {
                'stiffness': 15780.0,
                'vel_desired': 3.14,
                'acc_max': 5000.0,
                'damping_ratio': 0.005,
                'frequency': 150.0,
                'wrapped': 0,
                'pos_limits': (-2.8973, 2.8973)
            },
            'admittance': {
                'InvM': 1.0,
                'Damping': 0.0,
                'dFlow': '-5.2,5.2',
                'Flow': '-2.615,2.615',
                'Pose': '-0.05,0.05'
            }
        }
    elif joint_id == 4:
        return {
            'stiffness': 70.0,
            'diff_effort_max': 100.0,
            'error_max': '0.05,0.5',
            'force_max': '3.5,10.0',
            'saturation_ratios': 0.9,
            'velocity': 1.2218,
            'planner': {
                'stiffness': 15780.0,
                'vel_desired': 3.14,
                'acc_max': 5000.0,
                'damping_ratio': 0.005,
                'frequency': 150.0,
                'wrapped': 0,
                'pos_limits': (-2.8973, 2.8973)
            },
            'admittance': {
                'InvM': 1.0,
                'Damping': 0.0,
                'dFlow': '-5.2,5.2',
                'Flow': '-5.2,5.2',
                'Pose': '-0.05,0.05'
            }
        }
    elif joint_id == 5:
        return {
            'stiffness': 40.0,
            'diff_effort_max': 50.0,
            'error_max': '0.05,0.7',
            'force_max': '2.0,5.0',
            'saturation_ratios': 0.9,
            'velocity': 1.2218,
            'planner': {
                'stiffness': 15780.0,
                'vel_desired': 3.14,
                'acc_max': 5000.0,
                'damping_ratio': 0.005,
                'frequency': 150.0,
                'wrapped': 0,
                'pos_limits': (-0.0175, 3.7525)
            },
            'admittance': {
                'InvM': 1.0,
                'Damping': 0.0,
                'dFlow': '-5.2,5.2',
                'Flow': '-4.171,4.171',
                'Pose': '-0.05,0.05'
            }
        }
    else:  # joint 6 and beyond
        return {
            'stiffness': 15.0,
            'diff_effort_max': 50.0,
            'error_max': '0.05,0.7',
            'force_max': '0.001,5.0',
            'saturation_ratios': 0.9,
            'velocity': 1.2218,
            'planner': {
                'stiffness': 15780.0,
                'vel_desired': 3.14,
                'acc_max': 5000.0,
                'damping_ratio': 0.00005,
                'frequency': 150.0,
                'wrapped': 0,
                'pos_limits': (-2.8973, 2.8973)
            },
            'admittance': {
                'InvM': 1.0,
                'Damping': 0.0,
                'dFlow': '-5.2,5.2',
                'Flow': '-5.2,5.2',
                'Pose': '-0.05,0.05'
            }
        }


def get_default_cartesian_params() -> dict:
    """Get default Cartesian parameters (Franka-like)."""
    axis_params = {
        'stiffness': 50.0,
        'diff_effort_max': 70.0,
        'error_max': '0.05,0.1',
        'force_max': '10.0,15.0',
        'saturation_ratios': 0.9,
        'planner': {
            'stiffness_P': 15753.0,
            'vel_desired': 3.14,
            'acc_max': 5000.0,
            'damping_ratio': 0.005,
            'frequency': 150.0
        },
        'admittance': {
            'InvM': 1.0,
            'Damping': 0.0,
            'dFlow': '-5.2,5.2',
            'Flow': '-5.2,5.2',
            'Pose': '-0.05,0.05'
        }
    }
    
    q_params = {
        'stiffness': 0.001,
        'diff_effort_max': 10.0,
        'error_max': '0.05,0.5',
        'force_max': '0.1,0.3',
        'saturation_ratios': 0.9,
        'planner': {
            'stiffness_P': 15753.0,
            'vel_desired': 1.14,
            'acc_max': 5000.0,
            'damping_ratio': 0.005,
            'frequency': 150.0
        },
        'admittance': {
            'InvM': 1.0,
            'Damping': 0.0,
            'dFlow': '-5.2,5.2',
            'Flow': '-5.2,5.2',
            'Pose': '-0.05,0.05'
        }
    }
    
    return {'x': axis_params, 'y': axis_params.copy(), 'z': axis_params.copy(), 'q': q_params}


def get_default_postural_params() -> list:
    """Get default Postural parameters (Franka-like)."""
    return [
        {
            'stiffness': 1,
            'diff_effort_max': 10.0,
            'error_max': '0.001,1.5',
            'force_max': '5.,10.2',
            'saturation_ratios': 0.9,
            'velocity': 0.5,
            'planner': {
                'stiffness': 15753.0,
                'vel_desired': 3.14,
                'acc_max': 5000.0,
                'damping_ratio': 0.05,
                'frequency': 150.0,
                'wrapped': 1
            }
        },
        {
            'stiffness': 1,
            'diff_effort_max': 10.0,
            'error_max': '0.001,1.5',
            'force_max': '5.,10.2',
            'saturation_ratios': 0.9,
            'velocity': 0.5,
            'planner': {
                'stiffness': 15780.0,
                'vel_desired': 3.14,
                'acc_max': 5000.0,
                'damping_ratio': 0.005,
                'frequency': 150.0,
                'wrapped': 1,
                'pos_limits': (-2.249729, 2.249729)
            }
        }
    ]


def _create_joint_element(joint_id: int, params: dict) -> ET.Element:
    """Create a joint XML element from parameters."""
    joint = ET.Element('joint', {'id': str(joint_id)})
    
    # Basic params
    ET.SubElement(joint, 'stiffness').text = str(params['stiffness'])
    ET.SubElement(joint, 'diff_effort_max').text = str(params['diff_effort_max'])
    ET.SubElement(joint, 'error_max').text = params['error_max']
    ET.SubElement(joint, 'force_max').text = params['force_max']
    ET.SubElement(joint, 'saturation_ratios').text = str(params['saturation_ratios'])
    ET.SubElement(joint, 'velocity').text = str(params['velocity'])
    
    # Planner
    planner = ET.SubElement(joint, 'planner')
    p = params['planner']
    ET.SubElement(planner, 'stiffness').text = str(p['stiffness'])
    ET.SubElement(planner, 'vel_desired').text = str(p['vel_desired'])
    ET.SubElement(planner, 'acc_max').text = str(p['acc_max'])
    ET.SubElement(planner, 'damping_ratio').text = str(p['damping_ratio'])
    ET.SubElement(planner, 'frequency').text = str(p['frequency'])
    ET.SubElement(planner, 'wrapped').text = str(p.get('wrapped', 0))
    if 'pos_limits' in p:
        pos_lim = ET.SubElement(planner, 'pos_limits')
        pos_lim.set('unit', 'rad')
        pos_lim.set('lower', str(p['pos_limits'][0]))
        pos_lim.set('upper', str(p['pos_limits'][1]))
    
    # Admittance
    adm = ET.SubElement(joint, 'admittance')
    a = params['admittance']
    ET.SubElement(adm, 'InvM').text = str(a['InvM'])
    ET.SubElement(adm, 'Damping').text = str(a['Damping'])
    ET.SubElement(adm, 'dFlow').text = a['dFlow']
    ET.SubElement(adm, 'Flow').text = a['Flow']
    ET.SubElement(adm, 'Pose').text = a['Pose']
    
    return joint


def _create_cartesian_axis_element(axis_name: str, params: dict) -> ET.Element:
    """Create a Cartesian axis XML element."""
    axis = ET.Element(axis_name)
    
    ET.SubElement(axis, 'stiffness').text = str(params['stiffness'])
    ET.SubElement(axis, 'diff_effort_max').text = str(params['diff_effort_max'])
    ET.SubElement(axis, 'error_max').text = params['error_max']
    ET.SubElement(axis, 'force_max').text = params['force_max']
    ET.SubElement(axis, 'saturation_ratios').text = str(params['saturation_ratios'])
    
    # Planner
    planner = ET.SubElement(axis, 'planner')
    p = params['planner']
    ET.SubElement(planner, 'stiffness_P').text = str(p['stiffness_P'])
    ET.SubElement(planner, 'vel_desired').text = str(p['vel_desired'])
    ET.SubElement(planner, 'acc_max').text = str(p['acc_max'])
    ET.SubElement(planner, 'damping_ratio').text = str(p['damping_ratio'])
    ET.SubElement(planner, 'frequency').text = str(p['frequency'])
    
    # Admittance
    adm = ET.SubElement(axis, 'admittance')
    a = params['admittance']
    ET.SubElement(adm, 'InvM').text = str(a['InvM'])
    ET.SubElement(adm, 'Damping').text = str(a['Damping'])
    ET.SubElement(adm, 'dFlow').text = a['dFlow']
    ET.SubElement(adm, 'Flow').text = a['Flow']
    ET.SubElement(adm, 'Pose').text = a['Pose']
    
    return axis


def _create_postural_element(postural_id: int, params: dict) -> ET.Element:
    """Create a Postural XML element."""
    postural = ET.Element('Postural', {'id': str(postural_id)})
    
    ET.SubElement(postural, 'stiffness').text = str(params['stiffness'])
    ET.SubElement(postural, 'diff_effort_max').text = str(params['diff_effort_max'])
    ET.SubElement(postural, 'error_max').text = params['error_max']
    ET.SubElement(postural, 'force_max').text = params['force_max']
    ET.SubElement(postural, 'saturation_ratios').text = str(params['saturation_ratios'])
    ET.SubElement(postural, 'velocity').text = str(params['velocity'])
    
    # Planner
    planner = ET.SubElement(postural, 'planner')
    p = params['planner']
    ET.SubElement(planner, 'stiffness').text = str(p['stiffness'])
    ET.SubElement(planner, 'vel_desired').text = str(p['vel_desired'])
    ET.SubElement(planner, 'acc_max').text = str(p['acc_max'])
    ET.SubElement(planner, 'damping_ratio').text = str(p['damping_ratio'])
    ET.SubElement(planner, 'frequency').text = str(p['frequency'])
    ET.SubElement(planner, 'wrapped').text = str(p.get('wrapped', 1))
    if 'pos_limits' in p:
        pos_lim = ET.SubElement(planner, 'pos_limits')
        pos_lim.set('unit', 'rad')
        pos_lim.set('lower', str(p['pos_limits'][0]))
        pos_lim.set('upper', str(p['pos_limits'][1]))
    
    return postural


def prettify_xml(elem: ET.Element) -> str:
    """Return a pretty-printed XML string."""
    rough_string = ET.tostring(elem, encoding='unicode')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="    ")


def add_robot_to_config_xml(config_path: str, robot_name: str, num_joints: int) -> bool:
    """
    Add a new robot to config.xml if it doesn't exist.

    Delegates to the main XMLManager so the new entry follows the current
    <mode id="N" profile="default"> schema.

    Args:
        config_path: Path to config.xml
        robot_name: Name of the robot to add
        num_joints: Number of joints

    Returns:
        bool: True if robot was added, False if it already exists
    """
    # Bootstrap empty config if it does not exist — XMLManager expects a file.
    if not os.path.exists(config_path):
        root = ET.Element('robots')
        tree = ET.ElementTree(root)
        os.makedirs(os.path.dirname(config_path) or '.', exist_ok=True)
        xml_str = prettify_xml(root)
        lines = xml_str.split('\n')
        lines[0] = "<?xml version='1.0' encoding='utf-8'?>"
        from fic_studio.xml_manager import write_atomic
        write_atomic(config_path, '\n'.join(lines))

    from fic_studio.xml_manager import XMLManager  # local import avoids cycles
    mgr = XMLManager(config_path)
    if robot_name in mgr.get_available_robots():
        return False
    mgr.ensure_robot(robot_name, num_joints)  # ensure_robot saves to disk
    return True


def get_robots_in_config_xml(config_path: str) -> list:
    """
    Get list of robot names in config.xml.
    
    Args:
        config_path: Path to config.xml
        
    Returns:
        list: List of robot names
    """
    if not os.path.exists(config_path):
        return []
    
    try:
        tree = ET.parse(config_path)
        root = tree.getroot()
        return [robot.get('name') for robot in root.findall('robot')]
    except Exception:
        return []


def sync_robots_to_config_xml(config_path: str, robot_configs: dict) -> dict:
    """
    Sync robot configurations to config.xml.
    Adds missing robots with defaults, does NOT remove robots not in robot_configs.
    
    Args:
        config_path: Path to config.xml
        robot_configs: Dict of robot configs from GUI {robot_id: config_dict}
        
    Returns:
        dict: {'added': [...], 'existing': [...]}
    """
    result = {'added': [], 'existing': []}
    
    existing_robots = get_robots_in_config_xml(config_path)
    
    for robot_id, config in robot_configs.items():
        num_joints = config.get('num_joints', 7)
        
        if robot_id in existing_robots:
            result['existing'].append(robot_id)
        else:
            if add_robot_to_config_xml(config_path, robot_id, num_joints):
                result['added'].append(robot_id)
    
    return result