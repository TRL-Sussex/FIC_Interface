#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import threading
import rclpy
from PyQt5.QtWidgets import QApplication

# Local imports
from fic_studio.config import XML_FILE_PATH
from fic_studio.ros_node import FicStudioNode
from fic_studio.main_window import RobotSelectionWindow
import xml.etree.ElementTree as ET

def check_xml_on_startup():
    print(f"XML file path: {XML_FILE_PATH}")
    try:
        tree = ET.parse(XML_FILE_PATH)
        root = tree.getroot()
        print("XML loaded successfully!")
    except FileNotFoundError:
        print(f"File not found: {XML_FILE_PATH}")

def main(args=None):
    check_xml_on_startup()
    
    rclpy.init(args=args)
    node = FicStudioNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = RobotSelectionWindow(node)
    window.show()
    ret = app.exec_()

    # cleanup
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    sys.exit(ret)

if __name__ == "__main__":
    main()