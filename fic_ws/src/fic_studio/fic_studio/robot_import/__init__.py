"""Import a robot from its URDF: joints, meshes and a MuJoCo model, ready to simulate.

``importer.analyse()`` works out what a URDF (or a folder / zip holding one)
contains; ``importer.create_robot()`` writes it into ``menagerie/model/<ID>/``.
The Studio's Robot Configuration page drives both from its drop zone.
"""
