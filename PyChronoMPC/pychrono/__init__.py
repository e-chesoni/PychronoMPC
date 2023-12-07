# PyChrono conda package.
# The following imports add the binaries to the modules.
# Because of the SWIG 4 import style (from . import _<modulename>), pyd/so files in the 
# PYTHONPATH dir cannot be found and have to be added to the package.

import os
import pathlib

# Cache directory of this script.
idir = pathlib.Path(__file__).parent.resolve()

try:
    import _core
    import _fea
except:
    pass
try:
    import _cascade
except:
    pass
try:
    import _irrlicht
except:
    pass
try:
    import _vehicle
except:
    pass
try:
    import _postprocess
except:
    pass
try:
    import _pardisomkl
except:
    pass
try:
    import _sensor
    sdir = os.path.join(idir, '../../../Library/lib/sensor_ptx')
    sdir = os.path.join(os.path.realpath(sdir), '')
    _sensor.SetSensorShaderDir(sdir)
except:
    pass
try:
    import _robot
except:
    pass

# The following allows the package "pychrono" to be directly
# used as C++ namespace chrono:: ie. you just need to type
#   import pychrono
# instead of 
#   import pychrono.core	
from .core import *

# Infer path to Chrono data/ directory from path of __init__.py
ddir = os.path.join(idir, '../../../Library/data')
ddir = os.path.join(os.path.realpath(ddir), '')
SetChronoDataPath(ddir)
