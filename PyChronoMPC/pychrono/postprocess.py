# This file was automatically generated by SWIG (http://www.swig.org).
# Version 4.0.2
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

from sys import version_info as _swig_python_version_info
if _swig_python_version_info < (2, 7, 0):
    raise RuntimeError("Python 2.7 or later required")

# Import the low-level C/C++ module
if __package__ or "." in __name__:
    from . import _postprocess
else:
    import _postprocess

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)


def _swig_setattr_nondynamic_instance_variable(set):
    def set_instance_attr(self, name, value):
        if name == "thisown":
            self.this.own(value)
        elif name == "this":
            set(self, name, value)
        elif hasattr(self, name) and isinstance(getattr(type(self), name), property):
            set(self, name, value)
        else:
            raise AttributeError("You cannot add instance attributes to %s" % self)
    return set_instance_attr


def _swig_setattr_nondynamic_class_variable(set):
    def set_class_attr(cls, name, value):
        if hasattr(cls, name) and not isinstance(getattr(cls, name), property):
            set(cls, name, value)
        else:
            raise AttributeError("You cannot add class attributes to %s" % cls)
    return set_class_attr


def _swig_add_metaclass(metaclass):
    """Class decorator for adding a metaclass to a SWIG wrapped class - a slimmed down version of six.add_metaclass"""
    def wrapper(cls):
        return metaclass(cls.__name__, cls.__bases__, cls.__dict__.copy())
    return wrapper


class _SwigNonDynamicMeta(type):
    """Meta class to enforce nondynamic attributes (no new attributes) for a class"""
    __setattr__ = _swig_setattr_nondynamic_class_variable(type.__setattr__)


import weakref

SHARED_PTR_DISOWN = _postprocess.SHARED_PTR_DISOWN

class SwigPyIterator(object):
    r"""Proxy of C++ swig::SwigPyIterator class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _postprocess.delete_SwigPyIterator

    def value(self):
        r"""value(SwigPyIterator self) -> PyObject *"""
        return _postprocess.SwigPyIterator_value(self)

    def incr(self, n=1):
        r"""incr(SwigPyIterator self, size_t n=1) -> SwigPyIterator"""
        return _postprocess.SwigPyIterator_incr(self, n)

    def decr(self, n=1):
        r"""decr(SwigPyIterator self, size_t n=1) -> SwigPyIterator"""
        return _postprocess.SwigPyIterator_decr(self, n)

    def distance(self, x):
        r"""distance(SwigPyIterator self, SwigPyIterator x) -> ptrdiff_t"""
        return _postprocess.SwigPyIterator_distance(self, x)

    def equal(self, x):
        r"""equal(SwigPyIterator self, SwigPyIterator x) -> bool"""
        return _postprocess.SwigPyIterator_equal(self, x)

    def copy(self):
        r"""copy(SwigPyIterator self) -> SwigPyIterator"""
        return _postprocess.SwigPyIterator_copy(self)

    def next(self):
        r"""next(SwigPyIterator self) -> PyObject *"""
        return _postprocess.SwigPyIterator_next(self)

    def __next__(self):
        r"""__next__(SwigPyIterator self) -> PyObject *"""
        return _postprocess.SwigPyIterator___next__(self)

    def previous(self):
        r"""previous(SwigPyIterator self) -> PyObject *"""
        return _postprocess.SwigPyIterator_previous(self)

    def advance(self, n):
        r"""advance(SwigPyIterator self, ptrdiff_t n) -> SwigPyIterator"""
        return _postprocess.SwigPyIterator_advance(self, n)

    def __eq__(self, x):
        r"""__eq__(SwigPyIterator self, SwigPyIterator x) -> bool"""
        return _postprocess.SwigPyIterator___eq__(self, x)

    def __ne__(self, x):
        r"""__ne__(SwigPyIterator self, SwigPyIterator x) -> bool"""
        return _postprocess.SwigPyIterator___ne__(self, x)

    def __iadd__(self, n):
        r"""__iadd__(SwigPyIterator self, ptrdiff_t n) -> SwigPyIterator"""
        return _postprocess.SwigPyIterator___iadd__(self, n)

    def __isub__(self, n):
        r"""__isub__(SwigPyIterator self, ptrdiff_t n) -> SwigPyIterator"""
        return _postprocess.SwigPyIterator___isub__(self, n)

    def __add__(self, n):
        r"""__add__(SwigPyIterator self, ptrdiff_t n) -> SwigPyIterator"""
        return _postprocess.SwigPyIterator___add__(self, n)

    def __sub__(self, *args):
        r"""
        __sub__(SwigPyIterator self, ptrdiff_t n) -> SwigPyIterator
        __sub__(SwigPyIterator self, SwigPyIterator x) -> ptrdiff_t
        """
        return _postprocess.SwigPyIterator___sub__(self, *args)
    def __iter__(self):
        return self

# Register SwigPyIterator in _postprocess:
_postprocess.SwigPyIterator_swigregister(SwigPyIterator)

class int_ptr(object):
    r"""Proxy of C++ int_ptr class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self):
        r"""__init__(int_ptr self) -> int_ptr"""
        _postprocess.int_ptr_swiginit(self, _postprocess.new_int_ptr())
    __swig_destroy__ = _postprocess.delete_int_ptr

    def assign(self, value):
        r"""assign(int_ptr self, int value)"""
        return _postprocess.int_ptr_assign(self, value)

    def value(self):
        r"""value(int_ptr self) -> int"""
        return _postprocess.int_ptr_value(self)

    def cast(self):
        r"""cast(int_ptr self) -> int *"""
        return _postprocess.int_ptr_cast(self)

    @staticmethod
    def frompointer(t):
        r"""frompointer(int * t) -> int_ptr"""
        return _postprocess.int_ptr_frompointer(t)

# Register int_ptr in _postprocess:
_postprocess.int_ptr_swigregister(int_ptr)

def int_ptr_frompointer(t):
    r"""int_ptr_frompointer(int * t) -> int_ptr"""
    return _postprocess.int_ptr_frompointer(t)

class double_ptr(object):
    r"""Proxy of C++ double_ptr class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self):
        r"""__init__(double_ptr self) -> double_ptr"""
        _postprocess.double_ptr_swiginit(self, _postprocess.new_double_ptr())
    __swig_destroy__ = _postprocess.delete_double_ptr

    def assign(self, value):
        r"""assign(double_ptr self, double value)"""
        return _postprocess.double_ptr_assign(self, value)

    def value(self):
        r"""value(double_ptr self) -> double"""
        return _postprocess.double_ptr_value(self)

    def cast(self):
        r"""cast(double_ptr self) -> double *"""
        return _postprocess.double_ptr_cast(self)

    @staticmethod
    def frompointer(t):
        r"""frompointer(double * t) -> double_ptr"""
        return _postprocess.double_ptr_frompointer(t)

# Register double_ptr in _postprocess:
_postprocess.double_ptr_swigregister(double_ptr)

def double_ptr_frompointer(t):
    r"""double_ptr_frompointer(double * t) -> double_ptr"""
    return _postprocess.double_ptr_frompointer(t)

class float_ptr(object):
    r"""Proxy of C++ float_ptr class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self):
        r"""__init__(float_ptr self) -> float_ptr"""
        _postprocess.float_ptr_swiginit(self, _postprocess.new_float_ptr())
    __swig_destroy__ = _postprocess.delete_float_ptr

    def assign(self, value):
        r"""assign(float_ptr self, float value)"""
        return _postprocess.float_ptr_assign(self, value)

    def value(self):
        r"""value(float_ptr self) -> float"""
        return _postprocess.float_ptr_value(self)

    def cast(self):
        r"""cast(float_ptr self) -> float *"""
        return _postprocess.float_ptr_cast(self)

    @staticmethod
    def frompointer(t):
        r"""frompointer(float * t) -> float_ptr"""
        return _postprocess.float_ptr_frompointer(t)

# Register float_ptr in _postprocess:
_postprocess.float_ptr_swigregister(float_ptr)

def float_ptr_frompointer(t):
    r"""float_ptr_frompointer(float * t) -> float_ptr"""
    return _postprocess.float_ptr_frompointer(t)

import pychrono.core
class ChPostProcessBase(object):
    r"""Proxy of C++ chrono::postprocess::ChPostProcessBase class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _postprocess.delete_ChPostProcessBase

    def SetSystem(self, system):
        r"""SetSystem(ChPostProcessBase self, ChSystem system)"""
        return _postprocess.ChPostProcessBase_SetSystem(self, system)

    def GetSystem(self):
        r"""GetSystem(ChPostProcessBase self) -> ChSystem"""
        return _postprocess.ChPostProcessBase_GetSystem(self)

    def ExportScript(self, filename):
        r"""ExportScript(ChPostProcessBase self, std::string const & filename)"""
        return _postprocess.ChPostProcessBase_ExportScript(self, filename)

    def ExportData(self, filename):
        r"""ExportData(ChPostProcessBase self, std::string const & filename)"""
        return _postprocess.ChPostProcessBase_ExportData(self, filename)

# Register ChPostProcessBase in _postprocess:
_postprocess.ChPostProcessBase_swigregister(ChPostProcessBase)

class ChPovRay(ChPostProcessBase):
    r"""Proxy of C++ chrono::postprocess::ChPovRay class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self, system):
        r"""__init__(ChPovRay self, ChSystem system) -> ChPovRay"""
        _postprocess.ChPovRay_swiginit(self, _postprocess.new_ChPovRay(system))
    __swig_destroy__ = _postprocess.delete_ChPovRay
    ContactSymbol_VECTOR_SCALELENGTH = _postprocess.ChPovRay_ContactSymbol_VECTOR_SCALELENGTH
    
    ContactSymbol_VECTOR_SCALERADIUS = _postprocess.ChPovRay_ContactSymbol_VECTOR_SCALERADIUS
    
    ContactSymbol_VECTOR_NOSCALE = _postprocess.ChPovRay_ContactSymbol_VECTOR_NOSCALE
    
    ContactSymbol_SPHERE_SCALERADIUS = _postprocess.ChPovRay_ContactSymbol_SPHERE_SCALERADIUS
    
    ContactSymbol_SPHERE_NOSCALE = _postprocess.ChPovRay_ContactSymbol_SPHERE_NOSCALE
    

    def Add(self, item):
        r"""Add(ChPovRay self, std::shared_ptr< chrono::ChPhysicsItem > item)"""
        return _postprocess.ChPovRay_Add(self, item)

    def Remove(self, item):
        r"""Remove(ChPovRay self, std::shared_ptr< chrono::ChPhysicsItem > item)"""
        return _postprocess.ChPovRay_Remove(self, item)

    def AddAll(self):
        r"""AddAll(ChPovRay self)"""
        return _postprocess.ChPovRay_AddAll(self)

    def RemoveAll(self):
        r"""RemoveAll(ChPovRay self)"""
        return _postprocess.ChPovRay_RemoveAll(self)

    def SetCustomCommands(self, item, commands):
        r"""SetCustomCommands(ChPovRay self, std::shared_ptr< chrono::ChPhysicsItem > item, std::string const & commands)"""
        return _postprocess.ChPovRay_SetCustomCommands(self, item, commands)

    def SetBasePath(self, mpath):
        r"""SetBasePath(ChPovRay self, std::string const & mpath)"""
        return _postprocess.ChPovRay_SetBasePath(self, mpath)

    def SetTemplateFile(self, filename):
        r"""SetTemplateFile(ChPovRay self, std::string const & filename)"""
        return _postprocess.ChPovRay_SetTemplateFile(self, filename)

    def SetOutputScriptFile(self, filename):
        r"""SetOutputScriptFile(ChPovRay self, std::string const & filename)"""
        return _postprocess.ChPovRay_SetOutputScriptFile(self, filename)

    def SetPictureFilebase(self, filename):
        r"""SetPictureFilebase(ChPovRay self, std::string const & filename)"""
        return _postprocess.ChPovRay_SetPictureFilebase(self, filename)

    def SetOutputDataFilebase(self, filename):
        r"""SetOutputDataFilebase(ChPovRay self, std::string const & filename)"""
        return _postprocess.ChPovRay_SetOutputDataFilebase(self, filename)

    def SetPictureSize(self, width, height):
        r"""SetPictureSize(ChPovRay self, unsigned int width, unsigned int height)"""
        return _postprocess.ChPovRay_SetPictureSize(self, width, height)

    def SetAntialiasing(self, active, depth, treshold):
        r"""SetAntialiasing(ChPovRay self, bool active, unsigned int depth, double treshold)"""
        return _postprocess.ChPovRay_SetAntialiasing(self, active, depth, treshold)

    def SetCamera(self, location, aim, angle, ortho=False):
        r"""SetCamera(ChPovRay self, chrono::ChVector< > location, chrono::ChVector< > aim, double angle, bool ortho=False)"""
        return _postprocess.ChPovRay_SetCamera(self, location, aim, angle, ortho)

    def SetLight(self, location, color, cast_shadow):
        r"""SetLight(ChPovRay self, chrono::ChVector< > location, ChColor color, bool cast_shadow)"""
        return _postprocess.ChPovRay_SetLight(self, location, color, cast_shadow)

    def SetBackground(self, color):
        r"""SetBackground(ChPovRay self, ChColor color)"""
        return _postprocess.ChPovRay_SetBackground(self, color)

    def SetAmbientLight(self, color):
        r"""SetAmbientLight(ChPovRay self, ChColor color)"""
        return _postprocess.ChPovRay_SetAmbientLight(self, color)

    def SetShowCOGs(self, show, msize=0.04):
        r"""SetShowCOGs(ChPovRay self, bool show, double msize=0.04)"""
        return _postprocess.ChPovRay_SetShowCOGs(self, show, msize)

    def SetShowFrames(self, show, msize=0.05):
        r"""SetShowFrames(ChPovRay self, bool show, double msize=0.05)"""
        return _postprocess.ChPovRay_SetShowFrames(self, show, msize)

    def SetShowLinks(self, show, msize=0.04):
        r"""SetShowLinks(ChPovRay self, bool show, double msize=0.04)"""
        return _postprocess.ChPovRay_SetShowLinks(self, show, msize)

    def SetShowContacts(self, show, mode, scale, width, max_size, do_colormap, colormap_start, colormap_end):
        r"""SetShowContacts(ChPovRay self, bool show, chrono::postprocess::ChPovRay::ContactSymbol mode, double scale, double width, double max_size, bool do_colormap, double colormap_start, double colormap_end)"""
        return _postprocess.ChPovRay_SetShowContacts(self, show, mode, scale, width, max_size, do_colormap, colormap_start, colormap_end)

    def SetWireframeThickness(self, wft):
        r"""SetWireframeThickness(ChPovRay self, double const wft)"""
        return _postprocess.ChPovRay_SetWireframeThickness(self, wft)

    def GetWireframeThickness(self):
        r"""GetWireframeThickness(ChPovRay self) -> double"""
        return _postprocess.ChPovRay_GetWireframeThickness(self)

    def SetCustomPOVcommandsScript(self, text):
        r"""SetCustomPOVcommandsScript(ChPovRay self, std::string const & text)"""
        return _postprocess.ChPovRay_SetCustomPOVcommandsScript(self, text)

    def GetCustomPOVcommandsScript(self):
        r"""GetCustomPOVcommandsScript(ChPovRay self) -> std::string const &"""
        return _postprocess.ChPovRay_GetCustomPOVcommandsScript(self)

    def SetCustomPOVcommandsData(self, text):
        r"""SetCustomPOVcommandsData(ChPovRay self, std::string const & text)"""
        return _postprocess.ChPovRay_SetCustomPOVcommandsData(self, text)

    def GetCustomPOVcommandsData(self):
        r"""GetCustomPOVcommandsData(ChPovRay self) -> std::string const &"""
        return _postprocess.ChPovRay_GetCustomPOVcommandsData(self)

    def SetFramenumber(self, fn):
        r"""SetFramenumber(ChPovRay self, unsigned int fn)"""
        return _postprocess.ChPovRay_SetFramenumber(self, fn)

    def ExportScript(self, *args):
        r"""
        ExportScript(ChPovRay self)
        ExportScript(ChPovRay self, std::string const & filename)
        """
        return _postprocess.ChPovRay_ExportScript(self, *args)

    def ExportData(self, *args):
        r"""
        ExportData(ChPovRay self)
        ExportData(ChPovRay self, std::string const & filename)
        """
        return _postprocess.ChPovRay_ExportData(self, *args)

    def SetUseSingleAssetFile(self, use):
        r"""SetUseSingleAssetFile(ChPovRay self, bool use)"""
        return _postprocess.ChPovRay_SetUseSingleAssetFile(self, use)

# Register ChPovRay in _postprocess:
_postprocess.ChPovRay_swigregister(ChPovRay)

class ChGnuPlotDataplot(object):
    r"""Proxy of C++ chrono::postprocess::ChGnuPlotDataplot class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    command = property(_postprocess.ChGnuPlotDataplot_command_get, _postprocess.ChGnuPlotDataplot_command_set, doc=r"""command : std::string""")
    data = property(_postprocess.ChGnuPlotDataplot_data_get, _postprocess.ChGnuPlotDataplot_data_set, doc=r"""data : chrono::ChMatrixDynamic<(double)>""")

    def __init__(self):
        r"""__init__(ChGnuPlotDataplot self) -> ChGnuPlotDataplot"""
        _postprocess.ChGnuPlotDataplot_swiginit(self, _postprocess.new_ChGnuPlotDataplot())
    __swig_destroy__ = _postprocess.delete_ChGnuPlotDataplot

# Register ChGnuPlotDataplot in _postprocess:
_postprocess.ChGnuPlotDataplot_swigregister(ChGnuPlotDataplot)

class ChGnuPlot(object):
    r"""Proxy of C++ chrono::postprocess::ChGnuPlot class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self, *args):
        r"""__init__(ChGnuPlot self, std::string const & mgpl_filename="__tmp_gnuplot.gpl") -> ChGnuPlot"""
        _postprocess.ChGnuPlot_swiginit(self, _postprocess.new_ChGnuPlot(*args))
    __swig_destroy__ = _postprocess.delete_ChGnuPlot

    def SetPersist(self, mpersist):
        r"""SetPersist(ChGnuPlot self, bool mpersist)"""
        return _postprocess.ChGnuPlot_SetPersist(self, mpersist)

    def SetCommand(self, command):
        r"""SetCommand(ChGnuPlot self, std::string const & command)"""
        return _postprocess.ChGnuPlot_SetCommand(self, command)

    def __lshift__(self, command):
        r"""__lshift__(ChGnuPlot self, std::string const & command) -> ChGnuPlot"""
        return _postprocess.ChGnuPlot___lshift__(self, command)

    def Plot(self, *args):
        r"""
        Plot(ChGnuPlot self, std::string const & datfile, int colX, int colY, std::string const & title, std::string const & customsettings=" with lines ")
        Plot(ChGnuPlot self, ChVectorDynamicD mx, ChVectorDynamicD my, std::string const & title="", std::string const & customsettings=" with lines ")
        Plot(ChGnuPlot self, chrono::ChMatrixConstRef mdata, int colX, int colY, std::string const & title="", std::string const & customsettings=" with lines ")
        Plot(ChGnuPlot self, ChFunction_Recorder & mrecorder, std::string const & title="", std::string const & customsettings=" with lines ")
        Plot(ChGnuPlot self, std::vector< double,std::allocator< double > > const & vals_x, std::vector< double,std::allocator< double > > const & vals_y, std::string const & title="", std::string const & customsettings=" with lines ")
        Plot(ChGnuPlot self, ChFunction_Oscilloscope & mrecorder, std::string const & title="", std::string const & customsettings=" with lines ")
        Plot(ChGnuPlot self, ChFunction & mfunct, double xmin, double xmax, double dx, std::string const & title="", std::string const & customsettings=" with lines ")
        Plot(ChGnuPlot self, ChFunction & mfunct, int der_order, double xmin, double xmax, double dx, std::string const & title="", std::string const & customsettings=" with lines ")
        """
        return _postprocess.ChGnuPlot_Plot(self, *args)

    def Replot(self):
        r"""Replot(ChGnuPlot self)"""
        return _postprocess.ChGnuPlot_Replot(self)

    def SetRangeX(self, mmin, mmax, automin=False, automax=False):
        r"""SetRangeX(ChGnuPlot self, double mmin, double mmax, bool automin=False, bool automax=False)"""
        return _postprocess.ChGnuPlot_SetRangeX(self, mmin, mmax, automin, automax)

    def SetRangeY(self, mmin, mmax, automin=False, automax=False):
        r"""SetRangeY(ChGnuPlot self, double mmin, double mmax, bool automin=False, bool automax=False)"""
        return _postprocess.ChGnuPlot_SetRangeY(self, mmin, mmax, automin, automax)

    def SetRangeZ(self, mmin, mmax, automin=False, automax=False):
        r"""SetRangeZ(ChGnuPlot self, double mmin, double mmax, bool automin=False, bool automax=False)"""
        return _postprocess.ChGnuPlot_SetRangeZ(self, mmin, mmax, automin, automax)

    def SetTitle(self, mlabel):
        r"""SetTitle(ChGnuPlot self, std::string const & mlabel)"""
        return _postprocess.ChGnuPlot_SetTitle(self, mlabel)

    def SetLabelX(self, mlabel):
        r"""SetLabelX(ChGnuPlot self, std::string const & mlabel)"""
        return _postprocess.ChGnuPlot_SetLabelX(self, mlabel)

    def SetLabelY(self, mlabel):
        r"""SetLabelY(ChGnuPlot self, std::string const & mlabel)"""
        return _postprocess.ChGnuPlot_SetLabelY(self, mlabel)

    def SetLabelZ(self, mlabel):
        r"""SetLabelZ(ChGnuPlot self, std::string const & mlabel)"""
        return _postprocess.ChGnuPlot_SetLabelZ(self, mlabel)

    def SetGrid(self, *args):
        r"""SetGrid(ChGnuPlot self, bool dashed=True, double linewidth=1.0, ChColor mcolor=chrono::ChColor(0, 0, 0))"""
        return _postprocess.ChGnuPlot_SetGrid(self, *args)

    def SetLegend(self, flag):
        r"""SetLegend(ChGnuPlot self, bool flag)"""
        return _postprocess.ChGnuPlot_SetLegend(self, flag)

    def SetAxesEqual(self):
        r"""SetAxesEqual(ChGnuPlot self)"""
        return _postprocess.ChGnuPlot_SetAxesEqual(self)

    def OutputWindow(self, windownum=0):
        r"""OutputWindow(ChGnuPlot self, int windownum=0)"""
        return _postprocess.ChGnuPlot_OutputWindow(self, windownum)

    def OutputPNG(self, filename, sizex=400, sizey=300):
        r"""OutputPNG(ChGnuPlot self, std::string const & filename, int sizex=400, int sizey=300)"""
        return _postprocess.ChGnuPlot_OutputPNG(self, filename, sizex, sizey)

    def OutputEPS(self, filename, inchsizex=4, inchsizey=3, color=True):
        r"""OutputEPS(ChGnuPlot self, std::string const & filename, double inchsizex=4, double inchsizey=3, bool color=True)"""
        return _postprocess.ChGnuPlot_OutputEPS(self, filename, inchsizex, inchsizey, color)

    def OutputCustomTerminal(self, filename, terminalsetting):
        r"""OutputCustomTerminal(ChGnuPlot self, std::string const & filename, std::string const & terminalsetting)"""
        return _postprocess.ChGnuPlot_OutputCustomTerminal(self, filename, terminalsetting)

# Register ChGnuPlot in _postprocess:
_postprocess.ChGnuPlot_swigregister(ChGnuPlot)



