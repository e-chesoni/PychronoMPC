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
    from . import _pardisomkl
else:
    import _pardisomkl

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

SHARED_PTR_DISOWN = _pardisomkl.SHARED_PTR_DISOWN

class SwigPyIterator(object):
    r"""Proxy of C++ swig::SwigPyIterator class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _pardisomkl.delete_SwigPyIterator

    def value(self):
        r"""value(SwigPyIterator self) -> PyObject *"""
        return _pardisomkl.SwigPyIterator_value(self)

    def incr(self, n=1):
        r"""incr(SwigPyIterator self, size_t n=1) -> SwigPyIterator"""
        return _pardisomkl.SwigPyIterator_incr(self, n)

    def decr(self, n=1):
        r"""decr(SwigPyIterator self, size_t n=1) -> SwigPyIterator"""
        return _pardisomkl.SwigPyIterator_decr(self, n)

    def distance(self, x):
        r"""distance(SwigPyIterator self, SwigPyIterator x) -> ptrdiff_t"""
        return _pardisomkl.SwigPyIterator_distance(self, x)

    def equal(self, x):
        r"""equal(SwigPyIterator self, SwigPyIterator x) -> bool"""
        return _pardisomkl.SwigPyIterator_equal(self, x)

    def copy(self):
        r"""copy(SwigPyIterator self) -> SwigPyIterator"""
        return _pardisomkl.SwigPyIterator_copy(self)

    def next(self):
        r"""next(SwigPyIterator self) -> PyObject *"""
        return _pardisomkl.SwigPyIterator_next(self)

    def __next__(self):
        r"""__next__(SwigPyIterator self) -> PyObject *"""
        return _pardisomkl.SwigPyIterator___next__(self)

    def previous(self):
        r"""previous(SwigPyIterator self) -> PyObject *"""
        return _pardisomkl.SwigPyIterator_previous(self)

    def advance(self, n):
        r"""advance(SwigPyIterator self, ptrdiff_t n) -> SwigPyIterator"""
        return _pardisomkl.SwigPyIterator_advance(self, n)

    def __eq__(self, x):
        r"""__eq__(SwigPyIterator self, SwigPyIterator x) -> bool"""
        return _pardisomkl.SwigPyIterator___eq__(self, x)

    def __ne__(self, x):
        r"""__ne__(SwigPyIterator self, SwigPyIterator x) -> bool"""
        return _pardisomkl.SwigPyIterator___ne__(self, x)

    def __iadd__(self, n):
        r"""__iadd__(SwigPyIterator self, ptrdiff_t n) -> SwigPyIterator"""
        return _pardisomkl.SwigPyIterator___iadd__(self, n)

    def __isub__(self, n):
        r"""__isub__(SwigPyIterator self, ptrdiff_t n) -> SwigPyIterator"""
        return _pardisomkl.SwigPyIterator___isub__(self, n)

    def __add__(self, n):
        r"""__add__(SwigPyIterator self, ptrdiff_t n) -> SwigPyIterator"""
        return _pardisomkl.SwigPyIterator___add__(self, n)

    def __sub__(self, *args):
        r"""
        __sub__(SwigPyIterator self, ptrdiff_t n) -> SwigPyIterator
        __sub__(SwigPyIterator self, SwigPyIterator x) -> ptrdiff_t
        """
        return _pardisomkl.SwigPyIterator___sub__(self, *args)
    def __iter__(self):
        return self

# Register SwigPyIterator in _pardisomkl:
_pardisomkl.SwigPyIterator_swigregister(SwigPyIterator)

class int_ptr(object):
    r"""Proxy of C++ int_ptr class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self):
        r"""__init__(int_ptr self) -> int_ptr"""
        _pardisomkl.int_ptr_swiginit(self, _pardisomkl.new_int_ptr())
    __swig_destroy__ = _pardisomkl.delete_int_ptr

    def assign(self, value):
        r"""assign(int_ptr self, int value)"""
        return _pardisomkl.int_ptr_assign(self, value)

    def value(self):
        r"""value(int_ptr self) -> int"""
        return _pardisomkl.int_ptr_value(self)

    def cast(self):
        r"""cast(int_ptr self) -> int *"""
        return _pardisomkl.int_ptr_cast(self)

    @staticmethod
    def frompointer(t):
        r"""frompointer(int * t) -> int_ptr"""
        return _pardisomkl.int_ptr_frompointer(t)

# Register int_ptr in _pardisomkl:
_pardisomkl.int_ptr_swigregister(int_ptr)

def int_ptr_frompointer(t):
    r"""int_ptr_frompointer(int * t) -> int_ptr"""
    return _pardisomkl.int_ptr_frompointer(t)

class double_ptr(object):
    r"""Proxy of C++ double_ptr class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self):
        r"""__init__(double_ptr self) -> double_ptr"""
        _pardisomkl.double_ptr_swiginit(self, _pardisomkl.new_double_ptr())
    __swig_destroy__ = _pardisomkl.delete_double_ptr

    def assign(self, value):
        r"""assign(double_ptr self, double value)"""
        return _pardisomkl.double_ptr_assign(self, value)

    def value(self):
        r"""value(double_ptr self) -> double"""
        return _pardisomkl.double_ptr_value(self)

    def cast(self):
        r"""cast(double_ptr self) -> double *"""
        return _pardisomkl.double_ptr_cast(self)

    @staticmethod
    def frompointer(t):
        r"""frompointer(double * t) -> double_ptr"""
        return _pardisomkl.double_ptr_frompointer(t)

# Register double_ptr in _pardisomkl:
_pardisomkl.double_ptr_swigregister(double_ptr)

def double_ptr_frompointer(t):
    r"""double_ptr_frompointer(double * t) -> double_ptr"""
    return _pardisomkl.double_ptr_frompointer(t)

class float_ptr(object):
    r"""Proxy of C++ float_ptr class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self):
        r"""__init__(float_ptr self) -> float_ptr"""
        _pardisomkl.float_ptr_swiginit(self, _pardisomkl.new_float_ptr())
    __swig_destroy__ = _pardisomkl.delete_float_ptr

    def assign(self, value):
        r"""assign(float_ptr self, float value)"""
        return _pardisomkl.float_ptr_assign(self, value)

    def value(self):
        r"""value(float_ptr self) -> float"""
        return _pardisomkl.float_ptr_value(self)

    def cast(self):
        r"""cast(float_ptr self) -> float *"""
        return _pardisomkl.float_ptr_cast(self)

    @staticmethod
    def frompointer(t):
        r"""frompointer(float * t) -> float_ptr"""
        return _pardisomkl.float_ptr_frompointer(t)

# Register float_ptr in _pardisomkl:
_pardisomkl.float_ptr_swigregister(float_ptr)

def float_ptr_frompointer(t):
    r"""float_ptr_frompointer(float * t) -> float_ptr"""
    return _pardisomkl.float_ptr_frompointer(t)

import pychrono.core
class ChSolverPardisoMKL(pychrono.core.ChDirectSolverLS):
    r"""Proxy of C++ chrono::ChSolverPardisoMKL class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self, num_threads=0):
        r"""__init__(ChSolverPardisoMKL self, int num_threads=0) -> ChSolverPardisoMKL"""
        _pardisomkl.ChSolverPardisoMKL_swiginit(self, _pardisomkl.new_ChSolverPardisoMKL(num_threads))
    __swig_destroy__ = _pardisomkl.delete_ChSolverPardisoMKL

    def GetType(self):
        r"""GetType(ChSolverPardisoMKL self) -> chrono::ChSolver::Type"""
        return _pardisomkl.ChSolverPardisoMKL_GetType(self)

    def GetMklEngine(self):
        r"""GetMklEngine(ChSolverPardisoMKL self) -> Eigen::PardisoLU< ChSparseMatrix > &"""
        return _pardisomkl.ChSolverPardisoMKL_GetMklEngine(self)

# Register ChSolverPardisoMKL in _pardisomkl:
_pardisomkl.ChSolverPardisoMKL_swigregister(ChSolverPardisoMKL)

class ChSolverComplexPardisoMKL(object):
    r"""Proxy of C++ chrono::ChSolverComplexPardisoMKL class."""

    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self, num_threads=0):
        r"""__init__(ChSolverComplexPardisoMKL self, int num_threads=0) -> ChSolverComplexPardisoMKL"""
        _pardisomkl.ChSolverComplexPardisoMKL_swiginit(self, _pardisomkl.new_ChSolverComplexPardisoMKL(num_threads))
    __swig_destroy__ = _pardisomkl.delete_ChSolverComplexPardisoMKL

    def GetMklEngine(self):
        r"""GetMklEngine(ChSolverComplexPardisoMKL self) -> Eigen::PardisoLU< Eigen::SparseMatrix< std::complex< double >,Eigen::ColMajor > > &"""
        return _pardisomkl.ChSolverComplexPardisoMKL_GetMklEngine(self)

# Register ChSolverComplexPardisoMKL in _pardisomkl:
_pardisomkl.ChSolverComplexPardisoMKL_swigregister(ChSolverComplexPardisoMKL)


def CastToChSolverPardisoMKL(in_obj):
    r"""CastToChSolverPardisoMKL(std::shared_ptr< chrono::ChSolver > in_obj) -> std::shared_ptr< chrono::ChSolverPardisoMKL >"""
    return _pardisomkl.CastToChSolverPardisoMKL(in_obj)

