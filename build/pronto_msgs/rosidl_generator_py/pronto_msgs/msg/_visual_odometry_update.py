# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pronto_msgs:msg/VisualOdometryUpdate.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'covariance'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VisualOdometryUpdate(type):
    """Metaclass of message 'VisualOdometryUpdate'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NO_DATA': 0,
        'ESTIMATE_VALID': 1,
        'ESTIMATE_INSUFFICIENT_FEATURES': 2,
        'ESTIMATE_DEGENERATE': 3,
        'ESTIMATE_REPROJECTION_ERROR': 4,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('pronto_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pronto_msgs.msg.VisualOdometryUpdate')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__visual_odometry_update
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__visual_odometry_update
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__visual_odometry_update
            cls._TYPE_SUPPORT = module.type_support_msg__msg__visual_odometry_update
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__visual_odometry_update

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from geometry_msgs.msg import Transform
            if Transform.__class__._TYPE_SUPPORT is None:
                Transform.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NO_DATA': cls.__constants['NO_DATA'],
            'ESTIMATE_VALID': cls.__constants['ESTIMATE_VALID'],
            'ESTIMATE_INSUFFICIENT_FEATURES': cls.__constants['ESTIMATE_INSUFFICIENT_FEATURES'],
            'ESTIMATE_DEGENERATE': cls.__constants['ESTIMATE_DEGENERATE'],
            'ESTIMATE_REPROJECTION_ERROR': cls.__constants['ESTIMATE_REPROJECTION_ERROR'],
        }

    @property
    def NO_DATA(self):
        """Message constant 'NO_DATA'."""
        return Metaclass_VisualOdometryUpdate.__constants['NO_DATA']

    @property
    def ESTIMATE_VALID(self):
        """Message constant 'ESTIMATE_VALID'."""
        return Metaclass_VisualOdometryUpdate.__constants['ESTIMATE_VALID']

    @property
    def ESTIMATE_INSUFFICIENT_FEATURES(self):
        """Message constant 'ESTIMATE_INSUFFICIENT_FEATURES'."""
        return Metaclass_VisualOdometryUpdate.__constants['ESTIMATE_INSUFFICIENT_FEATURES']

    @property
    def ESTIMATE_DEGENERATE(self):
        """Message constant 'ESTIMATE_DEGENERATE'."""
        return Metaclass_VisualOdometryUpdate.__constants['ESTIMATE_DEGENERATE']

    @property
    def ESTIMATE_REPROJECTION_ERROR(self):
        """Message constant 'ESTIMATE_REPROJECTION_ERROR'."""
        return Metaclass_VisualOdometryUpdate.__constants['ESTIMATE_REPROJECTION_ERROR']


class VisualOdometryUpdate(metaclass=Metaclass_VisualOdometryUpdate):
    """
    Message class 'VisualOdometryUpdate'.

    Constants:
      NO_DATA
      ESTIMATE_VALID
      ESTIMATE_INSUFFICIENT_FEATURES
      ESTIMATE_DEGENERATE
      ESTIMATE_REPROJECTION_ERROR
    """

    __slots__ = [
        '_header',
        '_curr_timestamp',
        '_prev_timestamp',
        '_relative_transform',
        '_covariance',
        '_estimate_status',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'curr_timestamp': 'builtin_interfaces/Time',
        'prev_timestamp': 'builtin_interfaces/Time',
        'relative_transform': 'geometry_msgs/Transform',
        'covariance': 'double[36]',
        'estimate_status': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Transform'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 36),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from builtin_interfaces.msg import Time
        self.curr_timestamp = kwargs.get('curr_timestamp', Time())
        from builtin_interfaces.msg import Time
        self.prev_timestamp = kwargs.get('prev_timestamp', Time())
        from geometry_msgs.msg import Transform
        self.relative_transform = kwargs.get('relative_transform', Transform())
        if 'covariance' not in kwargs:
            self.covariance = numpy.zeros(36, dtype=numpy.float64)
        else:
            self.covariance = numpy.array(kwargs.get('covariance'), dtype=numpy.float64)
            assert self.covariance.shape == (36, )
        self.estimate_status = kwargs.get('estimate_status', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.curr_timestamp != other.curr_timestamp:
            return False
        if self.prev_timestamp != other.prev_timestamp:
            return False
        if self.relative_transform != other.relative_transform:
            return False
        if all(self.covariance != other.covariance):
            return False
        if self.estimate_status != other.estimate_status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def curr_timestamp(self):
        """Message field 'curr_timestamp'."""
        return self._curr_timestamp

    @curr_timestamp.setter
    def curr_timestamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'curr_timestamp' field must be a sub message of type 'Time'"
        self._curr_timestamp = value

    @builtins.property
    def prev_timestamp(self):
        """Message field 'prev_timestamp'."""
        return self._prev_timestamp

    @prev_timestamp.setter
    def prev_timestamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'prev_timestamp' field must be a sub message of type 'Time'"
        self._prev_timestamp = value

    @builtins.property
    def relative_transform(self):
        """Message field 'relative_transform'."""
        return self._relative_transform

    @relative_transform.setter
    def relative_transform(self, value):
        if __debug__:
            from geometry_msgs.msg import Transform
            assert \
                isinstance(value, Transform), \
                "The 'relative_transform' field must be a sub message of type 'Transform'"
        self._relative_transform = value

    @builtins.property
    def covariance(self):
        """Message field 'covariance'."""
        return self._covariance

    @covariance.setter
    def covariance(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'covariance' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 36, \
                "The 'covariance' numpy.ndarray() must have a size of 36"
            self._covariance = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 36 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'covariance' field must be a set or sequence with length 36 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._covariance = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def estimate_status(self):
        """Message field 'estimate_status'."""
        return self._estimate_status

    @estimate_status.setter
    def estimate_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'estimate_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'estimate_status' field must be an unsigned integer in [0, 255]"
        self._estimate_status = value
