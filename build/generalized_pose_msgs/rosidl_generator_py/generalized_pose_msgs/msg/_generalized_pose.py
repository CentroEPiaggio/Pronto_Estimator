# generated from rosidl_generator_py/resource/_idl.py.em
# with input from generalized_pose_msgs:msg/GeneralizedPose.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'feet_acc'
# Member 'feet_vel'
# Member 'feet_pos'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GeneralizedPose(type):
    """Metaclass of message 'GeneralizedPose'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('generalized_pose_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'generalized_pose_msgs.msg.GeneralizedPose')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__generalized_pose
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__generalized_pose
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__generalized_pose
            cls._TYPE_SUPPORT = module.type_support_msg__msg__generalized_pose
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__generalized_pose

            from geometry_msgs.msg import Quaternion
            if Quaternion.__class__._TYPE_SUPPORT is None:
                Quaternion.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GeneralizedPose(metaclass=Metaclass_GeneralizedPose):
    """Message class 'GeneralizedPose'."""

    __slots__ = [
        '_base_acc',
        '_base_vel',
        '_base_pos',
        '_base_angvel',
        '_base_quat',
        '_feet_acc',
        '_feet_vel',
        '_feet_pos',
        '_contact_feet',
    ]

    _fields_and_field_types = {
        'base_acc': 'geometry_msgs/Vector3',
        'base_vel': 'geometry_msgs/Vector3',
        'base_pos': 'geometry_msgs/Vector3',
        'base_angvel': 'geometry_msgs/Vector3',
        'base_quat': 'geometry_msgs/Quaternion',
        'feet_acc': 'sequence<double>',
        'feet_vel': 'sequence<double>',
        'feet_pos': 'sequence<double>',
        'contact_feet': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Vector3
        self.base_acc = kwargs.get('base_acc', Vector3())
        from geometry_msgs.msg import Vector3
        self.base_vel = kwargs.get('base_vel', Vector3())
        from geometry_msgs.msg import Vector3
        self.base_pos = kwargs.get('base_pos', Vector3())
        from geometry_msgs.msg import Vector3
        self.base_angvel = kwargs.get('base_angvel', Vector3())
        from geometry_msgs.msg import Quaternion
        self.base_quat = kwargs.get('base_quat', Quaternion())
        self.feet_acc = array.array('d', kwargs.get('feet_acc', []))
        self.feet_vel = array.array('d', kwargs.get('feet_vel', []))
        self.feet_pos = array.array('d', kwargs.get('feet_pos', []))
        self.contact_feet = kwargs.get('contact_feet', [])

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
        if self.base_acc != other.base_acc:
            return False
        if self.base_vel != other.base_vel:
            return False
        if self.base_pos != other.base_pos:
            return False
        if self.base_angvel != other.base_angvel:
            return False
        if self.base_quat != other.base_quat:
            return False
        if self.feet_acc != other.feet_acc:
            return False
        if self.feet_vel != other.feet_vel:
            return False
        if self.feet_pos != other.feet_pos:
            return False
        if self.contact_feet != other.contact_feet:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def base_acc(self):
        """Message field 'base_acc'."""
        return self._base_acc

    @base_acc.setter
    def base_acc(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'base_acc' field must be a sub message of type 'Vector3'"
        self._base_acc = value

    @builtins.property
    def base_vel(self):
        """Message field 'base_vel'."""
        return self._base_vel

    @base_vel.setter
    def base_vel(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'base_vel' field must be a sub message of type 'Vector3'"
        self._base_vel = value

    @builtins.property
    def base_pos(self):
        """Message field 'base_pos'."""
        return self._base_pos

    @base_pos.setter
    def base_pos(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'base_pos' field must be a sub message of type 'Vector3'"
        self._base_pos = value

    @builtins.property
    def base_angvel(self):
        """Message field 'base_angvel'."""
        return self._base_angvel

    @base_angvel.setter
    def base_angvel(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'base_angvel' field must be a sub message of type 'Vector3'"
        self._base_angvel = value

    @builtins.property
    def base_quat(self):
        """Message field 'base_quat'."""
        return self._base_quat

    @base_quat.setter
    def base_quat(self, value):
        if __debug__:
            from geometry_msgs.msg import Quaternion
            assert \
                isinstance(value, Quaternion), \
                "The 'base_quat' field must be a sub message of type 'Quaternion'"
        self._base_quat = value

    @builtins.property
    def feet_acc(self):
        """Message field 'feet_acc'."""
        return self._feet_acc

    @feet_acc.setter
    def feet_acc(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'feet_acc' array.array() must have the type code of 'd'"
            self._feet_acc = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'feet_acc' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._feet_acc = array.array('d', value)

    @builtins.property
    def feet_vel(self):
        """Message field 'feet_vel'."""
        return self._feet_vel

    @feet_vel.setter
    def feet_vel(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'feet_vel' array.array() must have the type code of 'd'"
            self._feet_vel = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'feet_vel' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._feet_vel = array.array('d', value)

    @builtins.property
    def feet_pos(self):
        """Message field 'feet_pos'."""
        return self._feet_pos

    @feet_pos.setter
    def feet_pos(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'feet_pos' array.array() must have the type code of 'd'"
            self._feet_pos = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'feet_pos' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._feet_pos = array.array('d', value)

    @builtins.property
    def contact_feet(self):
        """Message field 'contact_feet'."""
        return self._contact_feet

    @contact_feet.setter
    def contact_feet(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'contact_feet' field must be a set or sequence and each value of type 'str'"
        self._contact_feet = value
