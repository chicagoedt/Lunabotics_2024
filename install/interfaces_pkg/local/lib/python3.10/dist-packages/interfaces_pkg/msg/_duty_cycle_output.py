# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interfaces_pkg:msg/DutyCycleOutput.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DutyCycleOutput(type):
    """Metaclass of message 'DutyCycleOutput'."""

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
            module = import_type_support('interfaces_pkg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interfaces_pkg.msg.DutyCycleOutput')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__duty_cycle_output
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__duty_cycle_output
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__duty_cycle_output
            cls._TYPE_SUPPORT = module.type_support_msg__msg__duty_cycle_output
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__duty_cycle_output

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DutyCycleOutput(metaclass=Metaclass_DutyCycleOutput):
    """Message class 'DutyCycleOutput'."""

    __slots__ = [
        '_left_drive_output',
        '_right_drive_output',
        '_left_lift_output',
        '_right_lift_output',
        '_tilt_output',
        '_vibrator_output',
    ]

    _fields_and_field_types = {
        'left_drive_output': 'double',
        'right_drive_output': 'double',
        'left_lift_output': 'double',
        'right_lift_output': 'double',
        'tilt_output': 'double',
        'vibrator_output': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.left_drive_output = kwargs.get('left_drive_output', float())
        self.right_drive_output = kwargs.get('right_drive_output', float())
        self.left_lift_output = kwargs.get('left_lift_output', float())
        self.right_lift_output = kwargs.get('right_lift_output', float())
        self.tilt_output = kwargs.get('tilt_output', float())
        self.vibrator_output = kwargs.get('vibrator_output', float())

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
        if self.left_drive_output != other.left_drive_output:
            return False
        if self.right_drive_output != other.right_drive_output:
            return False
        if self.left_lift_output != other.left_lift_output:
            return False
        if self.right_lift_output != other.right_lift_output:
            return False
        if self.tilt_output != other.tilt_output:
            return False
        if self.vibrator_output != other.vibrator_output:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def left_drive_output(self):
        """Message field 'left_drive_output'."""
        return self._left_drive_output

    @left_drive_output.setter
    def left_drive_output(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_drive_output' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_drive_output' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_drive_output = value

    @builtins.property
    def right_drive_output(self):
        """Message field 'right_drive_output'."""
        return self._right_drive_output

    @right_drive_output.setter
    def right_drive_output(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_drive_output' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_drive_output' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_drive_output = value

    @builtins.property
    def left_lift_output(self):
        """Message field 'left_lift_output'."""
        return self._left_lift_output

    @left_lift_output.setter
    def left_lift_output(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_lift_output' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_lift_output' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_lift_output = value

    @builtins.property
    def right_lift_output(self):
        """Message field 'right_lift_output'."""
        return self._right_lift_output

    @right_lift_output.setter
    def right_lift_output(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_lift_output' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_lift_output' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_lift_output = value

    @builtins.property
    def tilt_output(self):
        """Message field 'tilt_output'."""
        return self._tilt_output

    @tilt_output.setter
    def tilt_output(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tilt_output' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tilt_output' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tilt_output = value

    @builtins.property
    def vibrator_output(self):
        """Message field 'vibrator_output'."""
        return self._vibrator_output

    @vibrator_output.setter
    def vibrator_output(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vibrator_output' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vibrator_output' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vibrator_output = value
