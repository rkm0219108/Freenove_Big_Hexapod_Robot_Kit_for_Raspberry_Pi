from typing import Tuple, TypeVar, Union

T = TypeVar('T', int, float)

def restriction(value: T, value_range: Tuple[T, T]):
    """return value in range"""
    if value < value_range[0]:
        return value_range[0]
    elif value > value_range[1]:
        return value_range[1]
    else:
        return value

def map_num(value: Union[int, float], from_range: Tuple[int, int], to_range: Tuple[int, int]):
    """map value from old range to new range"""
    return (to_range[1] - to_range[0]) * (value - from_range[0]) / (from_range[1] - from_range[0]) + to_range[0]
