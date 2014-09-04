import numpy

def cast_array_as_matrix(array):
    """
    cast a numpy array as (size,1)-matrix to enable straight forward linear algebra
    """
    err_str = "Only accepts 1 dimensional numpy arrays as inputs.\n"
    assert len(array.shape) == 1, err_str
    return numpy.matrix(array.reshape(array.size, 1))
