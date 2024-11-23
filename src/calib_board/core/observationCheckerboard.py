import numpy as np

class ObservationCheckerboard: 

    def __init__(self, 
                 _2d: np.ndarray = None, 
                 is_conform: bool = True):
        self._2d = _2d 
        self._conformity_mask = np.ones(len(_2d), dtype=bool)  # by default all corners observations are conform
        self._is_conform = is_conform

    def get_number_conform_corners(self) -> int: 
        return self._conformity_mask.sum()
    

    def get_2d_conform(self): 
        return self._2d[self._conformity_mask, :]
    
    def get_2d_nonconform(self): 
        return self._2d[~self._conformity_mask, :]

