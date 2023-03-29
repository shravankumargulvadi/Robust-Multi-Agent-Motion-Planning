"""
Preprocessing and class Delta(set of state transition)

See preprocessing part in paper.


Kinodynamic constraints are specified by these arguments:
    start_state, end_state - an array of number/None s
        i-th number is n represent start/ending state with n in (i + 1)-th derivative
        of dynamics, None represent this order is not constrainted.

        Example:
             start_state = [None, 2] means we want agent start with acceleration as but
             velocity not constrainted

    bounds - array of number/None pairs
        i-th pair is lower/upper bound in (i + 1)-th derivative of dynamics, None
        represent that side is not bounded

        Example:
             bounds = [(0, 1), (None, 2)] means agent's velocity bounded by [0, 1]
             and acceleration is bounded by 2, but deceleration is unbounded.



TODO
   There are still hard-coded constant in this file

"""
import pickle
from scipy.misc import derivative
from kinodynamic_feasible import kinodynamic_feasible_aux, velocity_profile




def find_initial_feasible_point(L, lb, ub, start_state, end_state, bounds):
    """
    Find an initial feasible solution for given kinodynamic constraint in [lb, ub]

    Args:
        L - distance
        lb, ub - area of t that we are searching on, 0 < lb < ub.
        start_state, end_state, bounds - see beginning of files

    Outputs:
        feasible t or None

    """
    bs_threshold = 0.000001
    deriv_step=0.0000001

    I = lambda t: kinodynamic_feasible_aux(L, t, start_state, end_state, bounds, 20)[1]


    while (abs(ub - lb) > bs_threshold):
        mid = (ub + lb) / 2
        d_mid = derivative(I, mid, deriv_step)
        print((mid, d_mid))
        if abs(d_mid) < 0.001:
            i = I(mid)
            print(i)
            if abs(i) < 0.000001:
                return mid
            print("continuing")
            # if abs(i) > 0.00001:
            #     return None

        if d_mid > 0:
            ub = mid
        else:
            lb = mid
    return None


def find_feasible_interval(L, lb, ub, start_state, end_state, bounds):
    """ Attempt to find feasible interval. return None if not found

    Args:
        L - distance
        lb, ub - area of t that we are searching on, 0 < lb < ub.
        start_state, end_state, bounds - see beginning of files

    Outputs:
        None or
        ((lb_time, control_points), (ub_time, control_points))


    """
    threshold = 0.00001
    # Find initial solution
    initial_t = find_initial_feasible_point(L, lb, ub, start_state, end_state, bounds)
    if initial_t is None:
        return None
    if velocity_profile(L, initial_t, start_state, end_state, bounds, 40) is None:
        # There is very rare case that solution feasible in aux but
        # not feasible in original LP

        print(f"warning, inconsistency solution found in find_feasible_interval {(start_state, end_state, bounds)}")
        return None
    # Binary search


    # Binary search on both side
    ub_ = initial_t
    lb_ = lb
    while abs(ub_ - lb_) > threshold:
        mid = (lb_ + ub_)/2
        if velocity_profile(L, mid, start_state, end_state, bounds, 40) is not None:
            ub_ = mid
        else:
            lb_ = mid

    ret_lb = ub_

    lb_ = initial_t
    ub_ = ub
    while abs(ub_ - lb_) > threshold:
        mid = (lb_ + ub_)/2
        if velocity_profile(L, mid, start_state, end_state, bounds, 40) is None:
            ub_ = mid
        else:
            lb_ = mid
    ret_ub = lb_

    return ((ret_lb, velocity_profile(L, ret_lb, start_state, end_state, bounds, 40)),
            (ret_ub, velocity_profile(L, ret_ub, start_state, end_state, bounds, 40)))



class Delta:
    """Set of state transition and time bound.

    See paper for definition

    """

    def __init__(self, l, bounds, states=None, lb=0.1, ub=10, control_points=20):
        """
        Instance of class is initiated with edge distance and kinodynamic constraints
        It's also optional to provide a states set and transition will be attempted
        to generate pairwise.

        """
        self.l = l
        self.bounds = bounds
        self.states = states
        self.control_points = control_points
        self.lb = lb
        self.ub = ub
        self.__data__ = {}
        if states:
            self.__generate__()


    def __generate__(self):
        for start in self.states:
            for end in self.states:
                print((start, end))
                self.add_trans(start, end)

    def add_trans(self, start, end):
        """ Attempt to add transition.

        Return True if added, False if transition already contained or feasible interval
        cannot be found.
        """
        if self.have(start, end):
            return True

        res = find_feasible_interval(self.l, self.lb, self.ub, start, end, self.bounds)

        if res is not None:
            if start not in self.__data__:
                self.__data__[start] = {}
            self.__data__[start][end] = res
            return True
        return False

    def have(self, start, end):
        """
        If delta has transition (start, end)
        """
        return start in self.__data__ and end in self.__data__[start]


    def save(self, name):
        """Dump as pickle file
        """
        with open(name, "wb") as f:
            pickle.dump(self, f)

    def trans(self):
        """ return a iterator of all translation
        """
        for start in self.__data__:
            for end in self.__data__[start]:
                yield(start, end)

    def trans_start_with(self, start):
        """return a iterator of all translation with start state given
        """
        if start in self.__data__:
            for end in self.__data__[start]:
                yield (start, end)

    def generate_vp(self, t, start, end):

        # TODO if start is (0, 0) then should able to wait

        if not self.have(start, end):
            return None

        delta = 0.0001
        """
        Return cached control points already computed if time is equal to
        or nearly equal to lb or ub
        """
        if abs(self.__data__[start][end][0][0] - t) < delta:
            return self.__data__[start][end][0][1]
        if abs(self.__data__[start][end][1][0] - t) < delta:
            return self.__data__[start][end][1][1]


        """
        Generate velocity profile using lp 
        """
        # logger.debug("not cached %s bound: %s %s"%(t, self.dp[start][end][0][0], self.dp[start][end][1][0]))
        return self.__call_velocity_profile__(start, end, t)


    def __call_velocity_profile__(self, start, end, t):
        return velocity_profile(self.l, t, start, end, self.bounds, self.control_points)

    def __getitem__(self, index):
        return self.__data__[index]

    def size(self):
        """
        return number of entries
        """
        return sum([len(i[1]) for i in self.__data__.items()])

    def get_timebound(self, start, end):
        """
        return time bound as a pair of numbers
        """
        if not self.have(start, end):
            return None
        t = self[start][end]
        lb, ub = t[0][0], t[1][0]
        return (lb, ub)
