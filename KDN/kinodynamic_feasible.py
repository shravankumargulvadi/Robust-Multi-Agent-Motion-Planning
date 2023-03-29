"""LP for kinodynamic feasible velocity profile and preprocessing

There are three methods in this file which are taking same format of arguments

    L - distance
    T - time
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

     n - number of control points 

"""
import math
import numpy as np
from scipy.optimize import linprog

def kinodynamic_feasible_lp(L, T, start_state, end_state, bounds, n = 20):
    """
    return an LP with given constrainted.

    Args:
        see beginning of file

    Output:
        c, A_ub, b_ub, A_eq, b_eq
        Those are same format as arguments of linprog in scipy.optimize
    """
    c = np.zeros(n) - 1

    A_ub = []
    b_ub = []

    A_eq = []
    b_eq = []

    A_eq += [[1] + list(np.zeros(n - 1))]
    b_eq += [0]

    A_eq += [list(np.zeros(n - 1)) + [1]]
    b_eq += [L]

    d = [1]
    for idx, bound in enumerate(bounds):
        """
        idx   (int): order of derivative
        bound (int, int): lower / upper bound pairs. Unbounded if it is None
        """

        # d is used in upperbound of lp
        d = [-j + i for i,j in zip([0] + d, d+[0])]
        coeff = (math.factorial(n - idx - 2) * T**(idx + 1)) / math.factorial(n-1)

        if bounds[0] is not None:
            # lower bound
            A_tmp = [list(np.zeros(n)) for i in range(n - idx - 1)]
            b_tmp = [- bound[0] * coeff for i in range(n - idx - 1)]
            for i in range(n - idx - 1):
                for j in range(len(d)):
                    A_tmp[i][i + j] = -d[j]
            A_ub += A_tmp
            b_ub += b_tmp
        if bounds[1] is not None:
            # upper bound
            A_tmp = [list(np.zeros(n)) for i in range(n - idx - 1)]
            b_tmp = [bound[1] * coeff for i in range(n - idx - 1)]
            for i in range(n - idx - 1):
                for j in range(len(d)):
                    A_tmp[i][i + j] = d[j]
            A_ub += A_tmp
            b_ub += b_tmp

    d = [1]
    for idx, s in enumerate(start_state):
        d = [-j + i for i,j in zip([0] + d, d+[0])]
        coeff = (math.factorial(n - idx - 2) * T**(idx + 1)) / math.factorial(n-1)
        if s is not None:
            A_eq += [[num for num in d] + list(np.zeros(n - idx - 2))]
            b_eq += [s * coeff]

    d = [1]
    for idx, e in enumerate(end_state):
        d = [-j + i for i,j in zip([0] + d, d+[0])]
        coeff = (math.factorial(n - idx - 2) * T**(idx + 1)) / math.factorial(n-1)
        if e is not None:
            A_eq += [ list(np.zeros(n - idx - 2)) + [num for num in d] ]
            b_eq += [e * coeff]


    return (c, A_ub, b_ub, A_eq, b_eq)

def velocity_profile(L, T, start_state, end_state, bounds, n=20):
    """
    Generate velocity profile (in form of control points of Bernstein polynomial)
    for given constraints.

    Args:
        see beginning of file

    Output:
        an array of control points or None if not feasible

    It simply calls kinodynamic_feasible_lp and solve it using linprog

    """

    c, A_ub, b_ub, A_eq, b_eq = kinodynamic_feasible_lp(L, T, start_state, end_state, bounds, n)
    try:
        sol = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq)
    except:
        return None
    if sol.status == 0:
        return sol.x
    return None

def kinodynamic_feasible_aux(L, T, start_state, end_state, bounds, n=20, epsilon=0.01):
    """
    Return optimal value of auxiliary version of LP (see paper for definition)

    Args:
        see beginning of file

    Output:
        (status, val)
            status - status returned by linprog
            val - optimal value returned by linprog, not defined if status is
                  not feasible

    """

    _, A_ub, b_ub, A_eq, b_eq = kinodynamic_feasible_lp(L, T, start_state, end_state, bounds, n)
    c = list(np.zeros(n)) + [1]
    A_eq = [l + [0] for l in A_eq]
    A_ub = [l + [-1] for l in A_ub]

    A_ub += [list(np.zeros(n)) + [-1]]
    b_ub += [0]

    # A_ub += [l + [-epsilon] for l in A_eq]
    # b_ub += b_eq

    # A_ub += [[-v for v in l] + [-epsilon] for l in A_eq]
    # b_ub += [-v for v in b_eq]


    sol = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds = [(None, None) for _ in range(n + 1)])
    # sol = linprog(c, A_ub=A_ub, b_ub=b_ub, bounds = [(None, None) for _ in range(n + 1)])
    return sol.status, sol.fun
