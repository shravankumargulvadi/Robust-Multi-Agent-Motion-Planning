"""
MAPF plan file reader
"""

def read_plan(file_name, skip_line=1, split_char="|"):
    """File reader of MAPF plan

    Args:
        file_name (str): File name
        skip_line (int): Skip first several lines in the file in case begining of
                         files are used to store extra information
        split_char (str): Splitor for each line


    Example:
       Sample input:
           |A|B|
           |B|C|
           |C|F|
           |D|C|
           |E|D|

       The MAPF plan has 5 time steps.
       Agent 0's path: A->B->C->D->E
       Agent 1's path: B->C->F->C->D


       Sample output:
           [[(0, 'A'), (1, 'B'), (2, 'C'), (3, 'D'), (4, 'E')],
            [(0, 'B'), (1, 'C'), (2, 'F'), (3, 'C'), (4, 'D')]]

    """

    with open(file_name, "r") as f:
        docs = f.readlines()[skip_line:]
    docs = [doc.rstrip().split(split_char) for doc in docs]
    docs = [[w for w in doc if w != ""] for doc in docs if [w for w in doc if w != ""]]

    def to_timed_plan(plan):
        """Remove wait action
        """
        arr = []
        for idx, node in enumerate(plan):
            if arr and arr[-1][1] == node:
                continue
            arr.append((idx, node))
        return arr

    plans = [list(l) for l in zip(*docs)]
    plans = [to_timed_plan(plan) for plan in plans]

    # Ignore agent not moving at all
    return [p for p in plans if len(p) > 1]

def sample_mapf_plan():
    """return a sample MAPF plan used for debugging
    """
    return  [[(0, 'A'), (1, 'C'), (2, 'D'), (3, 'E')],
             [(0, 'B'), (2, 'C'), (3, 'D')]]



def sample_mapf_plan_2():
    """return a sample MAPF plan used for debugging
    """
    return  [[(0, 'A'), (2, 'B'), (3, 'C')],
             [(0, 'D'), (1, 'B'), (2, 'E')]]

def sample_mapf_plan_h16():
    """return a sample MAPF plan used for debugging
    """
    return  [[(0, 'A'), (1, 'B'), (2, 'C'), (3, 'D'), (4,'E')],
             [(0, 'B'), (1, 'C'), (2, 'F'), (3, 'C'), (4,'D')]]

