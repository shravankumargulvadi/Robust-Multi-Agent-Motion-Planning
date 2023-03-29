"""Temporal Plan Graph

Kinodynamic constraints isrepresented as Delta in each node.

"""
import logging
from util import sample_mapf_plan, sample_mapf_plan_2, sample_mapf_plan_h16
logger = logging.getLogger('tpg')
logging.basicConfig(format='%(asctime)s %(levelname)s:%(message)s', level=logging.DEBUG, datefmt='%I:%M:%S')

class TPG_node:
    """Node in Temporal Plan Graph

    Each node in TPG will have at most 1 outgoing type1 edge and
    at most 1 outgoing type 2 edge.

    Attributes:

    """

    def __init__(self, name):
        """Init node with name

        Args:
            name (str): a string as node name, which will be used in
                        printing node and graph, generating MILP etc.

        """
        self.name = name
        # Type one edge and its length
        self.next_node = None
        self.__delta__ = None


        self.t_lb_lim = None
        self.type2 = None

    def set_delta(self, delta):
        # add a dp to node.
        self.__delta__ = delta

    def add_next_node(self, node):
        self.next_node = node

    def add_type2_edge(self, node):
        self.type2 = node

    def get_delta(self):
        return self.__delta__

    def __repr__(self):
        if self.type2:
            return f"{self.name} ({self.type2.name})"
        return self.name


class TPG:
    def __init__(self, agent_num):
        self.tpg = [[] for _ in range(agent_num)]
        self.dp = [None for _ in range(agent_num)]
        self.node_dic = {}

    def append_node(self, i, node_name):
        """append node with node_name to agent i
        """
        assert node_name not in self.node_dic
        node = TPG_node(node_name)
        self.node_dic[node.name] = node
        if self.tpg[i]:
            self.tpg[i][-1].add_next_node(node)
        self.tpg[i].append(node)
        return node

    def add_type2_edge(self, node_0, node_1):
        self.node_dic[node_0].add_type2_edge(self.node_dic[node_1])

    def get_type2_edges(self):
        """
        return a iterator of all type 2 edges
        """
        for agent in self.tpg:
            for node in agent:
                if node.type2:
                    yield(node.name, node.type2.name)

    def get_type1_edges(self):
        """
        return a iterator of all type 1 edges
        """
        for agent in self.tpg:
            for node_0 in agent[:-1]:
                yield(node_0.name, node_0.next_node.name)

    def get_node(self, name):
        return self.node_dic[name]

    def get_nodes(self):
        for i in self.tpg:
            for n in i:
                yield n

    def __repr__(self):
        return "\n".join([str(a) for a in self.tpg])

    def __len__(self):
        return len(self.tpg)

    def __getitem__(self,index):
        return self.tpg[index]


def sample_tpg():
    """return a sample TPG instance

    Intend to be used for debugging
    """
    return plan_to_tpg(sample_mapf_plan())

def sample_tpg_2():
    """return a sample TPG instance

    Intend to be used for debugging
    """
    return plan_to_tpg(sample_mapf_plan_2())

def sample_tpg_h16():
    """return a sample TPG instance

    Intended to be used for debugging
    """
    return plan_to_tpg(sample_mapf_plan_h16())


def plan_to_tpg(plans):
    """transform a MAPF execution plan to MAPF plans.

    Input format:
        see comment section of read_plan function in util.py

    Output:
        A TPG instance

    """
    tpg = TPG(len(plans))
    tpg_node_dic = {}

    for agent_i, plan in enumerate(plans):
        for t, node in plan:
            name = f"{node}^{agent_i + 1}_{t}"
            tpg.append_node(agent_i, name)
            tpg_node_dic[(t, agent_i)] = name

    # Create type 2 edge
    node_time_list = {}

    for agent, plan in enumerate(plans):
        for t, node in plan:
            if node not in node_time_list:
                node_time_list[node] = []
            else:
                print(node)
            node_time_list[node].append((t, agent))
    # sorting the order of agent arriving for each location
    for k in node_time_list:
        node_time_list[k] = sorted(node_time_list[k])

    for _, l in node_time_list.items():
        """
        l list of (time, agent)
        """
        while len(l) > 1:
            node = l.pop(0)
            if node[1] != l[0][1]:
                tpg.add_type2_edge(tpg_node_dic[node], tpg_node_dic[l[0]])

    """
    agents, array of array of TPG_nodes
    """
    return tpg
