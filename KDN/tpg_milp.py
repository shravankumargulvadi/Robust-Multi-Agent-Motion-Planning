from pyscipopt import Model, quicksum
from tpg import TPG
import logging
import time
import pickle

logger = logging.getLogger('mip')
logging.basicConfig(format='%(asctime)s %(levelname)s:%(message)s', level=logging.DEBUG, datefmt='%I:%M:%S')



def tpg2milp(tpg: TPG):
    """
    tpg - a list of list of TPG nodes
    """
    model = Model("Tpg")  # model name is optional

    node_var_dic = {}
    type1_var_dic = {}

    # declare variable
    for node in tpg.get_nodes():
        t = model.addVar("t_" + node.name, vtype="C")
        node_var_dic[node.name] = t
        if node.t_lb_lim:
            model.addCons(t >= node.t_lb_lim)

    for node_0, node_1 in tpg.get_type1_edges():

        lb = model.addVar("lb_" + node_0 + "_" + node_1)
        ub = model.addVar("ub_" + node_0 + "_" + node_1)

        t_from = node_var_dic[node_0]
        t_to = node_var_dic[node_1]
        model.addCons(t_to - t_from <= ub)
        model.addCons(t_to - t_from >= lb)

        type1_var_dic[(node_0, node_1)] = (lb, ub)



    model.setObjective(quicksum(node_var_dic[agent[-1].name] for agent in tpg.tpg))

    # zero init time
    for agent in tpg.tpg:
        t0 = node_var_dic[agent[0].name]
        model.addCons(t0 == 0)

    # type2 cons
    for node_0, node_1 in tpg.get_type2_edges():
        model.addCons(node_var_dic[node_0] <= node_var_dic[node_1])

    # Encoding the table for type 1 edge
    table_type1_var_dic = {}

    for node_0, node_1 in tpg.get_type1_edges():
        # for node 0. Only consider state with 0 init speed
        trans_vars = {}
        dp = tpg.get_node(node_0).get_delta()
        for t in dp.trans():
            trans_vars[t] = model.addVar("tr_%s_%s_%s"%(node_0, t[0], t[1]) , vtype="B")

        # Mutual exclusive
        model.addCons(quicksum(var for k, var in trans_vars.items()) == 1)

        # lb, ub
        lb, ub = type1_var_dic[(node_0, node_1)]
        model.addCons(lb == quicksum(var * dp.get_timebound(k_from, k_to)[0] for (k_from, k_to), var in trans_vars.items()))
        model.addCons(ub  == quicksum(var * dp.get_timebound(k_from, k_to)[1] for (k_from, k_to), var in trans_vars.items()))
        table_type1_var_dic[node_0] = trans_vars

    for node_0, node_1 in tpg.get_type1_edges():
        dp_prev = tpg.get_node(node_0).get_delta()
        dp_next = tpg.get_node(node_1).get_delta()
        if node_1 not in table_type1_var_dic:
            continue
        for t in dp_prev.trans():
            t_var = table_type1_var_dic[node_0][t]

            next_state = t[1]

            model.addCons(t_var <= quicksum([table_type1_var_dic[node_1][t_next]
                                             for t_next in dp_next.trans_start_with(next_state)]))


    for agent in tpg.tpg:
        if len(agent) > 1:
            first_node = agent[0].name
            last_but_2_node = agent[-2].name
            # init and ending state
            model.addCons(quicksum(var for k, var in table_type1_var_dic[first_node].items() if k[0]==(0,0) ) == 1)
            model.addCons(quicksum(var for k, var in table_type1_var_dic[last_but_2_node].items() if k[1]==(0,0) ) == 1)



    return model, (node_var_dic, type1_var_dic , table_type1_var_dic)


class TPG_MILP:
    def __init__(self, tpg):
        self.tpg = tpg
        self.model, (self.node_var_dic, self.type1_var_dic , self.table_type1_var_dic) = tpg2milp(tpg)
        logger.info("Variable number: %s"%len(self.model.getVars()))
        logger.info("Constraints number: %s"%len(self.model.getConss()))
        self.is_optimized = False
        self.node_t = None
        self.node_state = None
        self.opt_time = None

    def __prepare_state_data__(self):
        def find_in_dic(d):
            for k, b in d.items():
                if abs(self.model.getVal(b) - 1) < 0.01:
                    return k
            return None
        all_states = []
        for agent in self.tpg:
            tmp_states = [find_in_dic(self.table_type1_var_dic[prev.name]) for prev in agent[:-1]]
            for i, j in zip(tmp_states[: -1], tmp_states[1:]):
                if i[1] != j[0]:
                    print(i[1], j[0])
                    logger.warning("Found inconsistent node states in type 1 edge")
            all_states.append([tmp_states[0][0]] + [s[1] for s in tmp_states])
            self.node_state = all_states

    def __prepare_data__(self):
        if not self.is_optimized:
            return
        self.node_t = [[self.model.getVal(self.node_var_dic[i.name]) for i in agent] for agent in self.tpg.tpg]

        self.__prepare_state_data__()

    def optimize(self):
        start = time.time()
        self.model.optimize()
        self.opt_time = time.time() - start
        logger.info("Optimized in %s sec"%self.opt_time)

        self.is_optimized = True

        self.__prepare_data__()

        return self.model.getStatus()

    def self_check(self):
        if not self.is_optimized:
            return True

        return True


    def save(self, name):
        m = self.model
        node_var_dic, type1_var_dic , table_type1_var_dic = self.node_var_dic, self.type1_var_dic , self.table_type1_var_dic = None, None, None
        self.node_var_dic, self.type1_var_dic , self.table_type1_var_dic = None, None, None
        self.model = None
        with open(name, "wb") as f:
            pickle.dump(self, f)
        self.node_var_dic, self.type1_var_dic , self.table_type1_var_dic = node_var_dic, type1_var_dic , table_type1_var_dic
        self.model = m


    def get_plan(self):
        pass


    def plot_vp(self, agent):
        pass
