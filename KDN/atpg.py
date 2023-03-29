from tpg import TPG

def get_agent(node):
    name = node.name
    return int(name.split("^")[1].split("_")[0])

def uniform_agent_aug_tpg(tpg, dp=None, dp_safe=None):
    """
    a           ->          b           ->         c
    a -> post_a -> pre_b -> b -> post_b -> pre_c ->c

    Transfer TPG to augmented TPG and also assign dp table to each type 1 edge

    """

    atpg = TPG(len(tpg))
    for idx, agent in enumerate(tpg):
        if len(agent) > 1:
            node_name = agent[0].name
            atpg.append_node(idx, node_name)
            atpg.append_node(idx, "post_" + node_name)

            atpg.get_node(node_name).set_delta(dp_safe)
            atpg.get_node("post_" + node_name).set_delta(dp)

            for node in agent[1:-1]:
                node_name = node.name
                atpg.append_node(idx, "pre_" + node_name)
                atpg.append_node(idx, node_name)
                atpg.append_node(idx, "post_" + node_name)

                atpg.get_node("pre_" + node_name).set_delta(dp_safe)
                atpg.get_node(node_name).set_delta(dp_safe)
                atpg.get_node("post_" + node_name).set_delta(dp)

            node_name = agent[-1].name
            atpg.append_node(idx, "pre_" + node_name)
            atpg.get_node("pre_" + node_name).set_delta(dp_safe)
            atpg.append_node(idx, node_name)
        else:
            atpg.append_node(idx, agent[0].name)

    for node_0, node_1 in tpg.get_type2_edges():
        atpg.add_type2_edge("post_" + node_0, "pre_" + node_1)


    # for agent in tpg.tpg:
    #     for node in agent:
    #         if node.type2:
    #             edge_list = []
    #             # agent_0 = get_agent(node)
    #             agent_1 = get_agent(node.type2)

    #             while get_agent(node.type2) == agent_1:
    #                 edge_list.append(node)
    #                 node = node.next

    #             if (len(node) == 1):
    #                 # single point order
    #                 pass

    #             else:
    #                 # full edge following
    #                 pass

    return atpg
