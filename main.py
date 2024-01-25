import sys
from truss import Truss
from node import Node

def run(truss_definition, verbose):
    """post precess the txt file

    Args:
        truss_definition (path): .txt file path 

    Returns:
        list: the first element is the force vector, and the second element is the failure vector
    """    

    with open(truss_definition) as f:
        lines = f.read()

    # Extract file info into three parts
    # split the buffer by '----\n'
    infos = lines.split('----\n')
    node_info = infos[0]
    member_info = infos[1]
    force_info = infos[2]

    # init nodes
    node_info = node_info.split('\n')
    node_info = list(filter(None, node_info))
    node_dict = {}
    for i in range(len(node_info)):
        node_definition = node_info[i].split(' ')
        node_definition = list(filter(None, node_definition))
        node_name = node_definition[0]
        node_type = node_definition[1]
        # check if it contains angle info
        if len(node_definition) > 4:
            angle = float(node_definition[2])
            x = float(node_definition[3])
            y = float(node_definition[4])
        else:
            angle = 0
            x = float(node_definition[2])
            y = float(node_definition[3])
        
        # add a initialized node to dict
        node_dict[node_name] = Node(i, node_type, x, y, inclination=angle)

    # init member
    member_info = member_info.split('\n')
    member_info = list(filter(None, member_info))
    member_names = []
    for i in range(len(member_info)):
        member_definition = member_info[i].split(' ')       
        member_definition = list(filter(None, member_definition))
        member_names.append(member_definition[0])
        connectors = member_definition[0].split('-')
        if connectors[0] not in node_dict.keys() or connectors[1] not in node_dict.keys():
            raise ValueError('The member is connecting to the undefined node')
        shape = member_definition[1]
        if shape == 'O':
            if len(member_definition[2:]) != 4:
                raise ValueError('Please config the member in a correct way')
        elif shape == 'I' or shape == 'C' or shape == 'T':
            if len(member_definition[2:]) != 5:
                raise ValueError('Please config the member in a correct way')
        if shape == 'O':
            if len(member_definition[2:-2]) != 2:
                raise ValueError('The O-shape member should have 2 dimensions')
        elif shape == 'I' or shape == 'C' or shape == 'T':
            if len(member_definition[2:-2]) != 3:
                raise ValueError('The I-shape, C-shape, and T-shape member should have 3 dimensions')
        dimensions = [float(i) for i in member_definition[2:-2]]
        e_module = float(member_definition[-2])
        yield_strength = float(member_definition[-1])

        # add members to each node
        node_dict[connectors[0]].add_member(i, node_dict[connectors[0]], node_dict[connectors[1]], shape, dimensions, e_module, yield_strength)
        node_dict[connectors[1]].add_member(i, node_dict[connectors[1]], node_dict[connectors[0]], shape, dimensions, e_module, yield_strength)

    # init forces
    force_info = force_info.split('\n')
    force_info = list(filter(None, force_info))
    for i in range(len(force_info)):
        force_definition = force_info[i].split(' ')          
        force_definition = list(filter(None, force_definition))
        bearing_node = force_definition[0]
        if bearing_node not in node_dict.keys():
            raise ValueError('The external force is acting on the undefined node')
        magnitude = float(force_definition[1])
        angle = float(force_definition[2])

        # add the force to the specified node
        node_dict[bearing_node].add_force(magnitude, angle)

    # init truss
    truss_solver = Truss(node_dict, len(member_info))
    # compute the member axis forces, reaction forces, and failures
    truss_solver.solveForceEquations()

    if verbose:
        for i in range(len(member_names)):
            print('member', member_names[i], 'force =', truss_solver.unknown_forces[i], '; the member fails: ', truss_solver.failures[i])
        
        r = 0
        for i, key in enumerate(truss_solver.nodes):
            if truss_solver.nodes[key].r is not None:
                if truss_solver.nodes[key].type == 'Loose':
                    print('reaction force of node', key, '=', truss_solver.unknown_forces[len(member_info)+r, 0])
                    r = r + 1
                elif truss_solver.nodes[key].type == 'Fixed':
                    print('reaction force of node', key, 'in x direction =', truss_solver.unknown_forces[len(member_info)+r, 0])
                    print('reaction force of node', key, 'in y direction =', truss_solver.unknown_forces[len(member_info)+r+1, 0])
                    r = r + 2
    
    return [truss_solver.unknown_forces, truss_solver.failures]

if  __name__ == '__main__':
    # pass the .txt file as the first argument
    truss_definition = sys.argv[1]
    run(truss_definition, True)