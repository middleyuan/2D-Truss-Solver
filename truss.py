from node import Node
import numpy as np

class Truss:
    def __init__(self, nodes, number_of_member):
        """constructor of Truss

        Args:
            nodes (dict): truss definition by a set of nodes
            number_of_member (int): total number of the bars
        """        

        self.number_of_member = number_of_member
        self.nodes = nodes
    def solveForceEquations(self):
        """gather equations of each nodes to form system equation

        Raises:
            ValueError: system is not deterministic
        """        

        self.n = len(self.nodes.keys())

        for key, item in self.nodes.items():
            item.buildForceEquations()

        # compute number of reastions
        self.number_of_reactions = 0
        for key, item in self.nodes.items():
            if item.type == 'Fixed':
                # Fixed pinned nodes have both x and y reactions
                self.number_of_reactions = self.number_of_reactions + 2
            elif item.type == 'Loose':
                # Loose pinned nodes only have reactions perpendicular to the ground
                self.number_of_reactions = self.number_of_reactions + 1
            else:
                self.number_of_reactions = self.number_of_reactions 
        # check if system is deterministic: 2*n = m(members) + r(reactions)
        if len(self.nodes)*2 != self.number_of_member + self.number_of_reactions:
            raise ValueError('system is not deterministic')

        # create system matrices: AF + f = 0
        # vector F contains forces vriables: 2*n(number of nodes) bar forces followed by reaction forces in initialization order
        # vector f is a set of external forces acting on the corresponding nodes in initialization order
        self._A = np.zeros((2*len(self.nodes), 2*len(self.nodes)))
        self._f = np.zeros((2*len(self.nodes), 1))
        # fill out the coefficient of matrix A
        r_index = 0
        for i, key in enumerate(self.nodes):
            m_indexes = []
            # append the member index to the indexes list
            for j in range(len(self.nodes[key].members)):
                m_indexes.append(self.nodes[key].members[j].member_index)
            # member forces coefficients
            self._A[2*i:2*i+2, m_indexes] = self.nodes[key].m
            # reaction forces coefficients
            if self.nodes[key].type == 'Fixed':
                self._A[2*i:2*i+2, self.number_of_member+r_index:self.number_of_member+r_index+2] = self.nodes[key].r
                r_index = r_index + 2
            elif self.nodes[key].type == 'Loose':
                self._A[2*i:2*i+2, self.number_of_member+r_index:self.number_of_member+r_index+1] = self.nodes[key].r
                r_index = r_index + 1 
            # external forces coefficients
            self._f[2*i:2*i+2, [0]] = self.nodes[key].f

        self._A_pinv = np.linalg.pinv(self._A)
        # solve vector F
        self.unknown_forces = np.matmul(self._A_pinv, -self._f)

        # return the forces back to the members to determine failure
        # more efficient implementations exist, feel free to try out
        self.failures = []
        self._visited_member = np.zeros((self.number_of_member,1), dtype=bool)
        r = 0
        for i, key in enumerate(self.nodes):
            for j in range(len(self.nodes[key].members)):
                for k in range(self.number_of_member):
                    if k == self.nodes[key].members[j].member_index:
                        # from the node's point of view, member forces which pull the node are considered as positive force
                        # now change to the member's viewpoint, the force which pushes the member is considered as negative
                        # putting the minor sign because the force is acting on the member
                        self.nodes[key].members[j].external_force = -self.unknown_forces[k, 0]
                        fail = self.nodes[key].members[j].fail()
                        if self._visited_member[k] == False:
                            self.failures.append(fail)
                            self._visited_member[k] = True

        return
