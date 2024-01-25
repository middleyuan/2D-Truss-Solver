from member import IShapedMember, TShapedMember, CShapedMember, OShapedMember
import numpy as np

class Node:
    def __init__(self, node_index, node_type, x, y, inclination=0):
        """constructor for a node
        """        
        self.node_index = node_index
        self.type = node_type
        self.angle = inclination*np.pi/180
        # postion of the nodes
        self.x = x
        self.y = y
        self.members = []
        # external forces act on the node
        self.fx = 0
        self.fy = 0
    def add_member(self, member_index, node_i, node_j, member_type, dimensions, e_module, yield_strength):
        """add a bar member by connecting it with two initialized nodes

        Raises:
            ValueError: Wrong type excluding I, T, C, O
        """        
        if member_type == 'I':
            member = IShapedMember(member_index, node_i, node_j, member_type, dimensions, e_module, yield_strength)
        elif member_type == 'T':
            member = TShapedMember(member_index, node_i, node_j, member_type, dimensions, e_module, yield_strength)
        elif member_type == 'C':
            member = CShapedMember(member_index, node_i, node_j, member_type, dimensions, e_module, yield_strength)
        elif member_type == 'O':
            member = OShapedMember(member_index, node_i, node_j, member_type, dimensions, e_module, yield_strength)
        else:
            raise ValueError('Wrong type for members')
    
        self.members.append(member)

    def add_force(self, amp, angle):
        """specfic the force acting on the node

        Args:
            amplitude (float): the force amplitude acts on the node
            angle(deg) (float): the force angle acts on the node
        """
        self.fx = amp*np.cos(angle*np.pi/180) 
        self.fy = amp*np.sin(angle*np.pi/180)

    def buildForceEquations(self):
        """build forces equilibrium equation on the node

        Raises:
            ValueError: Wrong type for node exluding Fixed, Loose, and Free

        Returns:
            m: members involved coefficients
            r: reactions involved coefficients
            f: external forces vector
        """        
        
        # forces equilibrium on the node
        # mF' + f' = 0
        # F': local force vector contains involved members and reactions
        # f': external forces act on the node
        self.n = len(self.members)
        self.m = np.zeros((2, self.n))
        # reaction unknown depend on the type of the pinned node
        self.r = None
        self.f = np.array([self.fx, self.fy]).reshape(2, 1)

        # a fixed joint has two reaction forces
        if self.type == 'Fixed':
            for i in range(self.n):
                self.m[0, i] = np.cos(self.members[i].getAngle(self.node_index))
                self.m[1, i] = np.sin(self.members[i].getAngle(self.node_index))
                # 2 reactions
                self.r = np.identity(2)
        elif self.type == 'Loose':
            for i in range(self.n):
                self.m[0, i] = np.cos(self.members[i].getAngle(self.node_index))
                self.m[1, i] = np.sin(self.members[i].getAngle(self.node_index))
                # 1 reactions
                self.r = np.zeros((2, 1))
                self.r[0, 0] = np.cos(self.angle+np.pi/2)
                self.r[1, 0] = np.sin(self.angle+np.pi/2)
        elif self.type == 'Free':
            for i in range(self.n):
                self.m[0, i] = np.cos(self.members[i].getAngle(self.node_index))
                self.m[1, i] = np.sin(self.members[i].getAngle(self.node_index))
        else:
            raise ValueError('wrong type for nodes')

        return self.m , self.r, self.f