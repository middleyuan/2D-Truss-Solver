import numpy as np 

class Member:
    def __init__(self, member_index, node_i, node_j, shape, dimensions, e_module, yield_strength):
        """constructor of member

        Args:
            member_index (int): to track the index so as to build whole system equation
            node_i (Node): the node who the member is connected with
            node_j (Node): the node who the member is connected with
            shape (string): cross-section shape of the member
            dimensions (list): the parameters to describe the shape
            e_module (float): young's modulus 
            yield_strength (float): yield stress 
        """        
        self.member_index = member_index
        self._e_module = e_module
        self._yield_strength = yield_strength
        self.shape = shape
        self._dimensions = dimensions
        self.node_i = node_i
        self.node_j = node_j
        self._length = np.linalg.norm(np.array([node_i.x-node_j.x, node_i.y-node_j.y]))
        self._angle = np.arctan2(node_j.y-node_i.y, node_j.x-node_i.x)
        self.external_force = None
        self.failure = None

    def getAngle(self, analysed_joint):
        """compute the bar orientation centered on analysed joint

        Raises:
            ValueError: wrong connected node 

        Returns:
            angle: relative angle of the member w.r.t analysed joint 
        """        
        if analysed_joint == self.node_i.node_index:
            self._angle = np.arctan2(self.node_j.y-self.node_i.y, self.node_j.x-self.node_i.x)
        elif analysed_joint == self.node_j.node_index:
            self._angle = np.arctan2(self.node_i.y-self.node_j.y, self.node_i.x-self.node_j.x)
        else:
            raise ValueError('the member does not connect to', analysed_joint)
        return self._angle

    def fail(self):
        """check if the member fails either because of buckling or yielding

        Returns:
            failure: if the member fails 
        """        
        # compression
        if self.external_force < 0:
            # check if buckling
            self.critical_force = pow(np.pi, 2)*self._e_module*min(self.inertia_xx, self.inertia_yy)/pow(self._length, 2)
            if abs(self.external_force) > self.critical_force:
                self.buckling = True
                self.failure = True
                return self.failure
            else:
                self.failure = False
                return False
        elif self.external_force > 0:
            # check if yielding because the member is subject to tension force
            self.critical_force = self._yield_strength*self.area
            if abs(self.external_force) > self.critical_force:
                self.yielding = True
                self.failure = True
                return self.failure
            else:
                self.failure = False
                return False
        else:
            # no external force
            self.buckling = False
            self.yielding = False
            return self.failure
    def getI(self):
        """compute second moment of inertia
        """        
        pass
    def getA(self):
        """compute the cross-sectional area
        """
        pass

class CShapedMember(Member):
    def __init__(self, member_index, node_i, node_j, shape, dimensions, e_module, yield_strength):
        super().__init__(member_index, node_i, node_j, shape, dimensions, e_module, yield_strength)
        self.B = self._dimensions[0]
        self.H = self._dimensions[1]
        self.h = self._dimensions[2]
        self.b = self.h
        self.centroid_x = (2*self.h*pow(self.B, 2)/2+pow(self.b, 2)*self.H/2)/self.getA()
        self.getI()
        self.getA() 
    def getI(self):
        self.inertia_yy = pow(self.H, 3)*self.b/12+self.b*self.H*pow(self.centroid_x-self.b/2, 2)+2*pow(self.B, 3)*self.h/12+2*self.B*self.h*pow(self.centroid_x-self.B/2, 2) 
        self.inertia_xx = pow(self.H, 3)*self.b/12+2*(pow(self.h, 3)*self.B/12+self.h*self.B*pow(self.h+self.H, 2)/4)
        return [self.inertia_xx, self.inertia_yy]
    def getA(self):
        self.area = self.H*self.b+ 2*self.B*self.h
        return self.area
    

class TShapedMember(Member):
    def __init__(self, member_index, node_i, node_j, shape, dimensions, e_module, yield_strength):
        super().__init__(member_index, node_i, node_j, shape, dimensions, e_module, yield_strength)
        self.B = self._dimensions[0]
        self.H = self._dimensions[1]
        self.h = self._dimensions[2]
        self.b = self._dimensions[2]
        self.centroid_x = self.B/2
        self.centroid_y = ((self.H+self.h/2)*self.h*self.B+pow(self.H, 2)*self.b/2)/self.getA()
        self.getI()
        self.getA()     
    def getI(self):
        self.inertia_xx = self.b*self.H*pow(self.centroid_y-self.H/2, 2)+self.h*pow(self.H, 3)/12+self.h*self.B*pow(self.H+self.h/2-self.centroid_y, 2)+pow(self.h, 3)*self.B/12
        self.inertia_yy = pow(self.b, 3)*self.H/12+pow(self.B, 3)*self.h/12
        return [self.inertia_xx, self.inertia_yy]
    def getA(self):
        self.area = self.B*self.h + self.H*self.b
        return self.area

class IShapedMember(Member):
    def __init__(self, member_index, node_i, node_j, shape, dimensions, e_module, yield_strength):
        super().__init__(member_index, node_i, node_j, shape, dimensions, e_module, yield_strength)
        self.B = self._dimensions[0]
        self.H = self._dimensions[1]
        self.h = self._dimensions[2]
        self.b = self.h
        self.getI()
        self.getA() 
    def getI(self):
        self.inertia_xx = pow(self.H, 3)*self.b/12+2*(pow(self.h, 3)*self.B/12+self.h*self.B*pow(self.H+self.h, 2)/4)
        self.inertia_yy = pow(self.b, 3)*self.H/12 + 2*(pow(self.B, 3)*self.h/12)
        return [self.inertia_xx, self.inertia_yy]
    def getA(self):
        self.area = self.H*self.b + 2*self.B*self.h
        return self.area
class OShapedMember(Member):
    def __init__(self, member_index, node_i, node_j, shape, dimensions, e_module, yield_strength):
        super().__init__(member_index, node_i, node_j, shape, dimensions, e_module, yield_strength)
        self.d = self._dimensions[0]
        self.d1 = self.d - self._dimensions[1]
        self.getI()
        self.getA() 
    def getI(self):
        self.inertia_xx = np.pi/64*(pow(self.d, 4)-pow(self.d1, 4))
        self.inertia_yy = self.inertia_xx
        return [self.inertia_xx, self.inertia_yy]
    def getA(self):
        self.area = pow(self.d/2, 2)*np.pi-pow(self.d1/2, 2)*np.pi
        return self.area