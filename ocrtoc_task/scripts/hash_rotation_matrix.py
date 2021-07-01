import numpy as np

NUM_DECIMAL = 2

class HashableRotationMatrix():
    @classmethod
    def UnitRotationMatrix(cls):
        return cls(np.eye(3, dtype = np.float32))

    @classmethod
    def XRotationMatrix(cls, angle):
        r = np.array([[1, 0, 0],
                    [0, np.cos(angle), -np.sin(angle)],
                    [0, np.sin(angle), np.cos(angle)]], dtype = np.float32)
        return cls(r)
    
    @classmethod
    def YRotationMatrix(cls, angle):
        r = np.array([[np.cos(angle), 0, np.sin(angle)],
                    [0, 1, 0],
                    [-np.sin(angle), 0, np.cos(angle)]], dtype = np.float32)
        return cls(r)

    @classmethod
    def ZRotationMatrix(cls, angle):
        r = np.array([[np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0, 0, 1]], dtype = np.float32)
        return cls(r)

    def __init__(self, matrix):
        self.matrix = np.array(matrix, dtype = np.float32)
        
    def __hash__(self):
        return hash(tuple(np.around(self.matrix, NUM_DECIMAL).reshape(-1)))
    
    def __eq__(self, m):
        return hash(self) == hash(m)

    def __mul__(self, m):
        return HashableRotationMatrix(np.matmul(self.matrix, m.matrix))
    
    def __repr__(self):
        return 'hashabel_rotation_matrix:\n{}'.format(self.matrix)
    

if __name__ == '__main__':
    r_set = set()
    rx = 3
    ry = 2
    rz = 1
    for x in range(rx):
        for y in range(ry):
            for z in range(rz):
                x_angle = 2 * np.pi / rx * x
                y_angle = 2 * np.pi / ry * y
                z_angle = 2 * np.pi / rz * z
                r = HashableRotationMatrix.XRotationMatrix(x_angle) * HashableRotationMatrix.YRotationMatrix(y_angle) * HashableRotationMatrix.ZRotationMatrix(z_angle)
                r_set.add(r)
    print(len(r_set))
