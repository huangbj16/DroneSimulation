import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import json
from cvxopt import matrix, solvers

class SafetyConstraint2D:
    def __init__(self, a1, a2, d1, d2, n, ds):
        self.a1 = a1
        self.a2 = a2
        self.d1 = d1
        self.d2 = d2
        self.k1 = 6
        self.k2 = 12
        self.n = n
        self.ds = ds
        self.point_center = np.array([d1, d2])
    
    def h(self, x):
        return ((x[0]-self.d1)/self.a1)**self.n + \
                ((x[1]-self.d2)/self.a2)**self.n - \
                self.ds
    
    def hd(self, x):
        return self.n*((x[0]-self.d1)/self.a1)**(self.n-1) * x[3] / self.a1 + \
                self.n*((x[1]-self.d2)/self.a2)**(self.n-1) * x[4] / self.a2
    
    def hdd_x(self, x):
        return (self.n * (self.n - 1) * ((x[0]-self.d1)/self.a1)**(self.n - 2) * x[3]**2 / self.a1**2) + \
                (self.n * (self.n - 1) * ((x[1]-self.d2)/self.a2)**(self.n - 2) * x[4]**2 / self.a2**2)

    def hdd_r(self, x):
        return [0,
                0,
                0,
                self.n*((x[0]-self.d1)/self.a1)**(self.n-1) / self.a1,
                self.n*((x[1]-self.d2)/self.a2)**(self.n-1) / self.a2,
                0]

    def calculate_A(self, x):
        A = self.hdd_r(x)
        return A
    
    def calculate_b(self, x):
        b = self.hdd_x(x) + self.k2*self.hd(x) + self.k1*self.h(x)
        return b
    
    def isInRange(self, x):
        point_vector = x[0:2] - self.point_center
        distance = np.linalg.norm(point_vector)
        if distance < (self.a1 + self.a2)/2 + 5.0:
            return True
        else:
            return False
    
    def updateParams(self, x):
        self.x = x
        self.A = self.calculate_A(x)
        self.b = self.calculate_b(x)

    def safety_constraint(self, u):
        # A = self.calculate_A(x)
        # b = self.calculate_b(x)
        # print("new A b = ", A, b)
        return np.dot(self.A, u) + self.b


class SafetyConstraint3D:
    def __init__(self, a1, a2, a3, d1, d2, d3, n, ds):
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.d1 = d1
        self.d2 = d2
        self.d3 = d3
        self.k1 = 6
        self.k2 = 6
        self.n = n
        self.ds = ds
        self.point_center = np.array([d1, d2, d3])
        # print("a1, d1 = ", self.a1, self.d1)
    
    def h(self, x):
        return ((x[0]-self.d1)/self.a1)**self.n + \
                ((x[1]-self.d2)/self.a2)**self.n + \
                ((x[2]-self.d3)/self.a3)**self.n - \
                self.ds
    
    def hd(self, x):
        return self.n*((x[0]-self.d1)/self.a1)**(self.n-1) * x[3] / self.a1 + \
                self.n*((x[1]-self.d2)/self.a2)**(self.n-1) * x[4] / self.a2 + \
                self.n*((x[2]-self.d3)/self.a3)**(self.n-1) * x[5] / self.a3
    
    def hdd_x(self, x):
        return (self.n * (self.n - 1) * ((x[0]-self.d1)/self.a1)**(self.n - 2) * x[3]**2 / self.a1**2) + \
                (self.n * (self.n - 1) * ((x[1]-self.d2)/self.a2)**(self.n - 2) * x[4]**2 / self.a2**2) + \
                (self.n * (self.n - 1) * ((x[2]-self.d3)/self.a3)**(self.n - 2) * x[5]**2 / self.a3**2)

    def hdd_r(self, x):
        return [0,
                0,
                0,
                self.n*((x[0]-self.d1)/self.a1)**(self.n-1) / self.a1,
                self.n*((x[1]-self.d2)/self.a2)**(self.n-1) / self.a2,
                self.n*((x[2]-self.d3)/self.a3)**(self.n-1) / self.a3
            ]

    def calculate_A(self, x):
        A = self.hdd_r(x)
        return A
    
    def calculate_b(self, x):
        b = self.hdd_x(x) + self.k2*self.hd(x) + self.k1*self.h(x)
        return b
    
    def isInRange(self, x):
        point_vector = x[0:3] - self.point_center
        distance = np.linalg.norm(point_vector)
        if distance < (self.a1 + self.a2 + self.a3)/3 + 5.0:
            return True
        else:
            return False
        
    def updateParams(self, x):
        self.x = x
        self.A = self.calculate_A(x)
        self.b = self.calculate_b(x)
    
    def safety_constraint(self, u):
        # A = self.calculate_A(x)
        # b = self.calculate_b(x)
        # print("new A b = ", A, b)
        return np.dot(self.A, u) + self.b


class SafetyConstraintWall3D:
    def __init__(self, a1, a2, a3, d1, d2, d3, s1, s2, s3, ort1, ort2):
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.d1 = d1
        self.d2 = d2
        self.d3 = d3
        self.s1 = s1
        self.s2 = s2
        self.s3 = s3
        self.plane_normal = np.array([self.a1, self.a2, self.a3])
        self.plane_point = np.array([self.d1, self.d2, self.d3])
        self.scale = np.array([self.s1, self.s2, self.s3])
        self.plane_ort1 = ort1
        self.plane_ort2 = ort2
        self.k1 = 6
        self.k2 = 6
        # print("a1, d1 = ", self.a1, self.d1)
    
    def h(self, x):
        return ((x[0]-self.d1)*self.a1) + \
                ((x[1]-self.d2)*self.a2) + \
                ((x[2]-self.d3)*self.a3)
    
    def hd(self, x):
        return x[3] * self.a1 + \
                x[4] * self.a2 + \
                x[5] * self.a3
    
    def hdd_x(self, x):
        return 0

    def hdd_r(self, x):
        return [0,
                0,
                0,
                self.a1,
                self.a2,
                self.a3
            ]

    def calculate_A(self, x):
        A = self.hdd_r(x)
        return A
    
    def calculate_b(self, x):
        b = self.hdd_x(x) + self.k2*self.hd(x) + self.k1*self.h(x)
        return b
    
    def isInRange(self, x):
        # Vector from point on plane to point in space
        point_vector = x[0:3] - self.plane_point
        # Distance from point to plane along the normal
        distance = np.dot(point_vector, self.plane_normal)
        # Projected point
        projected_point = x[0:3] - distance * self.plane_normal
        distance_ort1 = np.abs(np.dot(projected_point-self.plane_point, self.plane_ort1))
        distance_ort2 = np.abs(np.dot(projected_point-self.plane_point, self.plane_ort2))
        if distance_ort1 < self.scale[0] and distance_ort2 < self.scale[1]:
            return True
        else:
            return False
        
    def updateParams(self, x):
        self.x = x
        self.A = self.calculate_A(x)
        self.b = self.calculate_b(x)

    def safety_constraint(self, u):
        # A = self.calculate_A(x)
        # b = self.calculate_b(x)
        # print("new A b = ", A, b)
        return np.dot(self.A, u) + self.b



class ExponentialControlBarrierFunction:
    def __init__(self, safety_constraints):
        self.safety_constraint_list = safety_constraints

    def f(self, x):
        F = np.zeros((6, 6))
        identity_matrix = np.eye(3)
        F[:3, -3:] = identity_matrix
        return F @ x

    def g(self, x):
        G = np.zeros((6, 6))
        identity_matrix = np.eye(3)
        G[-3:, -3:] = identity_matrix
        return G

    # # Define the barrier function h(x)
    # def h(self, x):
    #     h = (x[0]-10)**2 + x[1]**2 - 4**2
    #     return h

    # # grad_h = grad(h)
    # def grad_h(self, x):
    #     grad = np.zeros_like(x)
    #     grad[0] = 2*(x[0]-10)
    #     grad[1] = 2*x[1]
    #     # grad[3] = 2*x[3]
    #     # grad[4] = 2*x[4]
    #     return grad

    # # Define the safety constraint as a function
    # def safety_constraint(self, u, x):
    #     # print(u, x)
    #     # Compute the Lie derivatives
    #     # Lf_h = np.dot(grad_h(x), f(x))
    #     # Lg_h = np.dot(grad_h(x), (g(x) @ u))
    #     A = [2*(x[0]-10), 2*x[1]]
        
    #     b = 2*(x[3]**2+x[4]**2) + 6*self.h(x) + 8*(2*((x[0]-10)*x[3]+x[1]*x[4]))
        
    #     # The safety constraint inequality is Lf_h + Lg_h*u + alpha(h(x)) >= 0
    #     # Define the class K function alpha(h(x)) as a simple linear function for this example
    #     # alpha = lambda h: 2 * h if h < 0 else h
        
    #     # Return the value of the safety constraint
    #     # print(Lf_h + Lg_h + h(x))
    #     # safety_value = Lf_h + Lg_h + h(x)
    #     print("old A b = ", A, b)
    #     return np.dot(A, u[3:5])+b

    # Define an optimization problem to find the control input u that satisfies the safety constraint
    def control_input_optimization(self, x_des, u_des):
        # print("u shape = ", u_des.shape)
        ### scipy minimize
        # # Define the objective function to be minimized (e.g., a simple quadratic cost on u)
        # objective = lambda u: np.sum((u - u_des)**2)
        
        # # Define the constraints (we need to ensure that safety_constraint(x, u) >= 0)
        # # constraints = []
        # # for sc in self.safety_constraint_list:
        # #     if sc.isInRange(x_des):
        # #         sc.updateParams(x_des)
        # #         constraints.append({'type': 'ineq', 'fun': sc.safety_constraint})
        # # constraints = [{'type': 'ineq', 'fun': sc.safety_constraint, 'args': (x_des,)} for sc in self.safety_constraint_list]
        # # print("# of constraint = ", len(constraints))

        # x0 = np.random.randn(u_des.shape[0])
        # # print(x0)
        
        # # Solve the optimization problem
        # u_opt = minimize(objective, x0, method="SLSQP", constraints=constraints, tol=1e-6, options={'maxiter': 1000})

        # # print("ref safety = ", self.safety_constraint_list[0].safety_constraint(u_des, x_des))
        # # print("alt safety = ", self.safety_constraint_list[0].safety_constraint(u_opt.x, x_des))
        # # print(u_opt)
        # # Return the optimal control input
        # return u_opt.x, u_opt.success
    
        ### QP Solver
        solvers.options['show_progress'] = False
        Q = matrix(np.identity(u_des.shape[0]), tc='d')
        p = matrix(-u_des.T, tc='d')
        A_array = []
        b_array = []
        for sc in self.safety_constraint_list:
            if sc.isInRange(x_des):
                sc.updateParams(x_des)
                A_array.append(sc.A)
                b_array.append(sc.b)
        G = matrix(-np.stack(A_array, axis=0), tc='d')
        h = matrix(np.stack(b_array, axis=0), tc='d')
        # print("G h shape = ", G.size, h.size)
        sol = solvers.qp(Q, p, G, h)
        res = np.array(sol['x']).reshape(u_des.shape[0])
        # print(res.shape)
        return res, True




if __name__ == "__main__":
    filepath = 'D:/2023Fall/DroneSimulation/TestScene/WindowsNoEditor/Blocks/Content/Settings/cubes.txt'

    def read_cubes_file(file_path):
        cubes = []
        with open(file_path, 'r') as file:
            for line in file:
                cubes.append(json.loads(line.strip()))
        return cubes

    cubes_data = read_cubes_file(filepath)

    # epsd1 = SafetyConstraint2D(1.0, 1.0, 10.0, 2.0, 2.0, 9.0)
    # epsd2 = SafetyConstraint2D(1.0, 1.0, 20.0, -4.0, 2.0, 9.0)
    # epsd3 = SafetyConstraint2D(1.0, 1.0, 30.0, 2.0, 2.0, 4.0)
    # epsd4 = SafetyConstraint2D(1.0, 1.0, 40.0, -1.0, 2.0, 4.0)
    obstacles = []
    for cube in cubes_data:
        print(cube)
        obstacles.append(SafetyConstraint3D(cube['ScaleX'], cube['ScaleY'], cube['ScaleZ'], cube['LocationX']/100, cube['LocationY']/100, cube['LocationZ']/100, 4, 16))
    ecbf = ExponentialControlBarrierFunction(obstacles)

    # cube = SafetyConstraint3D(1.0, 1.0, 1.0, 5.0, 0.0, 2.0, 4, 16)
    # ecbf = ExponentialControlBarrierFunction([cube])
    # Example usage
    x_des = np.array([0, 1, 2, 0, 0, 0], dtype=np.float32)  # initial state of the system
    u_des = np.array([0, 0, 0, 5, 0, 0], dtype=np.float32)
    delta = 0.1

    path = []

    for i in range(100):
        # Find the optimal control input that keeps the system within the safe set
        u_safe, success = ecbf.control_input_optimization(x_des, u_des)
        u_safe = np.round(u_safe, 3)
        print(f"Optimal control input: {u_safe}")
        x_dot = ecbf.f(x_des) + ecbf.g(x_des) @ u_safe
        x_des = x_des + x_dot * delta
        x_des = np.round(x_des, 3)
        path.append(x_des[:3])
        print(f"New state vector: {x_des}\n")

    path = np.array(path)

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

    ax.scatter(path[:, 0], path[:, 1], path[:, 2], c = np.arange(0, 1, step = 1.0/path.shape[0]), cmap=cm.coolwarm)

    # Make data.
    for obs in obstacles:
        X = np.arange(obs.d1-2, obs.d1+2, 0.1)
        Y = np.arange(obs.d2-2, obs.d2+2, 0.1)
        X, Y = np.meshgrid(X, Y)
        Z = (16 - (X-obs.d1)**4 - (Y-obs.d2)**4)**(0.25)
        Z1 = obs.d3 + Z
        Z2 = obs.d3 - Z
        # Plot the surface.
        surf = ax.plot_surface(X, Y, Z1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        surf = ax.plot_surface(X, Y, Z2, cmap=cm.coolwarm, linewidth=0, antialiased=False)
    # Customize the z axis.
    # ax.set_zlim(-0.01, 4.01)
    ax.zaxis.set_major_locator(LinearLocator(10))
    # A StrMethodFormatter is used automatically
    ax.zaxis.set_major_formatter('{x:.02f}')

    # Add a color bar which maps values to colors.
    # fig.colorbar(surf, shrink=0.5, aspect=5)

    plt.show()




    # epsd1 = SafetyConstraint2D(1.0, 1.0, 10.0, 2.0, 2.0, 9.0)
    # epsd2 = SafetyConstraint2D(1.0, 1.0, 20.0, -4.0, 2.0, 9.0)
    # epsd3 = SafetyConstraint2D(1.0, 1.0, 30.0, 2.0, 2.0, 4.0)
    # epsd4 = SafetyConstraint2D(1.0, 1.0, 40.0, -1.0, 2.0, 4.0)
    # ecbf = ExponentialControlBarrierFunction([epsd1, epsd2, epsd3, epsd4])
    # # Example usage
    # x_des = np.array([5, 0, 0, 1, 0, 0], dtype=np.float32)  # initial state of the system
    # u_des = np.array([0, 0, 0, 1, 0, 0], dtype=np.float32)
    # delta = 0.1

    # path = []

    # for i in range(200):
    #     # Find the optimal control input that keeps the system within the safe set
    #     u_safe = ecbf.control_input_optimization(x_des, u_des)
    #     u_safe = np.round(u_safe, 3)
    #     print(f"Optimal control input: {u_safe}")
    #     x_dot = ecbf.f(x_des) + ecbf.g(x_des) @ u_safe
    #     x_des = x_des + x_dot * delta
    #     x_des = np.round(x_des, 3)
    #     path.append(x_des[:2])
    #     print(f"New state vector: {x_des}\n")

    # from matplotlib.patches import Circle

    # path = np.array(path)
    # plt.scatter(path[:, 0], path[:, 1])
    # circles = [Circle((10, 2), 3, color='blue', fill=False),
    #     Circle((20, -4), 3, color='blue', fill=False),
    #     Circle((30, 2), 2, color='blue', fill=False),
    #     Circle((40, -1), 2, color='blue', fill=False)
    # ]
    # for circle in circles:
    #     plt.gca().add_patch(circle)
    # plt.show()