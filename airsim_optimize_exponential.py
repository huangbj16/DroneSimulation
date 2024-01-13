import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

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
    
    def safety_constraint(self, u, x):
        A = self.calculate_A(x)
        b = self.calculate_b(x)
        # print("new A b = ", A, b)
        return np.dot(A, u) + b


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
        # Define the objective function to be minimized (e.g., a simple quadratic cost on u)
        objective = lambda u: np.sum((u - u_des)**2)
        
        # Define the constraints (we need to ensure that safety_constraint(x, u) >= 0)
        constraints = [{'type': 'ineq', 'fun': sc.safety_constraint, 'args': (x_des,)} for sc in self.safety_constraint_list]

        x0 = np.random.randn(u_des.shape[0])
        # print(x0)
        
        # Solve the optimization problem
        u_opt = minimize(objective, x0, method="SLSQP", constraints=constraints, tol=1e-6, options={'maxiter': 1000})
        
        # print(self.safety_constraint(u_opt.x, x_des))
        # print(u_opt)
        # Return the optimal control input
        return u_opt.x




if __name__ == "__main__":
    epsd1 = SafetyConstraint2D(1.0, 1.0, 10.0, 2.0, 2.0, 9.0)
    epsd2 = SafetyConstraint2D(1.0, 1.0, 20.0, -4.0, 2.0, 9.0)
    epsd3 = SafetyConstraint2D(1.0, 1.0, 30.0, 2.0, 2.0, 4.0)
    epsd4 = SafetyConstraint2D(1.0, 1.0, 40.0, -1.0, 2.0, 4.0)
    ecbf = ExponentialControlBarrierFunction([epsd1, epsd2, epsd3, epsd4])
    # Example usage
    x_des = np.array([5, 0, 0, 1, 0, 0], dtype=np.float32)  # initial state of the system
    u_des = np.array([0, 0, 0, 1, 0, 0], dtype=np.float32)
    delta = 0.1

    path = []

    for i in range(200):
        # Find the optimal control input that keeps the system within the safe set
        u_safe = ecbf.control_input_optimization(x_des, u_des)
        u_safe = np.round(u_safe, 3)
        print(f"Optimal control input: {u_safe}")
        x_dot = ecbf.f(x_des) + ecbf.g(x_des) @ u_safe
        x_des = x_des + x_dot * delta
        x_des = np.round(x_des, 3)
        path.append(x_des[:2])
        print(f"New state vector: {x_des}\n")

    from matplotlib.patches import Circle

    path = np.array(path)
    plt.scatter(path[:, 0], path[:, 1])
    circles = [Circle((10, 2), 3, color='blue', fill=False),
        Circle((20, -4), 3, color='blue', fill=False),
        Circle((30, 2), 2, color='blue', fill=False),
        Circle((40, -1), 2, color='blue', fill=False)
    ]
    for circle in circles:
        plt.gca().add_patch(circle)
    plt.show()