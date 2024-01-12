import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

class ExponentialControlBarrierFunction:
    def __init__(self):
        return

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

    # Define the barrier function h(x)
    def h(self, x):
        h = (x[0]-10)**2 + x[1]**2 - 5**2
        return h

    # grad_h = grad(h)
    def grad_h(self, x):
        grad = np.zeros_like(x)
        grad[0] = 2*(x[0]-10)
        grad[1] = 2*x[1]
        # grad[3] = 2*x[3]
        # grad[4] = 2*x[4]
        return grad

    # Define the safety constraint as a function
    def safety_constraint(self, u, x):
        # print(u, x)
        # Compute the Lie derivatives
        # Lf_h = np.dot(grad_h(x), f(x))
        # Lg_h = np.dot(grad_h(x), (g(x) @ u))
        A = 2*(x[0]-10)*u[3] + 2*x[1]*u[4]
        b = 2*(x[3]+x[4]) + 6*self.h(x) + 8*(2*((x[0]-10)*x[3]+x[1]*x[4]))
        
        # The safety constraint inequality is Lf_h + Lg_h*u + alpha(h(x)) >= 0
        # Define the class K function alpha(h(x)) as a simple linear function for this example
        # alpha = lambda h: 2 * h if h < 0 else h
        
        # Return the value of the safety constraint
        # print(Lf_h + Lg_h + h(x))
        # safety_value = Lf_h + Lg_h + h(x)
        return A+b

    # Define an optimization problem to find the control input u that satisfies the safety constraint
    def control_input_optimization(self, x_des, u_des):
        # Define the objective function to be minimized (e.g., a simple quadratic cost on u)
        objective = lambda u: np.sum((u - u_des)**2)
        
        # Define the constraints (we need to ensure that safety_constraint(x, u) >= 0)
        constraint = {'type': 'ineq', 'fun': self.safety_constraint, 'args': (x_des,)}

        x0 = np.random.randn(u_des.shape[0])
        # print(x0)
        
        # Solve the optimization problem
        u_opt = minimize(objective, x0, method="SLSQP", constraints=constraint, tol=1e-6, options={'maxiter': 1000})
        
        # print(self.safety_constraint(u_opt.x, x_des))
        # print(u_opt)
        # Return the optimal control input
        return u_opt.x




if __name__ == "__main__":
    ecbf = ExponentialControlBarrierFunction()
    # Example usage
    x_des = np.array([6, 1, 0, 1, 0, 0], dtype=np.float32)  # initial state of the system
    u_des = np.array([0, 0, 0, 1, 0, 0], dtype=np.float32)
    delta = 0.1

    path = []

    for i in range(50):
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
    circle = Circle((10, 0), 3, color='blue', fill=False)
    plt.gca().add_patch(circle)
    plt.show()