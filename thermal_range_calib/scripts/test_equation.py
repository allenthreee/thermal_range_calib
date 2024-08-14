import numpy as np

def check_solution(n, p1, p2):
    # Calculate the dot products
    dot_product1 = np.dot(n, p1)
    dot_product2 = np.dot(n, p2)

    # Check if the dot products are equal
    if dot_product1 == dot_product2:
        print("The dot products are equal. There might be multiple solutions.")
    else:
        print("The dot products are not equal. There is a unique solution.")

    # Check if p2 lies on the plane formed by n and p1
    if np.dot(n, p2 - p1) == 0:
        print("p2 lies on the plane formed by n and p1.")
    else:
        print("p2 does not lie on the plane formed by n and p1.")

# Define n, p1, and p2
n = np.array([1, 0, 1])
p1 = np.array([0, 0, 1])
p2 = np.array([1, 0, 0])

# Call the function to check the solutions
check_solution(n, p1, p2)

