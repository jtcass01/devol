from typing import Dict
from numpy import cross
from sympy import symbols, Matrix, pprint, simplify, cos, sin, pi, zeros, latex

def solve():
    theta, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, d, a, alpha = symbols('theta theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 d a alpha')

    A_definition: Matrix = Matrix([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

    local_homogenous_transforms: Dict[str, Matrix] = {
        'A_1': A_definition.subs({theta: theta_1, d: 0.2363, a: 0, alpha: pi / 2.}),
        'A_2': A_definition.subs({theta: theta_2, d: 0, a: -0.8620, alpha: 0}),
        'A_3': A_definition.subs({theta: theta_3, d: 0, a: -0.7287, alpha: 0}),
        'A_4': A_definition.subs({theta: theta_4, d: 0.2010, a: 0, alpha: pi / 2.}),
        'A_5': A_definition.subs({theta: theta_5, d: 0.1593, a: 0, alpha: -pi / 2.}),
        'A_6': A_definition.subs({theta: theta_6, d: 0.1543, a: 0, alpha: 0}),
    }

    T_1  = simplify(local_homogenous_transforms['A_1'])
    T_2  = simplify(T_1 @ local_homogenous_transforms['A_2'])
    T_3  = simplify(T_2 @ local_homogenous_transforms['A_3'])
    T_4  = simplify(T_3 @ local_homogenous_transforms['A_4'])
    T_5  = simplify(T_4 @ local_homogenous_transforms['A_5'])
    T_6  = simplify(T_5 @ local_homogenous_transforms['A_6'])

    R_1 = T_1[:3, :3]
    R_2 = T_2[:3, :3]
    R_3 = T_3[:3, :3]
    R_4 = T_4[:3, :3]
    R_5 = T_5[:3, :3]
    R_6 = T_6[:3, :3]

    # Center of mass are taken from documentation
    O_c_1: Matrix = Matrix([[0], [-0.0610], [0.0062], [1]])
    O_c_2: Matrix = Matrix([[0.5226], [0], [0.2098], [1]])
    O_c_3: Matrix = Matrix([[0.3234], [0], [0.0604], [1]])
    O_c_4: Matrix = Matrix([[0], [-0.0026], [0.0393], [1]])
    O_c_5: Matrix = Matrix([[0], [0.0024], [0.0379], [1]])
    O_c_6: Matrix = Matrix([[0], [-0.0003], [-0.0318], [1]])

    COM_1: Matrix = Matrix([[0], [0], [0.2363/2], [1]])[:3, :]
    COM_2 = (T_1 @ O_c_1)[:3, :]
    COM_3 = (T_2 @ O_c_2)[:3, :]
    COM_4 = (T_3 @ O_c_3)[:3, :]
    COM_5 = (T_4 @ O_c_4)[:3, :]
    COM_6 = (T_5 @ O_c_5)[:3, :]

    O_0 = Matrix([0, 0, 0])
    O_1 = T_1[0:3, 3]
    O_2 = T_2[0:3, 3]
    O_3 = T_3[0:3, 3]
    O_4 = T_4[0:3, 3]
    O_5 = T_5[0:3, 3]

    z_0 = Matrix([0, 0, 1])
    z_1 = T_1[0:3, 2]
    z_2 = T_2[0:3, 2]
    z_3 = T_3[0:3, 2]
    z_4 = T_4[0:3, 2]
    z_5 = T_5[0:3, 2]

    Jvc0_0 = simplify(Matrix(cross(z_0.T, COM_1.T - O_0.T)))
    Jvc0 = Matrix([
        Jvc0_0,
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3)
    ]).T

    Jvc1_0 = simplify(Matrix(cross(z_0.T, COM_2.T - O_0.T)))
    Jvc1_1 = simplify(Matrix(cross(z_1.T, COM_2.T - O_1.T)))
    Jvc1 = Matrix([
        Jvc1_0,
        Jvc1_1,
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3)
    ]).T

    Jvc2_0 = simplify(Matrix(cross(z_0.T, COM_3.T - O_0.T)))
    Jvc2_1 = simplify(Matrix(cross(z_1.T, COM_3.T - O_1.T)))
    Jvc2_2 = simplify(Matrix(cross(z_2.T, COM_3.T - O_2.T)))
    Jvc2 = Matrix([
        Jvc2_0,
        Jvc2_1,
        Jvc2_2,
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3)
    ]).T

    Jvc3_0 = simplify(Matrix(cross(z_0.T, COM_4.T - O_0.T)))
    Jvc3_1 = simplify(Matrix(cross(z_1.T, COM_4.T - O_1.T)))
    Jvc3_2 = simplify(Matrix(cross(z_2.T, COM_4.T - O_2.T)))
    Jvc3_3 = simplify(Matrix(cross(z_3.T, COM_4.T - O_3.T)))
    Jvc3 = Matrix([
        Jvc3_0,
        Jvc3_1,
        Jvc3_2,
        Jvc3_3,
        zeros(1, 3),
        zeros(1, 3)
    ]).T

    Jvc4_0 = simplify(Matrix(cross(z_0.T, COM_5.T - O_0.T)))
    Jvc4_1 = simplify(Matrix(cross(z_1.T, COM_5.T - O_1.T)))
    Jvc4_2 = simplify(Matrix(cross(z_2.T, COM_5.T - O_2.T)))
    Jvc4_3 = simplify(Matrix(cross(z_3.T, COM_5.T - O_3.T)))
    Jvc4_4 = simplify(Matrix(cross(z_4.T, COM_5.T - O_4.T)))
    Jvc4 = Matrix([
        Jvc4_0,
        Jvc4_1,
        Jvc4_2,
        Jvc4_3,
        Jvc4_4,
        zeros(1, 3)
    ]).T

    Jvc5_0 = simplify(Matrix(cross(z_0.T, COM_6.T - O_0.T)))
    Jvc5_1 = simplify(Matrix(cross(z_1.T, COM_6.T - O_1.T)))
    Jvc5_2 = simplify(Matrix(cross(z_2.T, COM_6.T - O_2.T)))
    Jvc5_3 = simplify(Matrix(cross(z_3.T, COM_6.T - O_3.T)))
    Jvc5_4 = simplify(Matrix(cross(z_4.T, COM_6.T - O_4.T)))
    Jvc5_5 = simplify(Matrix(cross(z_5.T, COM_6.T - O_5.T)))
    Jvc5 = Matrix([
        Jvc5_0,
        Jvc5_1,
        Jvc5_2,
        Jvc5_3,
        Jvc5_4,
        Jvc5_5
    ]).T

    Jwc0 = Matrix([
        z_0.T,
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3)
    ]).T

    Jwc1 = Matrix([
        z_0.T,
        z_1.T,
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3)
    ]).T

    Jwc2 = Matrix([
        z_0.T,
        z_1.T,
        z_2.T,
        zeros(1, 3),
        zeros(1, 3),
        zeros(1, 3)
    ]).T

    Jwc3 = Matrix([
        z_0.T,
        z_1.T,
        z_2.T,
        z_3.T,
        zeros(1, 3),
        zeros(1, 3)
    ]).T

    Jwc4 = Matrix([
        z_0.T,
        z_1.T,
        z_2.T,
        z_3.T,
        z_4.T,
        zeros(1, 3)
    ]).T

    Jwc5 = Matrix([
        z_0.T,
        z_1.T,
        z_2.T,
        z_3.T,
        z_4.T,
        z_5.T
    ]).T

    Jc0 = Matrix([
        Jvc0,
        Jwc0
    ])

    Jc1 = Matrix([
        Jvc1,
        Jwc1
    ])

    Jc2 = Matrix([
        Jvc2,
        Jwc2
    ])

    Jc3 = Matrix([
        Jvc3,
        Jwc3
    ])

    Jc4 = Matrix([
        Jvc4,
        Jwc4
    ])

    Jc5 = Matrix([
        Jvc5,
        Jwc5
    ])

    for Jc_index, Jc in enumerate([Jc0, Jc1, Jc2, Jc3, Jc4, Jc5]):
        print(f"\nJc_{Jc_index+1}:")
        pprint(Jc)

        with open(f"Jc_{Jc_index+1}.tex", "w") as f:
            f.write(r"\documentclass{article}" + "\n")
            f.write(r"\usepackage{amsmath}" + "\n")
            f.write(r"\begin{document}" + "\n")
            f.write(r"\[\text{Transformation Matrix: }" + latex(Jc) + r"\]" + "\n")
            f.write(r"\end{document}" + "\n")

    # kg
    m_1 = 16.343
    I_1 = Matrix([
        [0.0887, -0.0001, -0.0001],
        [-0.0001, 0.0763, 0.0072],
        [-0.0001, 0.0072, 0.0842]
    ])

    m_2 = 29.632
    I_2 = Matrix([
        [0.1467, 0.0002, -0.0516],
        [0.0002, 4.6659, 0.0000],
        [-0.0516, 0.0000, 4.6348]
    ])

    m_3 = 7.8
    I_3 = Matrix([
        [0.0261, -0.0001, -0.0290],
        [-0.0001, 0.75763, 0],
        [-0.0290, 0, 0.7533]
    ])

    m_4 = 3.054
    I_4 = Matrix([
        [0.0056, 0, 0],
        [0, 0.0054, 0.0004],
        [0, 0.0004, 0.0040]
    ])

    m_5 = 3.126
    I_5 = Matrix([
        [0.0059, 0, 0],
        [0, 0.0058, -0.0004],
        [0, -0.0004, 0.0043]
    ])

    m_6 = 0.846
    I_6 = Matrix([
        [0.0009, 0, 0],
        [0, 0.0009, 0],
        [0, 0, 0.0012]
    ])

    D_0 = simplify(m_1 * Jvc0[:3, :].T @ Jvc0[:3, :] + Jwc0[:3, :].T @ R_1 @ I_1 @ R_1.T @ Jwc0[:3, :])
    D_1 = simplify(m_2 * Jvc1[:3, :].T @ Jvc1[:3, :] + Jwc1[:3, :].T @ R_2 @ I_2 @ R_2.T @ Jwc1[:3, :])
    D_2 = simplify(m_3 * Jvc2[:3, :].T @ Jvc2[:3, :] + Jwc2[:3, :].T @ R_3 @ I_3 @ R_3.T @ Jwc2[:3, :])
    D_3 = simplify(m_4 * Jvc3[:3, :].T @ Jvc3[:3, :] + Jwc3[:3, :].T @ R_4 @ I_4 @ R_4.T @ Jwc3[:3, :])
    D_4 = simplify(m_5 * Jvc4[:3, :].T @ Jvc4[:3, :] + Jwc4[:3, :].T @ R_5 @ I_5 @ R_5.T @ Jwc4[:3, :])
    D_5 = simplify(m_6 * Jvc5[:3, :].T @ Jvc5[:3, :] + Jwc5[:3, :].T @ R_6 @ I_6 @ R_6.T @ Jwc5[:3, :])

    D = simplify(D_0 + D_1 + D_2 + D_3 + D_4 + D_5)
    print("D:")
    pprint(D.evalf(4))

    c = zeros(6, 6)
    thetas = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]
    theta_dots = symbols('theta_dot_1:7')
    for i in range(6):
        for j in range (6):
            c_ij = 0
            for k in range(6):
                c_ijk = (D[i, j].diff(thetas[k]) + D[i, k].diff(thetas[j]) - D[j, k].diff(thetas[i])) / 2
                c_ij += simplify(c_ijk * theta_dots[k])
            c[i, j] = c_ij
            print(f"c[{i}, {j}] = {c[i, j]}")
    print("c:")
    pprint(c)

    P_1 = m_1 * 9.81 * COM_1[2]
    P_2 = m_2 * 9.81 * COM_2[2]
    P_3 = m_3 * 9.81 * COM_3[2]
    P_4 = m_4 * 9.81 * COM_4[2]
    P_5 = m_5 * 9.81 * COM_5[2]
    P_6 = m_6 * 9.81 * COM_6[2]
    P = P_1 + P_2 + P_3 + P_4 + P_5 + P_6
    print("P:")
    pprint(P)

    g = Matrix([ simplify(P.diff(theta)) for theta in thetas ])

    print("\n\n\ng(q):")
    pprint(g)

    with open("robot_dynamics.tex","w") as f:
        f.write(r"\documentclass{article}"+"\n")
        f.write(r"\usepackage{amsmath}"+"\n")
        f.write(r"\begin{document}"+"\n")
        f.write(r"\section*{Inertia Matrix}"+"\n")
        f.write(r"\[ D = " + latex(D.evalf(4)) + r"\]"+"\n")
        f.write(r"\section*{Coriolis Matrix}"+"\n")
        f.write(r"\[ C = " + latex(c.evalf(4)) + r"\]"+"\n")
        f.write(r"\section*{Gravity Terms}"+"\n")
        f.write(r"\[ g(q) = " + latex(g.evalf(4)) + r"\]"+"\n")
        f.write(r"\end{document}")

if __name__ == "__main__":
    solve()