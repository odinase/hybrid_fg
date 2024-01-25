import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

def visualize_gaussian_mixture():
    w = np.array([1.0, 3.0])
    w = w / w.sum()

    m1, s1 = 1.0, 0.5
    m2, s2 = 3.0, 0.2
    p1 = norm(loc=m1, scale=s1)
    p2 = norm(loc=m2, scale=s2)

    x = np.linspace(m1 - 3*s1, m2 + 3*s2, 1000)
    w1, w2 = w
    y = w1*p1.pdf(x) + w2*p2.pdf(x)

    fig, ax = plt.subplots()
    ax.plot(x, y)

    plt.show()
    