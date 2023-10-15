
import random
import math 

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def draw_circle(radius, px, py):
    """
    Draw a circle with the given radius using Matplotlib.

    Args:
        radius (float): The radius of the circle.

    Returns:
        None
    """
    # Create a figure and axis
    fig, ax = plt.subplots()

    # Create a Circle object with the specified radius
    circle = Circle((0, radius), radius, fill=False, color='b')

    # Add the circle to the axis
    ax.add_patch(circle)


    # Create a Circle object with the specified radius
    point = Circle((px, py), radius/15.0, fill=True, color='r')
    # Add the circle to the axis
    ax.add_patch(point)

    # Set axis limits based on the circle's size
    # ax.set_xlim(-radius - 1, radius + 1)
    # ax.set_ylim(-radius - 1, radius + 1)
    ax.set_xlim(-radius * 2, radius * 2)
    ax.set_ylim(-radius * 2, radius * 2)

    # Set aspect ratio to be equal, so the circle looks like a circle
    ax.set_aspect('equal')

    # Display the plot
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(f'Circle with Radius {radius}')
    plt.grid(True)
    plt.show()


# # Example usage
# radius = 5.0
# draw_circle(radius)


# # Example usage
# radius = 2.0
# draw_circle(radius)


# # Example usage
# radius = 1.0
# draw_circle(radius)


def random_float_in_range(start, end):
    """
    Generate a random float within the given range [start, end).

    Args:
        start (float): The inclusive start of the range.
        end (float): The exclusive end of the range.

    Returns:
        float: A random float within the specified range.
    """
    if start >= end:
        raise ValueError("Start must be less than end.")
    return random.uniform(start, end)

# # Example usage:
# start_range = 1.0
# end_range = 10.0
# random_num = random_float_in_range(start_range, end_range)


def sample_point():
    x = random_float_in_range(0.05, 0.25)
    y = random_float_in_range(0.05, 0.12)
    return x, y




def calc_radius(x, y):
    # return math.sqrt(x*x + 2*y*y) - abs(y)
    return (x*x + y*y) / (2*abs(y))




# for _ in range(10):
#     x, y = sample_point()
#     r = calc_radius(x, y)
#     draw_circle(r, x, y)
x = 0.0614049 
y = -0.0324787
r = calc_radius(x, y)
draw_circle(r, abs(x), abs(y))