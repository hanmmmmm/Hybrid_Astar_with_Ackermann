
import math
import matplotlib.pyplot as plt

def draw_lines(points_list):
    """
    Draw lines based on a list of points using Matplotlib.

    Args:
        points_list (list of tuples): A list of tuples, where each tuple contains two points (x, y).

    Returns:
        None
    """
    fig, ax = plt.subplots()

    for line_points in points_list:
        x, y = zip(*line_points)  # Separate x and y coordinates
        ax.plot(x, y, marker='o')

    ax.set_aspect('equal')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Lines with 2 Points Each')
    plt.grid(True)
    plt.show()

# # Example usage:
# points_list = [((1, 2), (4, 5)), ((2, 4), (6, 8)), ((-1, -1), (3, 7))]
# draw_lines(points_list)



def move_points(m_p1x_, m_p1y_, m_p2x_, m_p2y_):
    p2x = m_p2x_ - m_p1x_
    p2y = m_p2y_ - m_p1y_
    p1x = m_p1x_ - m_p1x_
    p1y = m_p1y_ - m_p1y_
    return p1x, p1y, p2x, p2y


def rotate(_p2x, _p2y, theta):
    # _p2x = math.cos(theta) * _p2x + math.sin(theta) * _p2y
    # _p2y = -math.sin(theta) * _p2x + math.cos(theta) * _p2y

    _p2xrrr = math.cos(theta) * _p2x - math.sin(theta) * _p2y
    _p2yrrr = math.sin(theta) * _p2x + math.cos(theta) * _p2y
    return _p2xrrr, _p2yrrr


def process(m_p1x_, m_p1y_, m_p1t, m_p2x_, m_p2y_):
    
    line_original = ((m_p1x_, m_p1y_), (m_p2x_, m_p2y_))

    p1x, p1y, p2x, p2y = move_points(m_p1x_, m_p1y_, m_p2x_, m_p2y_)

    line_moved = ((p1x, p1y), (p2x, p2y))

    np2x, np2y = rotate(p2x, p2y, -m_p1t)

    line_rotated = ((p1x, p1y), (np2x, np2y))

    d_mv = math.sqrt(p2x*p2x + p2y*p2y)
    d_rt = math.sqrt(np2x*np2x + np2y*np2y)

    print(f"{round(d_mv,4)} <> {round(d_rt,4)}")

    # points_list = [line_original, line_moved, line_rotated]
    points_list = [line_moved, line_rotated]
    draw_lines(points_list)





process(4.60068, 1.38325, 1.33773, 4.60327, 1.40259)

