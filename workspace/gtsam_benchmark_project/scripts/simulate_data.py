import argparse
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
import os
import yaml

def generate_landmarks(space_range):
    return [np.random.uniform(-space_range/2, space_range/2, 2) for _ in range(6)]

def generate_landmarks_init_estimates(landmarks, landmark_uniform_err):
    landmarks_init_estimates = []

    for l in landmarks:
        x_init = l[0] + np.random.uniform(0, landmark_uniform_err)
        y_init = l[1] + np.random.uniform(0, landmark_uniform_err)
        landmarks_init_estimates.append(np.array([x_init, y_init]))

    return landmarks_init_estimates

def generate_poses(space_range, n_steps, delta_s):
    t_steps_per_turn = 10000
    delta_t = 0.5
    turns = 1
    
    path_length = 0
    
    for _ in range(2):
        n_t_steps = t_steps_per_turn*turns

        t = np.arange(0, n_t_steps, delta_t)
        x = (space_range/2)*np.sin(2*(2*np.pi/(n_t_steps/turns))*t) + (space_range/4)*np.sin(7*(2*np.pi/(n_t_steps/turns))*t)
        y = (space_range/2)*np.cos(3*(2*np.pi/(n_t_steps/turns))*t + np.pi/2)

        distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        arc_length = np.insert(np.cumsum(distances), 0, 0)

        path_length = arc_length[-1]
        turns = (n_steps*delta_s // path_length) + 1


    interp_x = interp1d(arc_length, x, kind='cubic')
    interp_y = interp1d(arc_length, y, kind='cubic')

    s = np.arange(0, n_steps*delta_s, delta_s)

    x = interp_x(s)
    y = interp_y(s)
    theta = np.arctan2(np.diff(y, prepend=y[0]), np.diff(x, prepend=x[0]))
    theta[0] = theta[1]

    poses = [np.array([i, j, k]) for i, j, k in zip(x, y, theta)]

    return poses

def generate_poses_init_estimates(poses, pose_uniform_err, pose_theta_uniform_err):
    poses_init_estimates = []

    for p in poses:
        x_init = p[0] + np.random.uniform(0, pose_uniform_err)
        y_init = p[1] + np.random.uniform(0, pose_uniform_err)
        theta_init = p[2] + np.random.uniform(0, pose_theta_uniform_err)
        poses_init_estimates.append(np.array([x_init, y_init, theta_init]))

    return poses_init_estimates


def store_data(dir_path, args, landmarks_init_estimates, poses_init_estimates):
    with open(f"{dir_path}/params.yaml", "w") as f:
        yaml.dump(vars(args), f)

    with open(f"{dir_path}/landmarks_initial_estimates.csv", "w") as f:
        f.write("id,x,y\n")
        for i, l in enumerate(landmarks_init_estimates):
            f.write(f"{i},{l[0]},{l[1]}\n")
    
    with open(f"{dir_path}/poses_initial_estimates.csv", "w") as f:
        f.write("id,x,y,theta\n")
        for i, p in enumerate(poses_init_estimates):
            f.write(f"{i},{p[0]},{p[1]},{p[2]}\n")
    


def plot(poses, landmarks, poses_init_estimates, landmarks_init_estimates):
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    
    ax = axes[0]
    x = [p[0] for p in poses]
    y = [p[1] for p in poses]
    theta = [p[2] for p in poses]

    ax.plot(x, y, label="Poses")
    dx = 0.33 * np.cos(theta)  
    dy = 0.33 * np.sin(theta)
    ax.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1., color='red', width=0.01)
    ax.scatter([l[0] for l in landmarks], [l[1] for l in landmarks], c='green', label="Landmarks")
    ax.set_title("Ground Truth")
    # ax.legend()
    ax.grid()

    ax = axes[1]
    x_init = [p[0] for p in poses_init_estimates]
    y_init = [p[1] for p in poses_init_estimates]
    theta_init = [p[2] for p in poses_init_estimates]

    ax.plot(x_init, y_init, label="Initial Pose Estimates")
    dx_init = 0.33 * np.cos(theta_init)  
    dy_init = 0.33 * np.sin(theta_init)
    ax.quiver(x_init, y_init, dx_init, dy_init, angles='xy', scale_units='xy', scale=1., color='red', width=0.01)
    ax.scatter([l[0] for l in landmarks_init_estimates], [l[1] for l in landmarks_init_estimates], c='green', label="Initial Landmark Estimates")
    ax.set_title("Initial Estimates")
    # ax.legend()
    ax.grid()

    plt.tight_layout()
    plt.show()




def main(args):

    dir_path = f"./data/exp_{args.exp}"
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)
    else:
        raise Exception(f"Directory {dir_path} already exists")

    # landmarks = generate_landmarks(space_range)
    # print(landmarks)
    # plt.scatter([l[0] for l in landmarks], [l[1] for l in landmarks], c='red')
    # plt.show()

    landmarks = [
        np.array([ 49.10878951, -19.96772516]),
        np.array([-14.71252512,   2.86688899]),
        np.array([ 45.46266376, -27.60415914]),
        np.array([  2.65279246, -13.18585996]),
        np.array([-1.66187825, 33.13027665]),
        np.array([-41.15576638, -49.28131293])
    ]

    landmarks_init_estimates = generate_landmarks_init_estimates(landmarks, args.landmark_uniform_err)

    poses = generate_poses(args.space_range, args.n_steps, args.delta_s)

    poses_init_estimates = generate_poses_init_estimates(poses, args.pose_uniform_err, args.pose_theta_uniform_err)


    # measures (bearing, range for landmarks)

    # measures (dx, dy, dtheta for landmarks)

    store_data(dir_path, args, landmarks_init_estimates, poses_init_estimates)
    
    if args.plot:
        plot(poses, landmarks, poses_init_estimates, landmarks_init_estimates)
        # plot(poses_init_estimates, landmarks_init_estimates)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Simulate data to perform a GTSAM benchmark.')
    parser.add_argument('--exp', type=int, required=True, help='Experiment number')
    parser.add_argument('--space_range', type=float, default=100, help='Range of the 2D space')
    parser.add_argument('--n_steps', type=int, default=10000, help='Number of steps')
    parser.add_argument('--delta_s', type=float, default=0.5, help='Step size (m)')
    parser.add_argument('--landmark_uniform_err', type=float, default=0.0, help='Max error from uniform distribution on landmark position (m)')
    parser.add_argument('--pose_uniform_err', type=float, default=0.0, help='Max error from uniform distribution on poses position (m)')
    parser.add_argument('--pose_theta_uniform_err', type=float, default=0.0, help='Max error from uniform distribution on poses yaw (rad)')
    parser.add_argument('--plot', action="store_true", help='Plot flag')

    args = parser.parse_args()
    main(args)