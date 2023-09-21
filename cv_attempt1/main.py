import cv2
import numpy as np
import os
from DataLoader import DataLoader
from VO_rehash import VO
import plotly.graph_objects as go
import matplotlib.pyplot as plt

image_path = "/home/aemandev/repos/datasets/dataset-corridor4_1024_16/dataset-corridor4_1024_16/mav0/cam0/data"
csv_path = "/home/aemandev/repos/datasets/dataset-corridor4_1024_16/dataset-corridor4_1024_16/mav0/cam0/data.csv"
gt_path = "/home/aemandev/repos/datasets/dataset-corridor4_1024_16/dataset-corridor4_1024_16/mav0/mocap0/data.csv"


def run_cv():
    dataLoader = DataLoader(image_path)
    intrinsics = [
        380.81042871360756,
        380.81194179427075,
        510.29465304840727,
        514.3304630538506,
    ]
    K = np.array(
        [
            [intrinsics[0], 0, intrinsics[2]],
            [0, intrinsics[1], intrinsics[3]],
            [0, 0, 1],
        ]
    )
    vo = VO(K)
    pos_arr = np.array([0, 0, 0])
    traj = np.zeros(shape=(600, 800, 3))
    gt_traj = np.zeros(shape=(600, 800, 3))
    with open(csv_path, "r") as f, open(gt_path, "r") as gt:
        next(gt)
        next(f)
        for line in f:
            gt_data = next(gt)
            gt_arr = np.asarray(gt_data.split(",")[1:4], dtype=np.float32)
            dataLoader.load_next_image(line)
            img = cv2.imread(dataLoader.current_image)
            # Downsample image
            img = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
            # make gray scale
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            vo.process_fame(img)
            # cv2.imshow('image', img)
            if vo.keyImg is not None:
                # Print values on image
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(
                    vo.keyImg,
                    f"r:{round(vo.angles[0],1)} p:{round(vo.angles[1],1)} y: {round(vo.angles[2],1)}",
                    (10, 50),
                    font,
                    0.75,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    vo.keyImg,
                    f"x:{round(vo.t[0][0],1)} y:{round(vo.t[1][0],1)} z: {round(vo.t[2][0],1)}",
                    (10, 100),
                    font,
                    1,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("image", vo.keyImg)
                # cv2.imshow("image", vo.matches)
                # Append to position array
                pos_arr = np.vstack((pos_arr, vo.t.T))
                # Plot the trajectory matplotlib
                gt_x, gt_y, gt_z = [int(round(x * 100)) for x in gt_arr]
                draw_x, draw_y, draw_z = [int(round(x[0])) for x in vo.t]
                traj = cv2.circle(
                    traj, (draw_x + 400, draw_z + 100), 1, list((0, 255, 0)), 4
                )
                traj = cv2.circle(
                    traj, (gt_x + 400, gt_z + 100), 1, list((0, 0, 255)), 4
                )
                cv2.putText(
                    traj,
                    "Estimated Odometry Position:",
                    (30, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                )
                cv2.putText(
                    traj,
                    "Green",
                    (270, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

                cv2.imshow("trajectory", traj)
                print(len(vo.curr_pts))
            k = cv2.waitKey(1)
            if k == 27:
                break
    cv2.destroyAllWindows()

    # Plot the trajectory
    fig = go.Figure()
    fig.add_trace(
        go.Scatter3d(x=pos_arr[:, 0], y=pos_arr[:, 1], z=pos_arr[:, 2], mode="markers")
    )
    fig.show()


if __name__ == "__main__":
    run_cv()
