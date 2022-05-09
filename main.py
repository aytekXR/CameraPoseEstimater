import numpy as np
import argparse

from src.FeatureBased import *
from src.FlowBased import *


parser = argparse.ArgumentParser(description='Estimate Camera Trajectory for given images and object ground truth')
parser.add_argument('--image-dir', type=str, default='./data/images/',
                    help='path to the images')
parser.add_argument('--points-dir', type=str, default='./data/points/',
                    help='path to the ground truth image and object points')
parser.add_argument('--results-dir', type=str, default='./results/',
                    help='path to the directory where results are saved.')

args = parser.parse_args()

poseEstimater = FeatureBased(args.image_dir, args.points_dir, args.results_dir)

# Initial Calibration Matrix
intrinsicMatrix = np.eye(3)
intrinsicMatrix[0,0] = 100 #focal length initial value in x direction
intrinsicMatrix[1,1] = 100 #focal length initial value in y direction
intrinsicMatrix[0,2] = 960 #principal point x component
intrinsicMatrix[1,2] = 540 #principal point v component
distortions = np.zeros((14,), dtype=np.float32) #no distortion

# Calibrate the camera
calibMatrix = poseEstimater.calibrateCamera(intrinsicMatrix, distortion=distortions )

# Estimate the camera trajectory and Rotation translation matrix
trajectories, Rt = poseEstimater.calculateTrajectories(saveRes= False)
poseEstimater.saveTrajectories(Rt, trajectories)