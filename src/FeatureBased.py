import cv2
import numpy as np

from .utils import PoseEstimater

class FeatureBased(PoseEstimater):
    def __init__(self, imgDir, pointsDir, resultDir):
        super().__init__(imgDir, pointsDir, resultDir)

    def _findRelativeRt(self, referencePoints, img):
        """Calculates the relative Rotation and translation for given reference points
        Adapted from https://docs.opencv.org/3.4/d1/de0/tutorial_py_feature_homography.html

        Args:
            referencePoints (array): Reference points where calculations are made wrt it.
            img (image): Calculated Rt belongs to this image

        Raises:
            ValueError: If any good match is not found, Rt can not be calculated. 

        Returns:
            Rt: returns Rt
        """
        kp1, des1 = referencePoints
        kp2, des2 = self.detector.detectAndCompute(img, None)
        
        # find the keypoints and descriptors with given detector with Brute Force
        matcher = cv2.BFMatcher(cv2.NORM_L2)

        matchedPoints = matcher.knnMatch(des1, des2, k=2)

        # Apply ratio test to filter results
        good = []
        for m, n in matchedPoints:
            if m.distance < 0.4 * n.distance:
                good.append(m)

        if not good:
            raise ValueError("No good matches for matching algorithm!")
        else:
            srcPoints = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dstPoints = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            
            # Find essential matrix for pose estimation
            E, maskE = cv2.findEssentialMat(dstPoints, srcPoints, self._calibMatrix, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            
            # Recover Rotation and translation from Essential matrix
            _, R, t, _ = cv2.recoverPose(E, dstPoints, srcPoints, cameraMatrix=self._calibMatrix, mask=maskE)
            return R, t, (kp2, des2)

    def calculateTrajectories(self, saveRes= False):
        """Calculates Rt and trajectories for the images given in the iamge dir.

        Returns:
            Rt and Trajectories: Poses of the cameras
        """
        self.detector = cv2.SIFT_create()
        referencePoints = self.detector.detectAndCompute(super()._readImage(imgNum=0), None)
        
        trajectories = [ np.zeros([3, 1])]
        Rt = []

        for img in self._imgList:
            if img == self._imgList[0]:
                continue
            relativeR, relative_t, referencePoints = self._findRelativeRt(referencePoints, self._readImage(img))
            
            trajectories.append(relativeR @ trajectories[-1] + relative_t)
            if not Rt:
                Rt.append((relativeR, relative_t))
            else:
                previousR, previous_t = Rt[-1]
                currentR = relativeR @ previousR
                current_t = relativeR @ previous_t + relative_t
                Rt.append((currentR, current_t))

        self._trajectories = np.array(trajectories).reshape(len(self._imgList), -1)
        self._Rt = Rt

        if saveRes:
            self.saveTrajectories()
        return np.array(trajectories).reshape(len(self._imgList), -1), Rt