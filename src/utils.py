import os
import cv2
import numpy as np
from matplotlib import pyplot as plt

class PoseEstimater:
    """ This is the base class that both flow and feature based class derived.
    """
    def __init__(self, imgDir, pointsDir, resultDir):
        """Constructor

        Args:
            imgDir (str): directory of images
            pointsDir (str): directory of GT points
            resultDir (str): directory of the results
        """
        self._dirImgs = imgDir
        self._dirPoints = pointsDir
        self._dirResults = resultDir
        self._calibMatrix = None
        self._trajectories = None
        self._Rt = None
        self._imgList = os.listdir(imgDir)

    def _readImage(self,imgName= None, imgNum= 0):
        """ Read the image from the image list on demand

        Args:
            imgName (str, optional): the image that has been given to be read. Defaults to None.
            imgNum (int, optional): The corresponding image in the img List will be read. Defaults to 0.

        Returns:
            image: returns the image on demand
        """
        if not imgName:
            return cv2.imread(os.path.join(self._dirImgs,self._imgList[imgNum]))
        else:
            return cv2.imread(os.path.join(self._dirImgs,imgName))

    def _loadNumpyPoints(self):
        """loads the GT poins into the instance
        """
        vr2d = np.load(os.path.join(self._dirPoints,"vr2d.npy"))
        self._imagePoints = vr2d.reshape(1, len(vr2d), -1)

        vr3d = np.load(os.path.join(self._dirPoints,"vr3d.npy"))
        self._objectPoints = vr3d.reshape(1, len(vr3d), -1)


    def saveResult(self, variable, filename, varname = '\n', flag = "a"):
        """Save given variable into the given filename with given varname

        Args:
            variable (any): variable to be saved
            filename (dir): file to be saved in
            varname (str): explanation on file for variable. Defaults to '\n'.
            flag (str, optional): whether overwrite("w") or append("a") . Defaults to "a".
        """
        with open(os.path.join(self._dirResults,filename),flag) as f:
            f.write( varname +"\n")
            f.write(str(variable))
            f.write("\n")
            f.close()

    def saveFigure(self, variableList, index1=0, index2=1, title= '', xlabel= 'x', ylabel= 'y', filename = None):
        """_summary_

        Args:
            variableList (_type_): variable to be 
            index1 (int, optional): Defaults to 0.
            index2 (int, optional): Defaults to 1.
            title (str, optional): Defaults to ''.
            xlabel (str, optional): Defaults to 'x'.
            ylabel (str, optional): Defaults to 'y'.
            filename (dir, optional): Defaults to None.
        """
        plt.figure()
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)

        plt.plot(variableList[:, index1], variableList[:, index2], label=title)
        for i, traj in enumerate(variableList):
            plt.scatter(traj[index1], traj[index2], label="Image-{}".format(i + 1))

        if not filename:
            filename = title + '.png'

        plt.legend()
        plt.savefig(os.path.join(self._dirResults,filename))


    def calibrateCamera(self, initialCalibMatrix, savefile = True, distortion=np.zeros((14,), dtype=np.float32)):
        """Fine tunes the given camera calib matrix

        Args:
            initialCalibMatrix (array): starting point
            savefile (bool, optional): . Defaults to True.
            distortion (array, optional): Initial distortion coeff. Defaults to np.zeros((14,), dtype=np.float32).

        Returns:
            array: camera intirinsic parameters
        """
        self._loadNumpyPoints()
        tmpImg = self._readImage()

        _, self._calibMatrix, _, _, _ = cv2.calibrateCamera(self._objectPoints, self._imagePoints,
        tmpImg.shape[:2][::-1], initialCalibMatrix, distortion,
        flags=(cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_FIX_PRINCIPAL_POINT +
        cv2.CALIB_FIX_ASPECT_RATIO + cv2.CALIB_FIX_S1_S2_S3_S4 + cv2.CALIB_ZERO_TANGENT_DIST +
        cv2.CALIB_FIX_K1 + cv2.CALIB_FIX_K2 + cv2.CALIB_FIX_K3 +
        cv2.CALIB_FIX_K4 + cv2.CALIB_FIX_K5 + cv2.CALIB_FIX_K6 ))

        if savefile:
            self.saveResult(self._calibMatrix,"CameraCalibRationMatrix.txt","Estimated Intrinsic Matrix","w")
        
        return self._calibMatrix

    def saveTrajectories(self,Rt= None, trajectories= None):
        """_summary_

        Args:
            Rt (array, optional): Rotation and translation matrix. Defaults to None.
            trajectories (array, optional): Trajectory where camera goes. Defaults to None.
        """
        if not Rt:
            Rt = self._Rt
            trajectories =self._trajectories

        for i, (R, t) in enumerate(Rt):
            self.saveResult(R,"Rt{0}.txt".format(i + 2),"Rotation", "w")
            self.saveResult(t,"Rt{0}.txt".format(i + 2),"Translation", "a")

        self.saveFigure(trajectories, 0, 1, 'Camera Trajectory- XYplane', "x", "y","xyplane.png")
        self.saveFigure(trajectories, 0, 2, 'Camera Trajectory- XZplane', "x", "z","xzplane.png")
        self.saveFigure(trajectories, 1, 2, 'Camera Trajectory- YZplane', "y", "z","yzplane.png")