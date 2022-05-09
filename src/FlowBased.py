from .utils import PoseEstimater

class FlowBased(PoseEstimater):
    def __init__(self, imgDir, pointsDir, resultDir):
        super().__init__(imgDir, pointsDir, resultDir)
        print("Is not complete!\n")