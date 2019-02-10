from styx_msgs.msg import TrafficLight
import cv2 as cv

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image, probability):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # light color prediction
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        h, s, v = cv.split(hsv)

        h[h > 240] = 255
        h[h < 15] = 255
        h[h != 255] = 0
        s[s > 150] = 255
        s[s != 255] = 0
        v[v > 200] = 255
        v[v != 255] = 0

        thres = cv.bitwise_and(cv.bitwise_and(h, s), v)

        nonzero = cv.countNonZero(thres)

        return TrafficLight.RED if nonzero * probability > 70 else TrafficLight.UNKNOWN
