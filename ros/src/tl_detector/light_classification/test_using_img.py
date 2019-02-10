#!/usr/bin/env python

from tl_classifier import TLClassifier
import cv2 as cv

if __name__ == '__main__':
    classifier = TLClassifier()
    img = cv.imread('image.png')

    # hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    # cv.imshow('test image', img)
    # h, s, v = cv.split(hsv)

    # h[h > 240] = 255
    # h[h < 15] = 255
    # h[h != 255] = 0
    # # cv.imshow('test h', h)
    # s[s > 150] = 255
    # s[s != 255] = 0
    # # cv.imshow('test s', s)
    # v[v > 200] = 255
    # v[v != 255] = 0
    # # cv.imshow('test v', v)

    # thres = cv.bitwise_and(cv.bitwise_and(h, s), v)
    # cv.imshow('test thres', thres)

    # nonzero = cv.countNonZero(thres)
    # print(nonzero)

    print(classifier.get_classification(img, 0.5))

    cv.waitKey()
    cv.destroyAllWindows()
