#!/usr/bin/env python3
import cv2
import numpy as np

class FeatureExtractor:
    """Example class for feature extraction - students should extend this"""

    def __init__(self, method='ORB'):
        self.method = method

        if method == 'ORB':
            self.detector = cv2.ORB_create()
        elif method == 'SIFT':
            self.detector = cv2.SIFT_create()
        elif method == 'SURF':
            self.detector = cv2.xfeatures2d.SURF_create()
        else:
            raise ValueError(f"Unknown method: {method}")

        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING if method == 'ORB' else cv2.NORM_L2, crossCheck=True)

    def extract_features(self, image):
        """Extract features from image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.detector.detectAndCompute(gray, None)
        return keypoints, descriptors

    def match_features(self, desc1, desc2):
        """Match features between two sets of descriptors"""
        matches = self.matcher.match(desc1, desc2)
        matches = sorted(matches, key=lambda x: x.distance)
        return matches

class IBVSController:
    """Template for IBVS controller - students should implement this"""

    def __init__(self, camera_matrix):
        self.camera_matrix = camera_matrix
        self.lambda_gain = 0.1  # Control gain

    def compute_interaction_matrix(self, features, depth):
        """
        Compute the interaction matrix (image Jacobian)
        Students need to implement this based on the feature type
        """
        # TODO: Implement interaction matrix computation
        pass

    def compute_velocity(self, current_features, desired_features, depth):
        """
        Compute camera velocity using IBVS control law
        v = -lambda * L^+ * e
        where L is interaction matrix, e is feature error
        """
        # TODO: Implement IBVS control law
        pass