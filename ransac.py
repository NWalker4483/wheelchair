import cv2
import numpy as np

# Camera internals
size = (10, 10)
focal_length = size[1]
center = (size[1]/2, size[0]/2)
camera_matrix = np.array(
                         [[focal_length, 0, center[0]],
                         [0, focal_length, center[1]],
                         [0, 0, 1]], dtype = "double"
                         )

print(f"Camera Matrix :\n {camera_matrix}")

dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion

# FLANN parameters
FLANN_INDEX_KDTREE = 0
FLANN_INDEX_LSH = 6
index_params = dict(algorithm = FLANN_INDEX_LSH, trees = 5)
search_params = dict(checks=50)   # or pass empty dictionary

flann = cv2.FlannBasedMatcher(index_params, search_params)

# bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

orb = cv2.ORB_create(nfeatures=250)

MIN_MATCHES = 50

cam = cv2.VideoCapture(0)

ret, frame = cam.read()
last = frame
display = last

while True:
    ret, frame = cam.read()
    # # Need to draw only good matches, so create a mask
    matchesMask = []
    good_matches = []
    try:
        kp1, des1 = orb.detectAndCompute(last, None)
        #kp2, des2 = orb.detectAndCompute(frame, None)
        matches = []#flann.knnMatch(des1, des2, k=2)
        matchesMask = [[0,0] for i in range(len(matches))]
    
        # ratio test as per Lowe's paper
        # TODO: I do not understand this value error but it keeps coming up
        for i, pair in enumerate(matches):
            try:
                m, n = pair
                if m.distance < 0.7*n.distance:
                    matchesMask[i]=[1,0]
                    good_matches.append(m)
            except ValueError:
                #print("Match Failed", i)
                pass
    except Exception as e:
        print(e,3)

    if len(good_matches) >= MIN_MATCHES:
        old_points = np.float32([kp1[m.queryIdx].pt for m in good_matches])
        curr_points = np.float32([kp2[m.trainIdx].pt for m in good_matches])

        old_points = np.pad(old_points, [(0,0),(0,1)]) # Add Z axis of 0
        (success, rotation_vector, translation_vector, inliers) = cv2.solvePnPRansac(old_points, curr_points, camera_matrix, dist_coeffs, flags = cv2.SOLVEPNP_ITERATIVE)
        
        draw_params = dict(matchColor = (0,255,0),
                   singlePointColor = (255,0,0),
                   matchesMask = matchesMask,
                   flags = 0)
        
        display = cv2.drawMatchesKnn(last, kp1, frame, kp2, matches, None, **draw_params)
    last = frame
    cv2.imshow("3", frame)
    cv2.waitKey(1)
    