import sys
import cv2
import numpy as np


ratio = 0.55

""" Clear matches for which NN ratio is > than threshold """
def filter_distance(matches):
    dist = [m.distance for m in matches]
    thres_dist = (sum(dist) / len(dist)) * ratio

    # keep only the reasonable matches
    sel_matches = [m for m in matches if m.distance < thres_dist]
    print(f'#selected matches: {len(sel_matches)} (out of {len(matches)})')
    return sel_matches

""" keep only symmetric matches """
def filter_asymmetric(matches1, matches2, pts1, pts2):
    sel_matches = []
    for match1 in matches1:
        for match2 in matches2:
            if pts1[match1.queryIdx] == pts1[match2.trainIdx] and pts2[match1.trainIdx] == pts2[match2.queryIdx]:
                sel_matches.append(match1)
                break
    return sel_matches

# Todo: filter_ransac

def filter_matches(matches1, matches2, pts1, pts2):
    matches = filter_distance(matches1)
    matches2 = filter_distance(matches2)
    return filter_asymmetric(matches, matches2, pts1, pts2)


def match_flann(desc1, desc2, r_threshold = 0.6):
    flann = cv2.flann_Index(desc2, flann_params)
    idx2, dist = flann.knnSearch(desc1, 2, params = {}) # bug: need to provide empty dict
    mask = dist[:,0] / dist[:,1] < r_threshold
    idx1 = np.arange(len(desc1))
    pairs = np.int32( zip(idx1, idx2[:,0]) )
    return pairs[mask]


if __name__ == '__main__':
    path_to_video = sys.argv[1]

    video = cv2.VideoCapture(path_to_video)
    assert video.isOpened()

    # Ground Truth
    orb = cv2.ORB_create()
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    template = cv2.imread('data/template.png', cv2.IMREAD_COLOR)
    gt_blobs, gt_descriptors = orb.detectAndCompute(template, None)

    frame_ix = 0
    success, frame = video.read()
    while success:
        # cv2.absdiff(frame1, frame0, diff_img)
        # _, diff_img = cv2.threshold(diff_img[:, :, 2], 127, 255, cv2.THRESH_BINARY)

        blobs, descriptors = orb.detectAndCompute(frame, None)

        matches1 = bf.match(descriptors, gt_descriptors)
        matches2 = bf.match(gt_descriptors, descriptors)

        selected = filter_matches(matches1, matches2, gt_blobs, blobs)

        matched_keypoints = [blobs[m.trainIdx] for m in selected]
        # for m in matched_keypoints:
        #     frame = cv2.circle(frame, m.pt, 10, cv2.Scalar(0,0,255),10)
        demo = cv2.drawKeypoints(frame, matched_keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        for k_pt in matched_keypoints:
            demo = cv2.circle(demo, (int(k_pt.pt[0]), int(k_pt.pt[1])), int(k_pt.size), (0, 0, 255), 1, 8, 0)
        cv2.imwrite(f'./data/output/{frame_ix}.jpg', demo)

        success, frame = video.read()
        frame_ix = frame_ix + 1
