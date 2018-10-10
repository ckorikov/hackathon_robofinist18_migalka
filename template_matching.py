import cv2
import imutils
import numpy as np
import sys


def main(path_to_mov):
    path_to_video = sys.argv[1]
    # template = cv2.Canny(cv2.imread('data/tmpl1.jpg', cv2.IMREAD_GRAYSCALE), 50, 200)
    template = cv2.imread('data/template.png', cv2.IMREAD_GRAYSCALE)
    template = imutils.resize(template, width = 50)
    tmpl_h, tmpl_w = template.shape[:2]


    video = cv2.VideoCapture(path_to_video)
    assert video.isOpened()

    success, frame = video.read()
    while success:
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        print(f'frame = {frame.shape}')
        found = None

        # for scale in np.linspace(0.2, 1.0, 20)[::-1]:
        for scale in [0.2, 0.5, 1.0]:
            resized_frame = imutils.resize(gray_frame, width = int(gray_frame.shape[1] * scale))
            # edged_frame = cv2.Canny(resized_frame, 50, 200)
            edged_frame = resized_frame
            print(f'resized_frame = {resized_frame.shape}')
            print(f'edged_frame = {edged_frame.shape}')
            r = edged_frame.shape[1] / float(edged_frame.shape[1])

            res = cv2.matchTemplate(edged_frame, template, cv2.TM_CCOEFF_NORMED) #cv2.TM_SQDIFF
            cv2.imshow('match', res)
            cv2.waitKey(0)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

            top_left = min_loc
            bottom_right = (top_left[0] + tmpl_h, top_left[1] + tmpl_w)

            if found is None or max_val > found[0]:
                found = (max_val, max_loc, r)

        _, max_loc, r = found
        start_y, start_x = int(max_loc[0] / r), int(max_loc[1] / r)
        end_y, end_x = int((max_loc[0] + tmpl_h) / r), int((max_loc[1] + tmpl_w) / r)
 
        # draw a bounding box around the detected result and display the image
        cv2.rectangle(frame, (start_y, start_x), (end_y, end_x), (0, 0, 255), 2)

        cv2.imshow('TL', frame)
        cv2.waitKey(0)

        success, frame = video.read()


if __name__ == '__main__':
    main(sys.argv[1])
