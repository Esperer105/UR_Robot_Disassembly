import argparse
import torch
import torchvision
import cv2
from torch.autograd import Variable
from net_canny import Net
import numpy as np
import math

# create an array of points in the shape of a hexagon
def make_hex_shape():
    pts = []
    for ang in range(0, 355, 60):
        ang_r = math.radians(ang)
        x1 = int(100.0 * math.cos(ang_r) + 100.5)
        y1 = int(100.0 * math.sin(ang_r) + 100.5)
        pts.append([x1, y1])
    shape_np = np.array(pts, np.int32)
    shape_np = np.reshape(shape_np, (-1, 1, 2))
    return shape_np

def canny(raw_img, shape, use_cuda=False):
    img = torch.from_numpy(raw_img.transpose((2, 0, 1)))
    batch = torch.stack([img]).float()

    net = Net(threshold=4.0, use_cuda=use_cuda)
    if use_cuda:
        net.cuda()
    net.eval()

    data = Variable(batch)
    if use_cuda:
        data = Variable(batch).cuda()

    blurred_img, grad_mag, grad_orientation, early_threshold = net(data)
    img = early_threshold.data.cpu().numpy()[0, 0]
    return img
def procImage(img, shape):
    img = canny(img, hex, use_cuda=False)
    img = np.uint8(img * 255)
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    centerpoints = []
    hexes = []
    for cont in contours:
        rect = cv2.boundingRect(cont)
        # only process larger areas with at least 5 points in the contour
        if len(cont) > 80 and rect[2] > 40 and rect[3] > 40:
            match = cv2.matchShapes(cont, hex, cv2.CONTOURS_MATCH_I2, 0.0)
            if match < 0.08:
                print(f"len(cont)={len(cont):3d}:  match={match:.4f}")

            if match < 0.02:
                cx = rect[0] + (rect[2] * .5)
                cy = rect[1] + (rect[3] *.5)
                centerpoints = (cx, cy)
                hexes.append(cont)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img = cv2.drawContours(img, hexes, -1, (0, 255, 0), 4)
    ctr = np.array(centerpoints).reshape((-1, 1, 2)).astype(np.int32)
    img = cv2.drawContours(img, ctr, -1, (0, 255, 0), 20)
    return img
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='A tutorial of argparse!')
    parser.add_argument("--source", default=0,
                        help="Input Source")
    parser.add_argument("--save", action="store_true", help="save video to file")
    args = parser.parse_args()
    src = 0
    out = 0
    hex = make_hex_shape()
    print(args)
    if args.source != None:

        src = args.source

    cap = cv2.VideoCapture(src)
    if args.save:
        frame_width = int(cap.get(3))
        frame_height = int(cap.get(4))
        out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (frame_width, frame_height))

    while True:
        retr, img = cap.read()
        img = img / 255.0
        img = img[0:360, 0:1280]
        img = procImage(img, hex)
        if args.save:
            out.write(img)
        cv2.imshow("contours", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    out.release()
    cv2.destroyAllWindows()

