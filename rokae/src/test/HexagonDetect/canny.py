# from scipy.misc import imread, imsave
from imageio import imread, imsave
import torch
import cv2
from torch.autograd import Variable
from net_canny import Net


def canny(raw_img, use_cuda=False):
    img = torch.from_numpy(raw_img.transpose((2, 0, 1)))
    batch = torch.stack([img]).float()

    net = Net(threshold=4.0, use_cuda=use_cuda)
    if use_cuda:
        net.cuda()
    net.eval()

    data = Variable(batch)
    if use_cuda:
        data = Variable(batch).cuda()

    blurred_img, grad_mag, grad_orientation, thin_edges, thresholded, early_threshold = net(data)
    cv2.imshow("test", early_threshold.data.cpu().numpy()[0, 0])
    cv2.waitKey(1)



if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    while True:         
        ret, img = cap.read()
        img = img / 255.0
        # canny(img, use_cuda=False)
        canny(img, use_cuda=True)
        
