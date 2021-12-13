import torch
from torch import nn
from torchvision import models


class VGG(nn.Module):
    def __init__(self, num_classes=1000):
        super(VGG, self).__init__()
        self.net = models.vgg16(pretrained=True)
        self.net.classifier[6] =nn.Linear(4096, num_classes)

        # # 卷积层提取图像特征
        # self.features = self.net.features
        # self.avgpool = self.net.avgpool
        #
        # # 全连接层对图像分类
        # self.classifier = nn.Sequential(
        #     self.net.classifier[0],
        #     self.net.classifier[1],
        #     self.net.classifier[2],
        #     self.net.classifier[3],
        #     self.net.classifier[4],
        #     self.net.classifier[5],
        #     nn.Linear(4096, num_classes),
        #     nn.Softmax(dim=0)
        # )

    def forward(self, x):
        # x = self.features(x)
        # x = self.avgpool(x)
        # x = self.net.avgpool(x)
        x = self.net(x)
        return x
