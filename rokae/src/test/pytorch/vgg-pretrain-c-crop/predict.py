import torch
from module import VGG
from PIL import Image
from torchvision import transforms
import matplotlib.pyplot as plt
import json
import os
import random



# 预处理
data_transform = transforms.Compose(
    [transforms.Resize((224, 224)),
     transforms.ToTensor(),
     transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])

fileDir='/home/nuc/Desktop/test/tensorflow_vgg_classify/rokae_image/vertical_capture_false_obstacle/train/align'
pathDir = os.listdir(fileDir)    #取图片的原始路径
filenumber=len(pathDir)
rate=0.1    #自定义抽取图片的比例，比方说100张抽10张，那就是0.1
picknumber=int(filenumber*rate) #按照rate比例从文件夹中取一定数量图片
sample = random.sample(pathDir, picknumber)  #随机选取picknumber数量的样本图片


# load image
# images = ["tulips", "dandelion", "roses", "sunflower", "daisy"]
for i in sample:
    img = Image.open(fileDir+  '/'  + i)
    plt.imshow(img)
    # plt.show()
    # [N, C, H, W]
    img = data_transform(img)
    # expand batch dimension
    img = torch.unsqueeze(img, dim=0)

    # read class_indict
    try:
        json_file = open('class_indices.json', 'r')
        class_indict = json.load(json_file)
    except Exception as e:
        print(e)
        exit(-1)

    # create model
    model = VGG(num_classes=2)

    # load model weights
    model_weight_path = 'VGG.pth'
    model.load_state_dict(torch.load(model_weight_path))

    # 关闭 Dropout
    model.eval()
    with torch.no_grad():
        # predict class
        output = torch.squeeze(model(img))     # 将输出压缩，即压缩掉 batch 这个维度
        predict = torch.softmax(output, dim=0)
        predict_cla = torch.argmax(predict).numpy()
    print("origin: "+i+"\tpredict: "+class_indict[str(predict_cla)], "\tProbability: ",predict[predict_cla].item())
    #plt.show()