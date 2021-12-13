
import os, random, shutil

def mkfile(file):
    if not os.path.exists(file):
        os.makedirs(file)


def moveFile(fileDir):
    pathDir = [cla for cla in os.listdir(fileDir) if "rgb_img"  in cla]   #取图片的原始路径

    # pathDir = os.listdir(fileDir)    #取图片的原始路径
    filenumber=len(pathDir)
    rate=0.8    #自定义抽取图片的比例，比方说100张抽10张，那就是0.1
    picknumber=int(filenumber*rate) #按照rate比例从文件夹中取一定数量图片
    sample = random.sample(pathDir, picknumber)  #随机选取picknumber数量的样本图片
    print (sample)

    for index, image in enumerate(pathDir):
        if image  in sample:
            shutil.move(fileDir+ '/'+image, trainDir+ '/'+image)
        else :
            shutil.move(fileDir+ '/'+image, testDir+ '/'+image)
    return

if __name__ == '__main__':
    fileDir = 'rokae_image/vertical_capture/false_align/rgbd/'   #源图片文件夹路径
    trainDir = 'rokae_image/train/false_align'    #移动到新的文件夹路径
    testDir = 'rokae_image/test/false_align'    #移动到新的文件夹路径
    # trainDir = 'rokae_image/train/align'    #移动到新的文件夹路径
    # testDir = 'rokae_image/test/align'    #移动到新的文件夹路径
    mkfile(trainDir)    #创建文件夹
    mkfile(testDir)     #创建文件夹
    moveFile(fileDir)   #移动图像
