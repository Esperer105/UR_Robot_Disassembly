from PIL import Image

from yolo import YOLO
import std_msgs
import rospy
if __name__ == "__main__":
    yolo = YOLO()
    mode = "predict"
    crop = False
    count = False

    if mode == "predict":
        '''
        1、如果想要进行检测完的图片的保存，利用r_image.save("img.jpg")即可保存，直接在predict.py里进行修改即可。 
        2、如果想要获得预测框的坐标，可以进入yolo.detect_image函数，在绘图部分读取top，left，bottom，right这四个值。
        3、如果想要利用预测框截取下目标，可以进入yolo.detect_image函数，在绘图部分利用获取到的top，left，bottom，right这四个值
        在原图上利用矩阵的方式进行截取。
        4、如果想要在预测图上写额外的字，比如检测到的特定目标的数量，可以进入yolo.detect_image函数，在绘图部分对predicted_class进行判断，
        比如判断if predicted_class == 'car': 即可判断当前目标是否为车，然后记录数量即可。利用draw.text即可写字。
        '''
        while True:
            img = input('Input image filename:')
            try:
                image = Image.open(img)
            except:
                print('Open Error! Try again!')
                # continue
            else:
                # here
                new_image=Image.new('RGB',(1920,1080),(128,128,128))
                new_image.paste(image,((1920-640)//2,(1080-480)//2))
                # changed

                r_image, result = yolo.detect_image(
                    image, crop=crop, count=count)
                print(result)
                r_image.show()
