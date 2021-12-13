
#!/usr/bin/env python3
#-*- coding: UTF-8 -*-

from PIL import Image,ImageDraw
import numpy as np 
# template_path='src/battery_pack_describe/bolt.jpg'

# small_image = Image.open("./img/small.jpg")
# big_image = Image.open("./img/big.jpg")

def main(img_path):

    template_path='src/test/TemplateMatching/img/bolt.jpg'
    
    small_image = Image.open(template_path)
    big_image =Image.open(img_path)

    big_image_rgb = big_image
    small_image = small_image.convert('L')

    big_image = big_image.convert('L')
    
    big = np.array(big_image)
    small = np.array(small_image)
    

    rand_point = []
    for i in range(50):
        rand_point.append((np.random.randint(0,small.shape[0]-1) ,np.random.randint(0,small.shape[1]-1)))
    
    R = np.zeros([big.shape[0]-small.shape[0],big.shape[1]-small.shape[1]])
    
    sMean = np.mean(small)
    for i in range(R.shape[0]):
        for j in range(R.shape[1]):
            loss = 0 
            mean = np.mean(big[i:i+small.shape[0],j:j+small.shape[1]]) 
            for n in range(len(rand_point)):
                point = rand_point[n]
    
                loss += np.abs( big[i+point[0],j+point[1]] - mean - small[point[0],point[1]] + sMean)
                if loss >= 150 or n==len(rand_point)-1: 
                    R[i,j] = n
                    break 
    index = np.unravel_index(R.argmax(), R.shape)
     
    xy = [(index[1],index[0]),(index[1]+small.shape[1],index[0]+small.shape[0])]
    print('index[1]',index[1])
    print('index[0]',index[0])
    print('c[1]',index[1]+small.shape[1]/2)
    print('c[0]',index[0]+small.shape[0]/2)


    big_image = big_image_rgb
    draw = ImageDraw.Draw(big_image)
    # draw.rectangle(xy,outline='red',width=1)
    draw.rectangle(xy, fill ="#ffff33",width=3)
    big_image.show()
    # big_image.save("output.jpg")
    return index[1]+small.shape[1]/2,index[0]+small.shape[0]/2


if __name__ == '__main__':
    x,y=main(filename)



