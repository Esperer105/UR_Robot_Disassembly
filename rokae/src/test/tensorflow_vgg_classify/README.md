# tensorflow_vgg19_classify
> This repository aims to implement a VGG19 with tensorflow . it gives a pretrain weight (vgg19.npy), you can download from 
[here](https://mega.nz/#!xZ8glS6J!MAnE91ND_WyfZ_8mvkuSa2YcA7q-1ehfSm-Q1fxOvvs).The training file contains 668 images(223 fire images and 445 non-fire images).
The testing files only have 50 fire images and 50 non-fire images.Such few images impacted the precision and recall,which have lower results.We sincerely suggest you
to download more images to train.
> We built this VGG19 with Tensorflow and Keras in Windows ,  it's very convenient for most of you to train the net.

## Requirements
* Python 3.5
* TensorFlow 1.0
* Numpy
* fire and non-fire images [here](https://github.com/UIA-CAIR/Fire-Detection-Image-Dataset)

## Modifications
* image_generator: We abandoned the image iterator used in previous Alexnet, and created a new gennerator with tensorflow queue.
* In this VGG19 Net, we used precision, recall and f1 to measure the performance of the net, those parameters are always used in Statistics.

## Notes:
* The vgg19 net and datagenerator files have been builded, you don't have to modify it. But if you have more concise or effective codes, please do share them with us.
* finetune.py is aimed to tune the weights and bias in the full connected layer, you must define some varibles,functions,and class numbers according to your own classification projects.  

## Results:
* precision = 60%
* recall = 95%
* f1 = 72%

## Example output:
We choosed some pictures from the internet to validate the VGG19 Net, the sunset images or warm lights images are difficult to identify, and some non-fire images misidentified.
See the results below:

![2017-11-23-09-50-55](http://qiniu.xdpie.com/2017-11-23-09-50-55.png)

![2017-11-23-09-51-45](http://qiniu.xdpie.com/2017-11-23-09-51-45.png)

![2017-11-23-09-52-29](http://qiniu.xdpie.com/2017-11-23-09-52-29.png)

![2017-11-23-09-54-41](http://qiniu.xdpie.com/2017-11-23-09-54-41.png)

![2017-11-23-09-56-39](http://qiniu.xdpie.com/2017-11-23-09-56-39.png)

![2017-11-23-09-58-28](http://qiniu.xdpie.com/2017-11-23-09-58-28.png)

![2017-11-23-09-59-03](http://qiniu.xdpie.com/2017-11-23-09-59-03.png)

![2017-11-23-10-02-12](http://qiniu.xdpie.com/2017-11-23-10-02-12.png)

    
 
