import tensorflow as tf
from vgg19 import Vgg19
import matplotlib.pyplot as plt

class_name = ['not fire', 'fire']


def test_image(path_image, num_class):
    img_string = tf.read_file(path_image)
    img_decoded = tf.image.decode_png(img_string, channels=3)
    img_resized = tf.image.resize_images(img_decoded, [224, 224])
    img_resized = tf.reshape(img_resized, shape=[1, 224, 224, 3])
    model = Vgg19(bgr_image=img_resized, num_class=num_class, vgg19_npy_path='./vgg19.npy')
    score = model.fc8
    prediction = tf.argmax(score, 1)
    saver = tf.train.Saver()
    with tf.Session() as sess:
        sess.run(tf.global_variables_initializer())
        saver.restore(sess, "./tmp/checkpoints/model_epoch3.ckpt")

        plt.imshow(img_decoded.eval())
        plt.title("Class:" + class_name[sess.run(prediction)[0]])
        plt.show()


test_image('./validate/11.jpg', 2)
