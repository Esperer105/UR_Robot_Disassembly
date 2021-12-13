import tensorflow as tf
from vgg19 import Vgg19
from create_batch_images import ImageDataGenerator
from datetime import datetime
import glob
import os

learning_rate = 1e-4
num_epochs = 100  # 代的个数
num_iter = 50
batch_size = 3
num_classes = 2  # 类别标签
train_layer = ['fc8', 'fc7', 'fc6']

filewriter_path = "tmp/tensorboard"  # 存储tensorBoard文件
checkpoint_path = "tmp/checkpoints"  # 训练好的模型和参数存放目录

#对齐时的图像
training_obstacle_align_image_path = 'rokae_image/train/align/'

#没有障碍物的图像
training_non_align_obstacle_image_path = 'rokae_image/train/no_align/'





testing_obstacle_align_image_path = 'rokae_image/test/align/'
testing_non_align_obstacle_image_path = 'rokae_image/test/no_align/'
training_labels = []
testing_labels = []

# 加载训练数据
training_images = glob.glob(training_obstacle_align_image_path + '*.jpg')
training_images[len(training_images):len(training_images)] = glob.glob(training_non_align_obstacle_image_path + '*.jpg')
for i in range(len(training_images)):   # 对图像打标
    if i < 80:
        training_labels.append(1)
    else:
        training_labels.append(0)



training = ImageDataGenerator(
    images=training_images,
    labels=training_labels,
    batch_size=batch_size,
    num_classes=num_classes)

# 加载测试数据
testing_images = glob.glob(testing_obstacle_align_image_path + '*.jpg')
testing_images[len(testing_images):len(testing_images)] = glob.glob(testing_non_align_obstacle_image_path + '*.jpg')
for i in range(len(testing_images)):
    if i < 20:
        testing_labels.append(1)
    else:
        testing_labels.append(0)

testing = ImageDataGenerator(
    images=testing_images,
    labels=testing_labels,
    batch_size=batch_size,
    num_classes=num_classes)

x = tf.placeholder(tf.float32, [batch_size, 224, 224, 3])
y = tf.placeholder(tf.float32, [batch_size, num_classes])
keep_prob = tf.placeholder(tf.float32)

# 图片数据通过VGG19网络处理
model = Vgg19(bgr_image=x, num_class=num_classes, vgg19_npy_path='vgg19.npy')
score = model.fc8

# List of trainable variables of the layers we want to train
var_list = [v for v in tf.trainable_variables() if v.name.split('/')[0] in train_layer]

with tf.name_scope('loss'):
    loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=score, labels=y))

gradients = tf.gradients(loss, var_list)

gradients = list(zip(gradients, var_list))

with tf.name_scope('optimizer'):
    # 优化器，采用梯度下降算法进行优化
    optimizer = tf.train.GradientDescentOptimizer(learning_rate)
    train_op = optimizer.apply_gradients(grads_and_vars=gradients)

with tf.name_scope("accuracy"):
    # 定义网络精确度
    correct_pred = tf.equal(tf.argmax(score, 1), tf.argmax(y, 1))
    accuracy = tf.cast(correct_pred, tf.float32)

# 把精确度加入到TensorBoard
tf.summary.scalar('loss', loss)

merged_summary = tf.summary.merge_all()
writer = tf.summary.FileWriter(filewriter_path)
saver = tf.train.Saver()

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())
    tf.train.start_queue_runners(sess=sess)

    # 把模型图加入TensorBoard
    writer.add_graph(sess.graph)

    # 总共训练100代
    for epoch in range(num_epochs):

        print("{} Epoch number: {} start".format(datetime.now(), epoch + 1))
        # 开始训练每一代
        
        for step in range(num_iter):
            img_batch = sess.run(training.image_batch)
            label_batch = sess.run(training.label_batch)
            sess.run(train_op, feed_dict={x: img_batch, y: label_batch})

        # 测试模型精确度
        print("{} Start testing".format(datetime.now()))

        tp = tn = fn = fp = 0

        for _ in range(num_iter):
            img_batch = sess.run(testing.image_batch)
            label_batch = sess.run(testing.label_batch)
            softmax_prediction = sess.run(score, feed_dict={x: img_batch, y: label_batch})
            prediction_label = sess.run(tf.argmax(softmax_prediction, 1))
            actual_label = sess.run(tf.argmax(label_batch, 1))

            for i in range(len(prediction_label)):
                if prediction_label[i] == actual_label[i] == 1:
                    tp += 1
                elif prediction_label[i] == actual_label[i] == 0:
                    tn += 1
                elif prediction_label[i] == 1 and actual_label[i] == 0:
                    fp += 1
                elif prediction_label[i] == 0 and actual_label[i] == 1:
                    fn += 1
        denominator=tp + fp
        precision=0
        if  denominator is  0:
            precision=0
        else  :
            precision = tp / (tp + fp)
        recall = tp / (tp + fn)
        f1 = (2 * tp) / (2 * tp + fp + fn)  # f1为精确率precision和召回率recall的调和平均
        print("{} Testing Precision = {:.4f}".format(datetime.now(), precision))
        print("{} Testing Recall = {:.4f}".format(datetime.now(), recall))
        print("{} Testing F1 = {:.4f}".format(datetime.now(), f1))

    # 把训练好的模型存储起来
    print("{} Saving checkpoint of model...".format(datetime.now()))

    checkpoint_name = os.path.join(checkpoint_path, 'model_epoch' + str(epoch + 1) + '.ckpt')
    print("{} Epoch number: {} end".format(datetime.now(), epoch + 1))
    save_path = saver.save(sess, checkpoint_name)
