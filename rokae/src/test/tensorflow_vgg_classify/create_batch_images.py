import tensorflow as tf

VGG_MEAN = tf.constant([123.68, 116.779, 103.939], dtype=tf.float32)


# 把图片数据转化为三维矩阵
class ImageDataGenerator(object):
    def __init__(self, images, labels, batch_size, num_classes):
        self.filenames = images
        self.labels = labels
        self.batch_size = batch_size
        self.num_class = num_classes
        self.image_batch, self.label_batch = self.image_decode()

    """
    def read_decode_images(self, files, labels):
        image_list = []
        for i in range(len(files)):
            file = str(files[i], encoding='utf8')
            image_content = tf.read_file(file)
            image_data = tf.image.decode_jpeg(image_content, channels=3)
            image = tf.image.resize_images(image_data, [224, 224])
            img_centered = tf.subtract(image, VGG_MEAN)
            img_bgr = img_centered[:, :, ::-1]
            image_list.append(img_bgr)
        image = tf.convert_to_tensor(image_list, dtype=None)
        label_batch = tf.one_hot(labels, self.num_class, dtype=tf.uint8)
        return image, label_batch

    def batch_input(self):
        # 将文件列表和标签转化为tensor，存入文件名列表
        file_queue = tf.train.slice_input_producer([self.filenames, self.labels])

        image_batch, label_batch = tf.train.shuffle_batch([file_queue[0], file_queue[1]],
                                                          batch_size=self.batch_size,
                                                          capacity=2000,
                                                          min_after_dequeue=1000)
        with tf.Session() as sess:
            sess.run(tf.global_variables_initializer())
            tf.train.start_queue_runners(sess=sess)
            image_batch, label_batch = sess.run([image_batch, label_batch])
            return image_batch, label_batch
    """

    def image_decode(self):
        file_queue = tf.train.slice_input_producer([self.filenames, self.labels])

        image_content = tf.read_file(file_queue[0])
        image_data = tf.image.decode_jpeg(image_content, channels=3)
        image = tf.image.resize_images(image_data, [224, 224])
        img_centered = tf.subtract(image, VGG_MEAN)
        img_bgr = img_centered[:, :, ::-1]

        labels = tf.one_hot(file_queue[1],self.num_class, dtype=tf.uint8)

        image_batch, label_batch = tf.train.shuffle_batch([img_bgr, labels],
                                                          batch_size=self.batch_size,
                                                          capacity=2000,
                                                          min_after_dequeue=1000)
        return image_batch, label_batch
