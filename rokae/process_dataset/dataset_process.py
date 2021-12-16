import json
import os
from PIL import Image
import shutil


# 获取相关路径下文件夹中标号最大的文件
def return_max(path):
    files = os.listdir(path)
    if not files:
        max_num = 0
    else:
        files.sort(key=lambda x: int(x[:-4]))
        str = files[-1]
        max_num = int(str.split('.', 1)[0])
    return max_num


def data_reset(path):
    """重置原始数据集"""
    for root, dirs, files in os.walk(path):
        for dir in dirs:
            if dir == "out":
                shutil.rmtree(os.path.join(root, dir))


def recog_dir(recog_path):
    collect = []
    types = ["bolt1", "bolt1_hole", "bolt2", "bolt2_hole", "bolt3", "bolt3_hole", "bolt4", "bolt4_hole", "shell_hole",
             "neg"]
    for root, dirs, files in os.walk(recog_path):
        for name in dirs:
            if search(os.path.basename(os.path.join(root, name)), types):
                collect.append(os.path.join(root, name))
    return collect


def search(a, list):
    for i in list:
        if i == a:
            return True
    return False


def convert(input):
    if input == 1:
        para = 'bolt1'
    if input == 2:
        para = 'bolt1_hole'
    if input == 3:
        para = 'bolt2'
    if input == 4:
        para = 'bolt2_hole'
    if input == 5:
        para = 'bolt3'
    if input == 6:
        para = 'bolt3_hole'
    if input == 7:
        para = 'bolt4'
    if input == 8:
        para = 'bolt4_hole'
    if input == 9:
        para = 'shell_hole'
    if input == 10:
        para = 'neg'
    return para


# srcpath为文件的原始绝对路径
# dstpath为文件的目标绝对路径
def move_file(srcpath, dstpath):
    if not os.path.isfile(srcpath):
        print("%s not exist!" % (srcpath))
    else:
        fpath, fname = os.path.split(dstpath)
        if not os.path.exists(fpath):
            os.makedirs(fpath)  # 创建路径
        shutil.copyfile(srcpath, dstpath)
        os.remove(srcpath)


def build_category(save_path):
    types = ["bolt1", "bolt1_hole", "bolt2", "bolt2_hole", "bolt3", "bolt3_hole", "bolt4", "bolt4_hole", "shell_hole",
             "neg"]
    for type in types:
        if os.path.exists(os.path.join(save_path, type)):
            print("folder exists")
        else:
            os.makedirs(os.path.join(save_path, type))


def recog_json(recog_path):
    collect = []
    types = ["instances_default.json"]
    for root, dirs, files in os.walk(recog_path):
        for name in files:
            if search(os.path.basename(os.path.join(root, name)), types):
                collect.append(os.path.join(root, name))
    return collect


# 识别文件路径
def cut_main(recog_path):
    for jsonfile in recog_json(recog_path):

        with open(jsonfile, encoding='utf-8') as f:
            # 打开json文件，提取其中的元素
            line = f.readline()
            d = json.loads(line)
            licenses = d['licenses']
            info = d['info']
            categories = d['categories']
            images = d['images']
            annotations = d['annotations']
            subpath = os.path.split(jsonfile)[0]
            sub = subpath.split('\\')[-2]
            # 图片加载路径
            load_path = os.path.join(recog_path, sub, 'images')
            # 图片保存路径
            save_path = os.path.join(recog_path, sub, 'out')
            if not os.path.exists(os.path.join(recog_path, sub, 'out')):
                os.makedirs(os.path.join(recog_path, sub, 'out'))
            # 新建螺栓分类文件夹
        for category in categories:
            path1 = os.path.join(save_path, category['name'])
            if os.path.exists(path1):
                print("folder exists")
            else:
                os.makedirs(path1)
        i = 1
        for annotation in annotations:
            # 便于将螺栓分类
            para = convert(annotation['category_id'])
            # 图片裁剪操作
            bbox = annotation['bbox']
            box = [bbox[0] - 2, bbox[1] - 2, bbox[0] + bbox[2] + 2,
                   bbox[1] + bbox[3] + 2]
            imagename = images[annotation['image_id'] - 1]['file_name']
            img = Image.open(os.path.join(load_path, imagename))
            crop = img.crop(box)
            # resize = crop.resize((100, 100))
            j = annotation['id'] - 1
            # 图片保存操作
            if j != (len(annotations) - 1):
                if annotations[j]['category_id'] != annotations[j + 1]['category_id']:
                    i_str = str(i) + '.png'
                    crop.save(os.path.join(save_path, para, i_str), quality=95)
                    i = return_max(os.path.join(save_path, convert(annotations[j + 1]['category_id']))) + 1
                else:
                    i_str = str(i) + '.png'
                    crop.save(os.path.join(save_path, para, i_str), quality=95)
                    i = i + 1
            else:
                i = return_max(os.path.join(save_path, para)) + 1
                i_str = str(i) + '.png'
                crop.save(os.path.join(save_path, para, i_str), quality=95)


def merge_file(target_path, recog_path):
    """target_path目标路径（图片输出路径）
    recog_path识别路径（coco数据集的路径）"""
    build_category(target_path)
    for dir in recog_dir(recog_path):
        file_abs = []
        files = os.listdir(dir)
        for file in files:
            file_abs.append(os.path.join(dir, file))
        if return_max(os.path.join(target_path, os.path.basename(dir))) == 0:
            for abs in file_abs:
                move_file(abs, os.path.join(target_path, os.path.basename(dir), os.path.basename(abs)))
        else:
            i = 1
            const = return_max(os.path.join(target_path, os.path.basename(dir)))
            for abs in file_abs:
                filename, extention = os.path.splitext(os.path.basename(abs))
                name_num = const + i
                rename = str(name_num) + extention
                move_file(abs, os.path.join(target_path, os.path.basename(dir), rename))
                i = i + 1
    path1 = os.path.join(target_path, "out")
    if os.path.exists(path1):
        print("folder exists")
    else:
        os.makedirs(path1)
    for file in os.listdir(target_path):
        if file == "out":
            continue
        dir_path = os.path.join(target_path, file)
        i = return_max(path1) + 1
        for pic in os.listdir(dir_path):
            pic_path = os.path.join(dir_path, pic)
            new_pic = str(i) + ".jpg"
            move_path = os.path.join(path1, new_pic)
            shutil.copyfile(pic_path, move_path)
            i += 1


if __name__ == "__main__":
    # 原始数据集的路径
    data_path = "D:\\标注\\"
    # 数据集处理之后的目标路径
    target_path = "D:\\Realsense\\test_code"
    # 对原始数据集进行crop操作
    cut_main(data_path)
    merge_file(target_path, data_path)
