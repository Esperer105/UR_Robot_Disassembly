import copy
import os
import json
import shutil

"""
将coco数据集的格式转换为所有图片加一个.txt文件
.txt文件包含了每一张图片所包含的标注标签，前四个值为标注框的左上角和右下角，最后一个值为标签
"""
# 图片和txt文件保存路径
path = "D:\\Realsense\\test_code"
# 数据集识别路径
recog_path = "D:\\标注\\"


def same_name(a, list1):
    i = 0
    for lis in list1:
        if lis == a:
            i += 1
    return i


print("**********  process start  **********")
if not os.path.exists(os.path.join(path, 'out')):
    os.makedirs(os.path.join(path, 'out'))
else:
    print("'out' folder exists")
# txt文件保存路径
txt_path = os.path.join(path, 'out', 'data.txt')
# 图片输出路径
out_path = os.path.join(path, 'out')
bool_var = True
add_num = 0
imagename = []
for recog_dir in os.listdir(recog_path):
    jsonpath = os.path.join(recog_path, recog_dir, "annotations\\instances_default.json")
    imgpath = os.path.join(recog_path, recog_dir, "images")
    with open(jsonpath, encoding='utf-8') as f:
        line = f.readline()
        d = json.loads(line)
        categories = d['categories']
        category_correspond = []
        for category in categories:
            category_correspond.append((category['id'], category['name']))
        category_dict = dict(category_correspond)
        # print(category_dict)
        images = d['images']
        image_correspond = []
        for image in images:
            image_correspond.append((image['id'], image['file_name']))
        image_dict = dict(image_correspond)
        print("**********  please wait  **********")
        image_dict_copy = copy.deepcopy(image_dict)
        # 修改移动后的图片文件名
        for key in image_dict_copy.keys():
            pic_num = image_dict_copy[key].split("_")
            new_num = str(int(pic_num[0]) + add_num)
            value = new_num + "_Color.png"
            image_dict_copy[key] = value
        # 移动相应的文件到目标文件夹
        # 若有相同的文件名更改文件名
        for key1 in image_dict.keys():
            if same_name(image_dict_copy[key1], imagename) == 0:
                shutil.copy(os.path.join(imgpath, image_dict[key1]),
                            os.path.join(out_path, image_dict_copy[key1]))
            else:
                image_name = image_dict_copy[key1].split(".")
                image_dict_copy[key1] = image_name[0] + "(" + str(same_name(image_dict_copy[key1], imagename)) + ")." + \
                                        image_name[1]
                shutil.copy(os.path.join(imgpath, image_dict[key1]),
                            os.path.join(out_path, image_dict_copy[key1]))
        for key2 in image_dict_copy.keys():
            imagename.append(image_dict_copy[key2])
        add_num += len(image_dict)
        annotations = d['annotations']
        bool_num = True
        with open(txt_path, "a") as t:
            for annotation in annotations:
                if bool_num is True:
                    t.write('#')
                    t.write(image_dict_copy[annotation['image_id']])
                    t.write('\n')
                j = annotation['id'] - 1
                if j != (len(annotations) - 1):
                    if annotations[j]['image_id'] != annotations[j + 1]['image_id']:
                        bool_num = True
                    else:
                        bool_num = False
                box = []
                box.append(round(annotation['bbox'][0], 2))
                box.append(round(annotation['bbox'][1], 2))
                box.append(round(annotation['bbox'][0] + annotation['bbox'][2], 2))
                box.append(round(annotation['bbox'][1] + annotation['bbox'][3], 2))
                for bo in box:
                    t.write(str(bo))
                    t.write(',')
                t.write(str(annotation['category_id']))
                t.write('\n')
print("**********  finished   **********")
print("**********  please check the files  **********")
