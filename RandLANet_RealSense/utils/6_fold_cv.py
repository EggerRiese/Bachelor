import numpy as np
import glob, os, sys
import statistics

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(ROOT_DIR)
from helper_ply import read_ply
from helper_tool import Plot

global_test_total_correct = 0
global_test_total_seen = 0
global_gt_classes = [0 for _ in range(13)]
global_positive_classes = [0 for _ in range(13)]
global_true_positive_classes = [0 for _ in range(13)]
global_iou_list = [0 for _ in range(13)]
global_acc_list = [0 for _ in range(13)]

if __name__ == '__main__':
    base_dir = '/home/joshua/Dokumente/Bachelor/github/RandLaNet_RealSense/data/RealSense/results'
    original_data_dir = '/home/joshua/Dokumente/Bachelor/github/RandLaNet_RealSense/data/RealSense/original_ply'
    data_path = glob.glob(os.path.join(base_dir, '*.ply'))
    data_path = np.sort(data_path)
    file_count = 0

    for file_name in data_path:
        file_count += 1

        test_total_correct = 0
        test_total_seen = 0
        gt_classes = [0 for _ in range(13)]
        positive_classes = [0 for _ in range(13)]
        true_positive_classes = [0 for _ in range(13)]
        visualization = False

        number = str(file_name.split('/')[-1][:-4]).split('_')[0]

        pred_data = read_ply(file_name)
        pred = pred_data['pred']
        original_data = read_ply(os.path.join(original_data_dir, file_name.split('/')[-1][:-4] + '.ply'))
        labels = original_data['class']
        points = np.vstack((original_data['x'], original_data['y'], original_data['z'])).T
        
        ##################
        # Visualize data #
        ##################
        if visualization:
            colors = np.vstack((original_data['red'], original_data['green'], original_data['blue'])).T
            xyzrgb = np.concatenate([points, colors], axis=-1)
            Plot.draw_pc(xyzrgb)  # visualize raw point clouds
            Plot.draw_pc_sem_ins(points, labels)  # visualize ground-truth
            Plot.draw_pc_sem_ins(points, pred)  # visualize prediction

        correct = np.sum(pred == labels)
        print(str(file_name.split('/')[-1][:-4]) + '_acc:' + str(correct / float(len(labels))))
        test_total_correct += correct
        test_total_seen += len(labels)
        
        for j in range(len(labels)):
            gt_l = int(labels[j])
            pred_l = int(pred[j])
            gt_classes[gt_l] += 1
            positive_classes[pred_l] += 1
            true_positive_classes[gt_l] += int(gt_l == pred_l)

        iou_list = []
        for n in range(13):
            if true_positive_classes[n] != 0:
                iou = true_positive_classes[n] / float(gt_classes[n] + positive_classes[n] - true_positive_classes[n])
            else:
                iou = true_positive_classes[n]
            iou_list.append(iou)
        
        acc_list = []
        for n in range(13):
            if true_positive_classes[n] != 0:
                acc = true_positive_classes[n] / float(gt_classes[n])                
            else:
                acc = true_positive_classes[n]
            acc_list.append(acc)

        global_test_total_correct += test_total_correct
        global_test_total_seen += test_total_seen
        for n in range(13):
            global_gt_classes[n] = (global_gt_classes[n] + gt_classes[n]) / 2
            global_positive_classes[n] += positive_classes[n]
            global_true_positive_classes[n] += true_positive_classes[n]
            global_iou_list[n] += iou_list[n]
            global_acc_list[n] += acc_list[n]

    for n in range(13):
        if global_acc_list[n] != 0:
            global_acc_list[n] = global_acc_list[n]/file_count
        if global_iou_list[n] != 0:
            global_iou_list[n] = global_iou_list[n]/file_count

    mean_iou = sum(global_iou_list) / 13.0
    print('complete eval accuracy: {}'.format(global_test_total_correct / float(global_test_total_seen)))
    print('mean IOU:{}'.format(mean_iou))
    print(global_iou_list)
    
    mean_acc = sum(global_acc_list) / 13.0
    print('mAcc value is :{}'.format(mean_acc))
    print('median: {}'.format(statistics.median(global_acc_list)))
