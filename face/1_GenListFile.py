import os
import argparse

def gen_train_file(data_root, train_file):
    """Generate the train file, which has the following format.
    relative_path0 label0
    relative_path1 label1
    relative_path2 label2
    """
    train_file_buf = open(train_file, 'w')
    id_list = os.listdir(data_root)
    id_list.sort()
    for label, id_name in enumerate(id_list):
        cur_id_folder = os.path.join(data_root, id_name)
        # if cur_id_folder has '.txt' string, then skip it
        if cur_id_folder.find('.txt') != -1: continue
        print("\t", id_name)
        cur_img_list = os.listdir(cur_id_folder)
        cur_img_list.sort()
        for index, image_name in enumerate(cur_img_list):
            if not image_name.endswith(('.png', '.jpg', '.jpeg')): continue
            cur_image_path = os.path.join(id_name, image_name)
            line = cur_image_path + ' ' + str(label)
            train_file_buf.write(line + '\n')
    
if __name__ == '__main__':
    print("1_GenListFile.py: Generate image list text file.")
    conf = argparse.ArgumentParser(description='prepare file lists')
    conf.add_argument("--print_train_list", type = int, help = "To print train list file")
    conf.add_argument("--train_data_root",  type = str, help = "Path to the train data")
    conf.add_argument("--train_data_list",  type = str, help = "File name of the train_data_list")
    args = conf.parse_args()

    # file to be generate.
    if args.print_train_list > 0:
        print("train file ID:")
        gen_train_file(args.train_data_root, args.train_data_list)
        
    # Set 2_train.sh
    f = open("2_Train.sh", "w")
    f.write("python 2_Train.py \\\n")
    f.write("    --data_root '" + args.train_data_root + "' \\\n")
    f.write("    --train_file '" + args.train_data_list + "' \\\n")
    f.write("    --backbone_type 'MobileFaceNet' \\\n")
    f.write("    --backbone_conf_file 'config/backbone_conf.yaml' \\\n")
    f.write("    --head_type 'MagFace' \\\n")
    f.write("    --head_conf_file 'config/head_conf.yaml' \\\n")
    f.write("    --lr 0.1 \\\n")
    f.write("    --out_dir '" + args.train_data_root + "' \\\n")
    f.write("    --epoches 18 \\\n")
    f.write("    --step '10, 13, 16' \\\n")
    f.write("    --print_freq 200 \\\n")
    f.write("    --save_freq 3000 \\\n")
    f.write("    --batch_size 10 \\\n")
    f.write("    --momentum 0.9 \\\n")
    f.write("    --log_dir 'logs' \\\n")
    f.write("    --tensorboardx_logdir 'mv-hrnet' \\\n")
    f.write("    2>&1 | tee train.log")

    print("1_GenListFile.py: Done!")