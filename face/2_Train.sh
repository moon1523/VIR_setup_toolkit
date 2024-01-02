python 2_Train.py \
    --data_root './train_data/ab' \
    --train_file './train_data/ab/image_list.txt' \
    --backbone_type 'MobileFaceNet' \
    --backbone_conf_file 'config/backbone_conf.yaml' \
    --head_type 'MagFace' \
    --head_conf_file 'config/head_conf.yaml' \
    --lr 0.1 \
    --out_dir './train_data/ab' \
    --epoches 18 \
    --step '10, 13, 16' \
    --print_freq 200 \
    --save_freq 3000 \
    --batch_size 10 \
    --momentum 0.9 \
    --log_dir 'logs' \
    --tensorboardx_logdir 'mv-hrnet' \
    2>&1 | tee train.log