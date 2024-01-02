import sys
sys.path.append('.')
import logging.config
logging.config.fileConfig("config/logging.conf")
logger = logging.getLogger('api')

import os
import yaml
import argparse
import cv2
import numpy as np
from core.model_loader.face_detection.FaceDetModelLoader import FaceDetModelLoader
from core.model_handler.face_detection.FaceDetModelHandler import FaceDetModelHandler
from core.model_loader.face_alignment.FaceAlignModelLoader import FaceAlignModelLoader
from core.model_handler.face_alignment.FaceAlignModelHandler import FaceAlignModelHandler
from core.image_cropper.arcface_cropper.FaceRecImageCropper import FaceRecImageCropper
from data_processor.train_dataset import ImageDataset
import subprocess

with open('config/model_conf.yaml') as f:
    model_conf = yaml.load(f, Loader=yaml.FullLoader)

model_path = 'models'
scene = 'mask'

model_category = 'face_detection'
model_name =  model_conf[scene][model_category]
logger.info('Start to load the face detection model...')

# load model
try:
    faceDetModelLoader = FaceDetModelLoader(model_path, model_category, model_name)
except Exception as e:
    logger.error('Failed to parse model configuration file!')
    logger.error(e)
    sys.exit(-1)
else:
    logger.info('Successfully parsed the model configuration file model_meta.json!')
try:
    model, cfg = faceDetModelLoader.load_model()
except Exception as e:
    logger.error('Model loading failed!')
    logger.error(e)
    sys.exit(-1)
else:
    logger.info('Successfully loaded the face detection model!')
faceDetModelHandler = FaceDetModelHandler(model, 'cuda:0', cfg)

model_category = 'face_alignment'
model_name =  model_conf[scene][model_category]
logger.info('Start to load the face landmark model...')

# load model
try:
    faceAlignModelLoader = FaceAlignModelLoader(model_path, model_category, model_name)
except Exception as e:
    logger.error('Failed to parse model configuration file!')
    logger.error(e)
    sys.exit(-1)
else:
    logger.info('Successfully parsed the model configuration file model_meta.json!')
try:
    model, cfg = faceAlignModelLoader.load_model()
except Exception as e:
    logger.error('Model loading failed!')
    logger.error(e)
    sys.exit(-1)
else:
    logger.info('Successfully loaded the face landmark model!')
faceAlignModelHandler = FaceAlignModelHandler(model, 'cuda:0', cfg)

if __name__ == '__main__':
    conf = argparse.ArgumentParser(description='crop the faces from the data')
    conf.add_argument("--train_video", type = str, help = "train video name")
    conf.add_argument("--output_dir", type = str, help = "path to save cropped images")
    conf.add_argument("--threshold", type = float, default=0.9, help = "threshold for FaceDetModelHandler")
    args = conf.parse_args()   
    threshold = args.threshold
    face_cropper = FaceRecImageCropper()
    
    # if args.output_dir of last word has '/', remove it
    if args.output_dir[-1] == '/':
        args.output_dir = args.output_dir[:-1]

    # Find camera index
    camera_index = 0
    command = 'v4l2-ctl --list-devices'
    result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)    
    output_lines = result.stdout.split('\n')
    found_index = False
    for line in output_lines:
        if 'APC930' or 'UHD2160' in line:
            found_index = True
        elif found_index:
            camera_index = int(line[-1])
            break
    
    cap = cv2.VideoCapture(camera_index)
    key = ''
    # if camera is not opened, exit program
    if not cap.isOpened():
        print("Camera is not opened, please check your camera")
        sys.exit(0)
    
    print('\n\n\n\n')
    print("0_CropFaces.py: capture only one face in the video and categorize it into the folder of the person's name")
    print("Enter the folder name to saving")
    folder_name = input()
    folder_name = args.output_dir + '/' + folder_name
    
    person_id = 0
    isExit = False
    while True:
        print("\n\nEnter the name of the person, if you want to quit, enter 'q'")
        print("  ** '" + str(person_id) + "_name' will be created **")
        person_name = input()
        if person_name == "":
            print("Please enter a valid name: ")
        elif person_name == "q":
            break
        
        person_folder = folder_name + '/' + str(person_id) + '_' + person_name
        print("path: ", person_folder)
        
        id = 0
        ret, img = cap.read()
        aspect_ratio = img.shape[1] / img.shape[0]
        while True:
            ret, img = cap.read()
            if ret:
                # img = cv2.resize(img, (int(900*aspect_ratio), 900))
                try:
                    dets = faceDetModelHandler.inference_on_image(img)
                except Exception as e:
                    logger.error('Face detection failed!')
                    logger.error(e)
                    sys.exit(-1)
                image_show = img.copy()
                if dets.shape[0] == 1:
                    try:
                        for det in dets:
                            if det[4]<threshold: continue
                            landmarks = faceAlignModelHandler.inference_on_image(img, det)
                            for (x, y) in landmarks.astype(np.int32):
                                cv2.circle(image_show, (x, y), 2, (255, 0, 0),-1)
                            landmarks = landmarks.ravel().astype(np.int32)
                            cropped_image = face_cropper.crop_image_by_mat(img, landmarks)
                            save_path_img = os.path.join(person_folder, str(id)+".jpg")
                            id += 1
                            os.makedirs(os.path.dirname(save_path_img), exist_ok=True)
                            cv2.putText(image_show, str(id), (int(det[0]), int(det[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            cv2.imwrite(save_path_img, cropped_image)
                    except Exception as e:
                        logger.error('Face landmark failed!')
                        logger.error(e)
                        sys.exit(-1)

                cv2.imshow(person_name, image_show)
                key = cv2.waitKey(1)
                
                if key == ord('q'):
                    cv2.destroyWindow(person_name)
                    break
            if isExit:
                break
        person_id += 1
            
    # Set 1_GenListFile.sh
    f = open("1_GenListFile.sh", "w")
    f.write("python 1_GenListFile.py \\\n")
    f.write("    --print_train_list 1 \\\n")
    f.write("    --train_data_root '" + folder_name + "' \\\n")
    f.write("    --train_data_list '" + folder_name + '/'+ "image_list.txt'\n")
    
    print("0_CropFaces.py: Done!")