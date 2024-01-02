import numpy as np
import torch
import torch.nn.functional as F
import argparse
import sys
import os
import cv2
import yaml
import time
import logging.config
logging.config.fileConfig("config/logging.conf")
logger = logging.getLogger('api')
from core.model_loader.face_detection.FaceDetModelLoader import FaceDetModelLoader
from core.model_handler.face_detection.FaceDetModelHandler import FaceDetModelHandler
from core.model_loader.face_alignment.FaceAlignModelLoader import FaceAlignModelLoader
from core.model_handler.face_alignment.FaceAlignModelHandler import FaceAlignModelHandler
from core.image_cropper.arcface_cropper.FaceRecImageCropper import FaceRecImageCropper
from data_processor.test_dataset import CustomTestDataset
from torch.utils.data import Dataset, DataLoader
from utils.extractor.feature_extractor import CommonExtractor
from backbone.backbone_def import BackboneFactory
from utils.model_loader import ModelLoader
import subprocess

def TransformImg(img):
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    _, encoded_image = cv2.imencode('.png', image)
    decoded_image = cv2.imdecode(encoded_image, cv2.IMREAD_UNCHANGED)
    decoded_image = (decoded_image.transpose((2, 0, 1)) - 127.5) / 128.
    decoded_image = torch.from_numpy(decoded_image.astype(np.float32))
    return decoded_image

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
    torch.cuda.empty_cache()
    conf = argparse.ArgumentParser(description='crop the faces from the data')
    # argument data_root
    conf.add_argument("--data_root", type = str,
                  help = "The root folder of training set.")
    conf.add_argument("--model_file", type = str, 
                  help = "model file")
    conf.add_argument("--num_workers", type = int, 
                  help = "number of workers")
    conf.add_argument("--threshold", type = float, default=0.9, 
                  help = "threshold for FaceDetModelHandler")
    args = conf.parse_args()
    threshold = args.threshold
    face_cropper = FaceRecImageCropper()

    text = 'Sample Text'
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.8
    text_color = (0, 255, 0)
    text_thickness = 2
    text_size, _ = cv2.getTextSize(text, font, font_scale, text_thickness)

    model = torch.load(args.model_file)
    logger.info('loaded trained model')
    prototype = model['state_dict']['head.weight']
    prototype = torch.transpose(prototype.to('cuda:0'), 0, 1)
    logger.info('configured prototype')

    backbone_factory = BackboneFactory('MobileFaceNet', 'config/backbone_conf.yaml')
    model_loader = ModelLoader(backbone_factory)
    model = model_loader.load_model(args.model_file)

    feature_extractor = CommonExtractor('cuda:0')

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
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FPS, 15)
    
    # count the folder's number in data_root 
    num = 0
    id = []
    try:
        items = os.listdir(args.data_root)
        folders = [item for item in items if os.path.isdir(os.path.join(args.data_root, item))]
        num = len(folders)
        for folder in folders:
            id.append(folder)
    except Exception as e:
        logger.error('Failed to count the folder number!')
        logger.error(e)
        sys.exit(-1)
    
    key = ''
    if cap.isOpened():
        ret, img = cap.read()
        frame_count = 0
        start_time = time.time()
        
        while True:
            ret, img = cap.read()
            if ret:
                try:
                   dets = faceDetModelHandler.inference_on_image(img)
                except Exception as e:
                   logger.error('Face detection failed!')
                   logger.error(e)
                   sys.exit(-1)
                # perform face detection for extracted faces
                frame_start_time = time.time()
                images = []
                if dets.size > 0:
                    try:
                        for idx, det in enumerate(dets):
                            if det[4] < threshold: continue
                            landmarks = faceAlignModelHandler.inference_on_image(img, det).ravel().astype(np.int32)
                            cropped_image = face_cropper.crop_image_by_mat(img, landmarks)
                            images.append(cropped_image)
                    except Exception as e:
                        logger.error('Face landmark failed!')
                        logger.error(e)
                        sys.exit(-1)
                
                if len(images) > 0:
                    data_loader = DataLoader(CustomTestDataset(images), batch_size=10, num_workers = num, shuffle=False)
                    feature_list = feature_extractor.extract_online(model, data_loader)

                    for idx, feature in enumerate(feature_list):
                        features = torch.tensor(feature).to('cuda:0')
                        similarities = torch.cosine_similarity(features.unsqueeze(0), prototype, dim=1)
                        predicted_class = torch.argmax(similarities).item()
                        similarity_value = similarities[predicted_class].item()
                        text = f"{id[predicted_class]}/{similarity_value:.3f}"
                        det = dets[idx].astype(int)
                        text_x = det[0] + 10
                        text_y = det[1] + text_size[1] + 10
                        if similarity_value > 0.8:
                            text_color = (0, 255, 0)
                        else:
                            text_color = (0, 0, 255)
                        cv2.rectangle(img, (det[0], det[1]), (det[2], det[3]), text_color, 2)
                        img = cv2.putText(img, text, (text_x, text_y), font, font_scale, text_color, text_thickness, cv2.LINE_AA)
                        cv2.imshow(args.data_root, img)
                        key = cv2.waitKey(1)
                else:
                    cv2.imshow(args.data_root, img)
                    key = cv2.waitKey(1)
                    
                frame_end_time = time.time()
                frame_processing_time = frame_end_time - frame_start_time
                frame_fps = 1.0 / frame_processing_time
                frame_count += 1
            else:
                break
            if key == ord('q'):
                break
            
        
    end_time = time.time()
    video_duration = end_time - start_time
    total_fps = frame_count / video_duration
