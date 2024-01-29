import ultralytics
import numpy as np
import os
HOME = os.getcwd()
ultralytics.checks()
from ultralytics import YOLO
model = YOLO(f'{HOME}/yolov8n.pt')

# each class id is assigned a different color
colors = np.random.randint(0, 256, size=(len(model.names), 3))
print(model.names)

# Specify which classes you care about. The rest of classes will be filtered out.
chosen_class_ids = [0] # 0 refers to person, as per model.names


# Download SAM model weights

%cd {HOME}
!mkdir {HOME}/weights
%cd {HOME}/weights

!wget -q https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth

CHECKPOINT_PATH = os.path.join(HOME, "weights", "sam_vit_h_4b8939.pth")
print(CHECKPOINT_PATH, "; exist:", os.path.isfile(CHECKPOINT_PATH))

import torch
import matplotlib.pyplot as plt
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor

DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
sam = sam_model_registry["vit_h"](checkpoint=CHECKPOINT_PATH).to(device=DEVICE)
mask_predictor = SamPredictor(sam)