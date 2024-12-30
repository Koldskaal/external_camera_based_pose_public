import torch
from torch import nn
import torch.nn.functional as F
import torchvision.transforms as transforms
import torchvision
import os
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

import torchvision
from torchvision.models import resnet50, ResNet50_Weights, resnet18, ResNet18_Weights


class DegreeCNN(nn.Module):
    def __init__(self):
        super(DegreeCNN, self).__init__()
        self.resnet = resnet18(weights=ResNet18_Weights.DEFAULT)
        for param in self.resnet.parameters():
            param.required_grad = False
        num_ftrs = self.resnet.fc.in_features
        self.resnet.fc = nn.Linear(num_ftrs, 360)

    def forward(self, x):
        x = self.resnet(x)
        return x  # No need to apply softmax here if using CrossEntropyLoss


train_on_gpu = torch.cuda.is_available()

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = DegreeCNN().to(device)
model.load_state_dict(torch.load("orientation_with_manual.pt", weights_only=True))
model.eval()

transform = torchvision.transforms.Compose(
    [
        transforms.Resize((80, 80)),
        transforms.ToTensor(),
        transforms.Normalize((0.425, 0.415, 0.405), (0.205, 0.205, 0.205)),
    ]
)


def detect(img, label=""):
    input_tensor = transform(img).unsqueeze(0).to(device)

    # Make a prediction
    with torch.no_grad():  # Disable gradient computation for inference
        output = model(input_tensor)

    predicted_degree = torch.argmax(output, dim=1).item()
    # print(f"{label} Predicted Degree: {predicted_degree}")
    return predicted_degree


if __name__ == "__main__":
    import glob

    img_paths = glob.glob("output/images/*.png")

    for img_path in img_paths:
        img = Image.open(img_path)
        detect(img, img_path)
