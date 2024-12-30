from IPython import display

display.clear_output()

import ultralytics

ultralytics.checks()

from ultralytics import YOLO

import glob
from PIL import Image, ImageEnhance
import cv2


model = YOLO("yolo_mix.pt")  # load a custom model


def detect(image):
    img = Image.fromarray(image)
    # img = ImageEnhance.Brightness(img).enhance(1.2)
    # img = ImageEnhance.Contrast(img).enhance(1.2)
    # img.show()
    result = model(img, verbose=False)
    return result[0].boxes


def detect_xy(image):
    bounding_boxes = detect(image)

    x = 0
    y = 0
    biggest_conf = 0

    for box in bounding_boxes:
        # Coordinates of the bounding box
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        x1 = int(x1)
        x2 = int(x2)
        y1 = int(y1)
        y2 = int(y2)

        # Calculate center point
        cx = x1 + (x2 - x1) / 2
        cy = y1 + (y2 - y1) / 2

        # Confidence score
        confidence = box.conf[0].item()

        if confidence > biggest_conf:
            x = cx
            y = cy
            biggest_conf = confidence

    return biggest_conf > 0, x, y


def example():
    image_list = []
    for filename in glob.glob("images/*.png"):
        img = Image.open(filename)
        # img = ImageEnhance.Color(img).enhance(2)
        # img = ImageEnhance.Brightness(img).enhance(1.2)
        # img = ImageEnhance.Contrast(img).enhance(1.2)
        # img.show()
        image_list.append(img)
    # # Run batched inference on a list of images
    results = model(image_list)  # return a list of Results objects
    size = 100
    # Process results list
    for index, result in enumerate(results):
        for box in result.boxes:
            # Coordinates of the bounding box
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            # Calculate center point
            cx = x1 + (x2 - x1) / 2
            cy = y1 + (y2 - y1) / 2

            # Confidence score
            confidence = box.conf[0].item()
            # Class ID
            class_id = int(box.cls[0].item())
            # Class name
            class_name = result.names[class_id]

            left = cx - size / 2
            right = cx + size / 2
            top = cy - size / 2
            bot = cy + size / 2

            crop = image_list[index].crop((left, top, right, bot))
            crop.save(f"output/cropimg{index}.png")
            print(
                f"Detected {class_name} with confidence {confidence:.2f} at center point ({cx:.2f}, {cy:.2f})"
            )

        # if result.boxes:
        #     result.show()  # display to screen
        # result.save(filename="result.jpg")  # save to disk


if __name__ == "__main__":
    image_path = "baseline_Color.png"  # Replace with your image path
    original_image = cv2.imread(image_path)
    # detect(original_image)
    example()
