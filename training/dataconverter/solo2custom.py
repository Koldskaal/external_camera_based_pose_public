import argparse
import multiprocessing
import random
import shutil
import sys
from pathlib import Path
from PIL import Image

from pysolotools.consumers import Solo
from pysolotools.core import BoundingBox2DAnnotation, RGBCameraCapture
from pysolotools.core.models import Frame

from tqdm import tqdm



class Solo2CustomConverter:
    def __init__(self, solo: Solo):
        self._solo = solo
        self._pool = multiprocessing.Pool(processes=multiprocessing.cpu_count())
        self.progress = 0

    @staticmethod
    def _filter_annotation(ann_type, annotations):
        filtered_ann = list(
            filter(
                lambda k: isinstance(
                    k,
                    ann_type,
                ),
                annotations,
            )
        )
        if filtered_ann:
            return filtered_ann[0]
        return filtered_ann

    @staticmethod
    def _process_rgb_image(image_id, rgb_capture, output, data_root, sequence_num, bbox):
        image_file = data_root / f"sequence.{sequence_num}/{rgb_capture.filename}"
        image_to_file = f"camera_{image_id}.png"
        image_to = output / image_to_file
        im = Image.open(str(image_file))
        w, h = im.size
        im = im.crop(bbox)
        # print(im, w, h)
        # im.show()
        im.save(str(image_to))

    @staticmethod
    def _to_yolo_bbox(img_width, img_height, center_x, center_y, box_width, box_height):
        x = (center_x + box_width * 0.5) / img_width
        y = (center_y + box_height * 0.5) / img_height
        w = box_width / img_width
        h = box_height / img_height

        return x, y, w, h

    @staticmethod
    def _process_annotations(image_id, rgb_capture, output, images_output, data_root, sequence_num, metrics):

        width, height = rgb_capture.dimension
        filename = f"camera_{image_id}.txt"
        file_to = output / filename

        bbox_ann = Solo2CustomConverter._filter_annotation(
            ann_type=BoundingBox2DAnnotation, annotations=rgb_capture.annotations
        ).values
        size = 100
        with open(str(file_to), "w") as f:
            for bbox in bbox_ann:
                left = (bbox.origin[0] + bbox.dimension[0] * 0.5) - size/2
                top = (bbox.origin[1] + bbox.dimension[1] * 0.5) - size/2
                right = left+size #
                bottom = top+size #bbox.dimension[1]
                if left < 0:
                    continue
                if top < 0: continue
                if right > width: # skip out of bounds images
                    continue
                if bottom > height:
                    continue

                Solo2CustomConverter._process_rgb_image(image_id, rgb_capture, images_output, data_root, sequence_num, 
                                                        (left, # left
                                                         top, # top
                                                         right, #right
                                                         bottom, #bottom
                                                         ) 
                                                        )
                
                f.write(f"{metrics}\n")

    @staticmethod
    def _process_instances(frame: Frame, idx, images_output, labels_output, data_root):
        image_id = idx
        sequence_num = frame.sequence

        # Currently support only single camera
        rgb_capture = list(
            filter(lambda cap: isinstance(cap, RGBCameraCapture), frame.captures)
        )[0]

        # hardcoded
        rot_data = list(frame.metrics[2]["values"][0]["instances"][0]["transformRecord"]["rotationEuler"])
        rot_data = " ".join([str(rot) for rot in rot_data])
        Solo2CustomConverter._process_annotations(image_id, rgb_capture, labels_output, images_output, data_root, sequence_num, rot_data)

    def convert(self, output_path: str):
        base_path = Path(output_path)
        train_path = base_path / "train"
        test_path = base_path / "test"
        images_output_train = train_path / "images"
        labels_output_train = train_path / "labels"
        images_output_train.mkdir(parents=True, exist_ok=True)
        labels_output_train.mkdir(parents=True, exist_ok=True)

        images_output_test = test_path / "images"
        labels_output_test = test_path / "labels"
        images_output_test.mkdir(parents=True, exist_ok=True)
        labels_output_test.mkdir(parents=True, exist_ok=True)

        data_path = Path(self._solo.data_path)

        for idx, frame in enumerate(self._solo.frames()):
            if idx < 100:
                continue
            if idx < 200:
                self._pool.apply_async(
                    self._process_instances,
                    args=(frame, idx, images_output_test, labels_output_test, data_path),
                )
            else:
                self._pool.apply_async(
                        self._process_instances,
                        args=(frame, idx, images_output_train, labels_output_train, data_path),
                    )

        self._pool.close()
        self._pool.join()


def cli():
    parser = argparse.ArgumentParser(
        prog="solo2custom",
        description=("Converts SOLO datasets into Custom datasets",),
        epilog="\n",
    )

    parser.add_argument("solo_path")
    parser.add_argument("dest_path")

    args = parser.parse_args(sys.argv[1:])
    solo = Solo(args.solo_path)

    converter = Solo2CustomConverter(solo)

    converter.convert(args.dest_path)


if __name__ == "__main__":
    cli()
