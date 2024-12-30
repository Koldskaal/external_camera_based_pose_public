import argparse
import multiprocessing
import shutil
import sys
from pathlib import Path

from pysolotools.consumers import Solo
from pysolotools.core import BoundingBox2DAnnotation, RGBCameraCapture
from pysolotools.core.models import Frame

import yaml


class Solo2YoloConverter:
    def __init__(self, solo: Solo):
        self._solo = solo
        self._pool = multiprocessing.Pool(processes=multiprocessing.cpu_count())

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
    def _process_rgb_image(image_id, rgb_capture, output, data_root, sequence_num):
        image_file = data_root / f"sequence.{sequence_num}/{rgb_capture.filename}"
        image_to_file = f"camera_{image_id}.png"
        image_to = output / image_to_file
        shutil.copy(str(image_file), str(image_to))

    @staticmethod
    def _to_yolo_bbox(img_width, img_height, center_x, center_y, box_width, box_height):
        x = (center_x + box_width * 0.5) / img_width
        y = (center_y + box_height * 0.5) / img_height
        w = box_width / img_width
        h = box_height / img_height

        return x, y, w, h

    @staticmethod
    def _process_annotations(image_id, rgb_capture, output):
        width, height = rgb_capture.dimension
        filename = f"camera_{image_id}.txt"
        file_to = output / filename

        bbox_ann = Solo2YoloConverter._filter_annotation(
            ann_type=BoundingBox2DAnnotation, annotations=rgb_capture.annotations
        ).values

        with open(str(file_to), "w") as f:
            for bbox in bbox_ann:
                x, y, w, h = Solo2YoloConverter._to_yolo_bbox(
                    width,
                    height,
                    bbox.origin[0],
                    bbox.origin[1],
                    bbox.dimension[0],
                    bbox.dimension[1],
                )

                f.write(f"{bbox.labelId-1} {x:.8f} {y:.8f} {w:.8f} {h:.8f}\n")

    @staticmethod
    def _process_instances(frame: Frame, idx, images_output, labels_output, data_root):
        image_id = idx
        sequence_num = frame.sequence

        # Currently support only single camera
        rgb_capture = list(
            filter(lambda cap: isinstance(cap, RGBCameraCapture), frame.captures)
        )[0]

        Solo2YoloConverter._process_rgb_image(
            image_id, rgb_capture, images_output, data_root, sequence_num
        )
        Solo2YoloConverter._process_annotations(image_id, rgb_capture, labels_output)

    def convert(self, output_path: str, max_iter: int):
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

        # create yaml file
        config = {
            'train': 'train/images',
            'val': 'test/images',
            'nc' : 1,
            'names': ['robot']
        }

        with open(str(base_path/"data.yaml"), "w") as f:
            yaml.dump(config, f)

        for idx, frame in enumerate(self._solo.frames()):
            # if idx < 200:
            #     self._pool.apply_async(
            #         self._process_instances,
            #         args=(frame, idx, images_output_test, labels_output_test, data_path),
            #     )
            # else:
            self._pool.apply_async(
                self._process_instances,
                args=(frame, idx, images_output_train, labels_output_train, data_path),
            )
            if max_iter > 0 and idx >= max_iter:
                break
            
            

        self._pool.close()
        self._pool.join()

        



def cli():
    parser = argparse.ArgumentParser(
        prog="solo2yolo",
        description=("Converts SOLO datasets into YOLO datasets",),
        epilog="\n",
    )

    parser.add_argument("solo_path")
    parser.add_argument("yolo_path")
    parser.add_argument("count")

    args = parser.parse_args(sys.argv[1:])

    solo = Solo(args.solo_path)

    converter = Solo2YoloConverter(solo)
    count = int(args.count) if args.count else -1
    converter.convert(args.yolo_path, count)


if __name__ == "__main__":
    cli()
