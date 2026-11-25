import os
import sys
import json
import yaml
import inspect
import numpy as np
from PIL import ImageColor

from abc import ABC, abstractmethod


class BaseSegmenter(ABC):
    PROVIDER_PRIORITY = [
        "TensorrtExecutionProvider",
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]

    def __init__(self, config_file="", model_dir=""):
        self.model = ""
        self.model_dir = model_dir
        self.classes = {}
        self.class_map = {}
        self.palette = np.zeros((0, 3), dtype=np.uint8)
        self.latest_image = None

        self.execution_provider = self.PROVIDER_PRIORITY[-1]
        self.provider_list = self.PROVIDER_PRIORITY[-1:]

        self.config_file = config_file
        if self.config_file:
            self.load_config(self.config_file)

    @staticmethod
    def read_config(config_file):
        with open(config_file, "r") as f:
            return yaml.safe_load(f) or {}

    def load_config(self, config_file):
        config = self.read_config(config_file)

        self.model = self.resolve_model(config.get("model", ""))

        execution_provider = config.get("execution_provider", self.PROVIDER_PRIORITY[-1])
        if execution_provider not in self.PROVIDER_PRIORITY:
            raise ValueError(
                f"'{type(self).__name__}' cannot run on '{execution_provider}', expected one of "
                f"{', '.join(self.PROVIDER_PRIORITY)}"
            )

        self.execution_provider = execution_provider
        provider_index = self.PROVIDER_PRIORITY.index(execution_provider)
        self.provider_list = self.PROVIDER_PRIORITY[provider_index:]

        self.classes = {
            int(entry["id"]): {
                "name": entry["name"],
                "alias": entry.get("alias", entry["name"]),
                "color": entry.get("color", "black"),
                "show": bool(entry.get("show", True)),
            }
            for entry in config.get("classes", [])
        }

        self.build_palette()

    def resolve_model(self, model):
        if not model or os.path.isabs(model):
            return model

        return os.path.join(self.model_dir, model)

    def build_palette(self, id_space=0):
        ids = list(self.classes) + [id_space - 1]
        self.palette = np.zeros((max(ids) + 1, 3), dtype=np.uint8)
        self.class_map = {}

        for id, entry in self.classes.items():
            if not entry["show"]:
                continue

            self.palette[id] = ImageColor.getrgb(entry["color"])
            self.class_map[id] = entry["alias"]

    def match_model_classes(self, id2label):
        mismatches = [
            f"id {id} is '{name}' for the model but '{self.classes[id]['name']}' in the config"
            for id, name in id2label.items()
            if id in self.classes and self.classes[id]["name"] != name
        ]
        if mismatches:
            raise ValueError(
                f"'{self.config_file}' does not match the class map of '{self.model}': "
                + ", ".join(mismatches)
            )

        self.build_palette(id_space=max(id2label) + 1)

    @abstractmethod
    def _process_model_input(self, raw_image):
        pass

    @abstractmethod
    def _segment_in_model(self, model_input):
        pass

    @abstractmethod
    def _process_model_output(self, model_output):
        pass

    def run_segmentation(self, raw_image):
        model_input = self._process_model_input(raw_image)
        model_output = self._segment_in_model(model_input)
        mask = self._process_model_output(model_output)

        return mask


class Bypass(BaseSegmenter):
    def _process_model_input(self, raw_image):
        return raw_image

    def _segment_in_model(self, model_input):
        return model_input

    def _process_model_output(self, model_output):
        return model_output


# Just a mock segmenter, add a new model like this:
class Thresh(BaseSegmenter):
    def _process_model_input(self, raw_image):
        return np.mean(raw_image[..., :3], axis=-1).astype(np.uint8)

    def _segment_in_model(self, model_input):
        return (model_input > 127).astype(np.uint8)

    def _process_model_output(self, model_output):
        return self.palette[model_output]


class Mask2Former(BaseSegmenter):
    # This export runs as a plain ONNX graph, so only these two apply
    PROVIDER_PRIORITY = [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        try:
            # Keeping imports lazy to avoid shared dependencies between segmenters
            # This is reasonable since only one segmenter is implemented per node
            import cv2
            import onnxruntime as ort

            self.cv2 = cv2

            self.session = ort.InferenceSession(self.model, providers=self.provider_list)
            self.providers = self.session.get_providers()
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name

            # The export embeds its own label map and preprocessing
            metadata = self.session.get_modelmeta().custom_metadata_map
            self.input_size = (int(metadata["width"]), int(metadata["height"]))
            self.image_mean = np.array(
                json.loads(metadata["image_mean"]), dtype=np.float32
            ).reshape(3, 1, 1)
            self.image_std = np.array(json.loads(metadata["image_std"]), dtype=np.float32).reshape(
                3, 1, 1
            )

            # Metadata is JSON, so its class ids arrive as strings
            self.match_model_classes(
                {int(id): name for id, name in json.loads(metadata["id2label"]).items()}
            )

        except Exception as e:
            raise e

    def _process_model_input(self, raw_image):
        self.latest_image = raw_image
        image = self.cv2.resize(raw_image, self.input_size, interpolation=self.cv2.INTER_LINEAR)
        image = image.astype(np.float32).transpose(2, 0, 1) / 255.0
        image = (image - self.image_mean) / self.image_std

        return np.ascontiguousarray(image[None])

    def _segment_in_model(self, model_input):
        model_output = self.session.run([self.output_name], {self.input_name: model_input})

        return model_output[0]

    def _process_model_output(self, model_output):
        height, width = self.latest_image.shape[:2]
        scores = model_output[0].transpose(1, 2, 0)
        scores = self.cv2.resize(scores, (width, height), interpolation=self.cv2.INTER_LINEAR)
        raw_mask = scores.argmax(axis=-1)
        mask = self.palette[raw_mask]

        return mask


class SegFormer(BaseSegmenter):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        try:
            # Keeping imports lazy to avoid shared dependencies between segmenters
            # This is reasonable since only one segmenter is implemented per node
            import cv2
            import onnxruntime as ort

            self.cv2 = cv2

            trt_options = {
                "trt_fp16_enable": True,
                "trt_engine_cache_enable": True,
                "trt_engine_cache_path": os.path.join(os.path.dirname(self.model), "trt_cache"),
            }
            providers = [
                (provider, trt_options) if provider == self.PROVIDER_PRIORITY[0] else provider
                for provider in self.provider_list
            ]
            self.session = ort.InferenceSession(self.model, providers=providers)
            self.providers = self.session.get_providers()
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name

            # The export embeds its own label map and preprocessing
            metadata = self.session.get_modelmeta().custom_metadata_map
            self.input_size = (int(metadata["width"]), int(metadata["height"]))
            self.image_mean = np.array(
                json.loads(metadata["image_mean"]), dtype=np.float32
            ).reshape(3, 1, 1)
            self.image_std = np.array(json.loads(metadata["image_std"]), dtype=np.float32).reshape(
                3, 1, 1
            )

            # Metadata is JSON, so its class ids arrive as strings
            self.match_model_classes(
                {int(id): name for id, name in json.loads(metadata["id2label"]).items()}
            )

        except Exception as e:
            raise e

    def _process_model_input(self, raw_image):
        self.latest_image = raw_image
        image = self.cv2.resize(raw_image, self.input_size, interpolation=self.cv2.INTER_LINEAR)
        image = image.astype(np.float32).transpose(2, 0, 1) / 255.0
        image = (image - self.image_mean) / self.image_std

        return np.ascontiguousarray(image[None])

    def _segment_in_model(self, model_input):
        model_output = self.session.run([self.output_name], {self.input_name: model_input})

        return model_output[0]

    def _process_model_output(self, model_output):
        height, width = self.latest_image.shape[:2]
        scores = model_output[0].transpose(1, 2, 0)
        scores = self.cv2.resize(scores, (width, height), interpolation=self.cv2.INTER_LINEAR)
        raw_mask = scores.argmax(axis=-1)
        mask = self.palette[raw_mask]

        return mask


# A new BaseSegmenter childclass appears here with the same class name automatically
class SegmenterDict:
    def __init__(self):
        self.model_dict = {
            name: obj  # key: value pair of dict
            # Iterate over all members of the current module that are classes
            for name, obj in inspect.getmembers(sys.modules[__name__], inspect.isclass)
            # Filter classes that were defined in this module
            if obj.__module__ == sys.modules[__name__].__name__
            # Filter classes that are child-classes of BaseSegmenter
            and issubclass(obj, BaseSegmenter)
            # Filter that are not BaseSegmenter itself
            and obj is not BaseSegmenter
        }

    def get_model(self, model):
        return self.model_dict.get(model, Bypass)
