"""Semantic segmenters: model-specific pre/post-processing behind a common interface."""

import inspect
import json
import os
import sys
from abc import ABC, abstractmethod

import numpy as np


class BaseSegmenter(ABC):
    PROVIDER_PRIORITY = [
        'TensorrtExecutionProvider',
        'CUDAExecutionProvider',
        'CPUExecutionProvider',
    ]

    def __init__(self, model='', config_file='', execution_provider='', trt_cache_dir=''):
        self.model = model
        self.config_file = config_file
        self.trt_cache_dir = trt_cache_dir
        self.classes = {}
        self.groups = []
        self.default_group = 0
        self.group_matrix = None
        self.class_map = {}
        self.palette = np.zeros((0, 3), dtype=np.uint8)
        self.latest_image = None

        self.execution_provider = self.PROVIDER_PRIORITY[0]
        self.provider_list = list(self.PROVIDER_PRIORITY)

        if self.config_file:
            self.load_config(self.config_file)
        if execution_provider:
            self.set_execution_provider(execution_provider)

    @staticmethod
    def read_config(config_file):
        import yaml

        with open(config_file, 'r') as f:
            return yaml.safe_load(f) or {}

    def set_execution_provider(self, execution_provider):
        if execution_provider not in self.PROVIDER_PRIORITY:
            raise ValueError(
                f"'{type(self).__name__}' cannot run on '{execution_provider}', expected one of "
                f"{', '.join(self.PROVIDER_PRIORITY)}"
            )

        self.execution_provider = execution_provider
        self.provider_list = self.PROVIDER_PRIORITY[self.PROVIDER_PRIORITY.index(execution_provider):]

    def load_config(self, config_file):
        config = self.read_config(config_file)

        if config.get('execution_provider'):
            self.set_execution_provider(config['execution_provider'])

        self.groups = config.get('groups') or []
        self.default_group = int(config.get('default_group', 0))

        self.classes = {
            int(entry['id']): {
                'name': entry['name'],
                'alias': entry.get('alias', entry['name']),
                'color': entry.get('color', 'black'),
                'show': bool(entry.get('show', True)),
            }
            for entry in config.get('classes', [])
        }

        self.build_palette()

    def build_palette(self, id_space=0):
        if not self.classes and not id_space:
            return

        ids = list(self.classes) + [id_space - 1]
        self.palette = np.zeros((max(ids) + 1, 3), dtype=np.uint8)
        self.class_map = {}

        from PIL import ImageColor

        for id, entry in self.classes.items():
            if not entry['show']:
                continue

            self.palette[id] = ImageColor.getrgb(entry['color'])
            self.class_map[id] = entry['alias']

    def match_model_classes(self, id2label):
        """Validate the config classes against the model, or derive them from the model."""
        if self.groups:
            self.build_groups(id2label)
            return

        if not self.classes:
            # No class config: show every class the model knows, with deterministic colors
            colors = np.random.RandomState(0).randint(30, 256, size=(max(id2label) + 1, 3))
            self.palette = colors.astype(np.uint8)
            self.class_map = dict(id2label)
            return

        mismatches = [
            f"id {id} is '{name}' for the model but '{self.classes[id]['name']}' in the config"
            for id, name in id2label.items()
            if id in self.classes and self.classes[id]['name'] != name
        ]
        if mismatches:
            raise ValueError(
                f"'{self.config_file}' does not match the class map of '{self.model}': "
                + ', '.join(mismatches)
            )

        self.build_palette(id_space=max(id2label) + 1)

    def build_groups(self, id2label):
        """Merge model classes (by name) into coarser groups via a class->group matrix."""
        from PIL import ImageColor

        name2id = {name: id for id, name in id2label.items()}
        group_ids = [int(g['id']) for g in self.groups]
        if len(set(group_ids)) != len(group_ids):
            raise ValueError(f"'{self.config_file}' has duplicated group ids")
        if self.default_group not in group_ids:
            raise ValueError(f"default_group {self.default_group} is not a defined group")

        num_classes = max(id2label) + 1
        mapping = np.full(num_classes, self.default_group, dtype=np.int64)
        assigned = {}
        for group in self.groups:
            for name in group.get('classes') or []:
                if name not in name2id:
                    raise ValueError(
                        f"Class '{name}' of group '{group['name']}' is not in the model labels")
                if name in assigned:
                    raise ValueError(
                        f"Class '{name}' is in both '{assigned[name]}' and '{group['name']}'")
                assigned[name] = group['name']
                mapping[name2id[name]] = int(group['id'])

        num_groups = max(group_ids) + 1
        self.group_matrix = np.zeros((num_groups, num_classes), dtype=np.float32)
        self.group_matrix[mapping, np.arange(num_classes)] = 1.0

        self.palette = np.zeros((num_groups, 3), dtype=np.uint8)
        self.class_map = {}
        for group in self.groups:
            self.palette[int(group['id'])] = ImageColor.getrgb(group.get('color', 'black'))
            self.class_map[int(group['id'])] = group['name']

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
        """Return (RGB mask, confidence); confidence is None if the model gives none."""
        model_input = self._process_model_input(raw_image)
        model_output = self._segment_in_model(model_input)

        return self._process_model_output(model_output)


class Bypass(BaseSegmenter):
    def _process_model_input(self, raw_image):
        return raw_image

    def _segment_in_model(self, model_input):
        return model_input

    def _process_model_output(self, model_output):
        return model_output, None


class SegFormer(BaseSegmenter):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Keeping imports lazy to avoid shared dependencies between segmenters
        import cv2
        import onnxruntime as ort

        self.cv2 = cv2

        if not os.path.isfile(self.model):
            raise FileNotFoundError(f"Model file '{self.model}' not found")

        trt_cache = self.trt_cache_dir or os.path.join(os.path.dirname(self.model), 'trt_cache')
        os.makedirs(trt_cache, exist_ok=True)
        trt_options = {
            'trt_fp16_enable': True,
            'trt_engine_cache_enable': True,
            'trt_engine_cache_path': trt_cache,
        }
        providers = [
            (provider, trt_options) if provider == 'TensorrtExecutionProvider' else provider
            for provider in self.provider_list
        ]
        self.session = ort.InferenceSession(self.model, providers=providers)
        self.providers = self.session.get_providers()
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        # The export embeds its own label map and preprocessing
        metadata = self.session.get_modelmeta().custom_metadata_map
        self.input_size = (int(metadata['width']), int(metadata['height']))
        self.image_mean = np.array(json.loads(metadata['image_mean']), dtype=np.float32)
        self.image_mean = self.image_mean.reshape(3, 1, 1)
        self.image_std = np.array(json.loads(metadata['image_std']), dtype=np.float32)
        self.image_std = self.image_std.reshape(3, 1, 1)

        # Metadata is JSON, so its class ids arrive as strings
        self.match_model_classes(
            {int(id): name for id, name in json.loads(metadata['id2label']).items()}
        )

    def _process_model_input(self, raw_image):
        self.latest_image = raw_image
        image = self.cv2.resize(raw_image, self.input_size, interpolation=self.cv2.INTER_LINEAR)
        image = image.astype(np.float32).transpose(2, 0, 1) / 255.0
        image = (image - self.image_mean) / self.image_std

        return np.ascontiguousarray(image[None])

    def _segment_in_model(self, model_input):
        return self.session.run([self.output_name], {self.input_name: model_input})[0]

    def _process_model_output(self, model_output):
        height, width = self.latest_image.shape[:2]

        # Softmax at the (small) model resolution, then upsample the probabilities
        logits = model_output[0]
        logits = logits - logits.max(axis=0, keepdims=True)
        probs = np.exp(logits)
        probs /= probs.sum(axis=0, keepdims=True)

        # Merge class probabilities into groups: P(group) = sum of its classes' P
        if self.group_matrix is not None:
            probs = np.tensordot(self.group_matrix, probs, axes=(1, 0))

        probs = probs.transpose(1, 2, 0)
        probs = self.cv2.resize(probs, (width, height), interpolation=self.cv2.INTER_LINEAR)
        raw_mask = probs.argmax(axis=-1)
        confidence = probs.max(axis=-1).astype(np.float32)

        return self.palette[raw_mask], confidence


# A new BaseSegmenter childclass appears here with the same class name automatically
class SegmenterDict:
    def __init__(self):
        self.model_dict = {
            name: obj
            for name, obj in inspect.getmembers(sys.modules[__name__], inspect.isclass)
            if obj.__module__ == sys.modules[__name__].__name__
            and issubclass(obj, BaseSegmenter)
            and obj is not BaseSegmenter
        }

    def get_model(self, model):
        return self.model_dict.get(model, Bypass)
