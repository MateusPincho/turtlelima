"""Lifecycle node running SegFormer semantic segmentation on live camera images."""

import os
import warnings

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node, TransitionCallbackReturn
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import LabelInfo, VisionClass

from turtlelima_segmentation.segmenters import BaseSegmenter, SegFormer

PACKAGE = 'turtlelima_segmentation'

# variant -> (file stem, input resolution tag)
MODEL_VARIANTS = {
    'b0': 'segformer-b0-finetuned-ade-512-512',
    'b2': 'segformer-b2-finetuned-ade-512-512',
    'b5': 'segformer-b5-finetuned-ade-640-640',
}


class SemanticSegmenter(Node):
    def __init__(self):
        super().__init__('semantic_segmentation')

        self.declare_parameter('model_variant', 'b2')
        self.declare_parameter('use_tensorrt', True)
        self.declare_parameter('execution_provider', '')
        self.declare_parameter('model_dir', '')
        self.declare_parameter('trt_cache_dir', os.path.expanduser(
            '~/.cache/turtlelima_segmentation/trt_cache'))
        self.declare_parameter('segmenter_config_file', 'default')
        self.declare_parameter('frame_id', 'camera')

        self.segmenter = None
        self.bridge = CvBridge()
        self.get_logger().info('Segmentation node initialized (unconfigured)')

    def resolve_model_path(self, variant, use_tensorrt, model_dir):
        stem = MODEL_VARIANTS[variant]
        candidates = [f'{stem}-trt.onnx'] if use_tensorrt else []
        candidates.append(f'{stem}.onnx')

        for i, name in enumerate(candidates):
            path = os.path.join(model_dir, name)
            if os.path.isfile(path):
                if use_tensorrt and i > 0:
                    self.get_logger().warn(
                        f"No TensorRT export of '{variant}' found, using '{name}' instead")
                return path

        return None

    def on_configure(self, state):
        variant = self.get_parameter('model_variant').value
        use_tensorrt = self.get_parameter('use_tensorrt').value
        model_dir = self.get_parameter('model_dir').value
        config_file = self.get_parameter('segmenter_config_file').value
        self.frame_id = self.get_parameter('frame_id').value

        if variant not in MODEL_VARIANTS:
            self.get_logger().error(
                f"Unknown model_variant '{variant}', expected one of {list(MODEL_VARIANTS)}")
            return TransitionCallbackReturn.FAILURE

        if not model_dir:
            model_dir = os.path.join(get_package_share_directory(PACKAGE), 'models')

        model_path = self.resolve_model_path(variant, use_tensorrt, model_dir)
        if model_path is None:
            self.get_logger().error(f"No model for '{variant}' found in '{model_dir}'")
            return TransitionCallbackReturn.FAILURE

        if config_file == 'default':
            config_file = os.path.join(
                get_package_share_directory(PACKAGE), 'config', 'classes.yaml')

        if config_file:
            try:
                BaseSegmenter.read_config(config_file)
            except Exception as e:
                self.get_logger().error(f"Failed to read class config '{config_file}': {e}")
                return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f"Loading '{os.path.basename(model_path)}'...")
        try:
            with warnings.catch_warnings():
                warnings.simplefilter('ignore')
                self.segmenter = SegFormer(
                    model=model_path,
                    config_file=config_file,
                    execution_provider=self.get_parameter('execution_provider').value,
                    trt_cache_dir=self.get_parameter('trt_cache_dir').value,
                )
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.segmenter = None
            return TransitionCallbackReturn.FAILURE

        providers = self.segmenter.providers
        self.get_logger().info(f'Inference running on {providers[0]}')
        if providers[0] != self.segmenter.execution_provider:
            self.get_logger().warn(
                f"'{self.segmenter.execution_provider}' unavailable, fell back to '{providers[0]}'")
        if providers[0] == 'CPUExecutionProvider':
            self.get_logger().warn('GPU acceleration unavailable!')

        image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        latched_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.mask_publisher = self.create_lifecycle_publisher(Image, 'mask', 10)
        self.confidence_publisher = self.create_lifecycle_publisher(Image, 'confidence', 10)
        self.labels_publisher = self.create_publisher(LabelInfo, 'label_info', latched_qos)
        self.palette_publisher = self.create_publisher(Image, 'class_palette', latched_qos)
        self.image_subscriber = self.create_subscription(
            Image, 'image_input', self.image_callback, image_qos)

        self.publish_label_info()
        self.publish_class_palette()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating node...')
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating node...')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up...')
        self.segmenter = None
        self.destroy_subscription(self.image_subscriber)
        self.destroy_publisher(self.mask_publisher)
        self.destroy_publisher(self.confidence_publisher)
        self.destroy_publisher(self.labels_publisher)
        self.destroy_publisher(self.palette_publisher)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        return TransitionCallbackReturn.SUCCESS

    def publish_label_info(self):
        msg = LabelInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for id, name in self.segmenter.class_map.items():
            label = VisionClass()
            label.class_id = id
            label.class_name = name
            msg.class_map.append(label)
        msg.threshold = 0.0

        self.labels_publisher.publish(msg)
        self.get_logger().info(f'Published LabelInfo with {len(msg.class_map)} classes')

    def publish_class_palette(self):
        if not len(self.segmenter.palette):
            self.get_logger().info('Segmenter colorizes no class, publishing no palette')
            return

        palette = self.segmenter.palette.reshape(1, -1, 3)
        msg = self.bridge.cv2_to_imgmsg(palette, encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.palette_publisher.publish(msg)
        self.get_logger().info('Published class palette')

    def process_image(self, img_msg):
        img_rgb = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        start = self.get_clock().now()

        mask_rgb, confidence = self.segmenter.run_segmentation(img_rgb)

        elapsed = self.get_clock().now() - start
        self.get_logger().debug(f'Inference time: {elapsed.nanoseconds * 1e-6:.3f} ms')

        mask_msg = self.bridge.cv2_to_imgmsg(mask_rgb, encoding='rgb8', header=img_msg.header)
        confidence_msg = None
        if confidence is not None:
            confidence_msg = self.bridge.cv2_to_imgmsg(
                confidence, encoding='32FC1', header=img_msg.header)

        return mask_msg, confidence_msg

    def image_callback(self, img_msg):
        if not self.mask_publisher.is_activated or self.segmenter is None:
            return

        try:
            mask_msg, confidence_msg = self.process_image(img_msg)
        except Exception as e:
            self.get_logger().error(f'Could not process frame: {e}')
            return

        self.mask_publisher.publish(mask_msg)
        if confidence_msg is not None:
            self.confidence_publisher.publish(confidence_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSegmenter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
