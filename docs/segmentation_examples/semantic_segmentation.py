import os
import rclpy
import warnings
import threading  # TODO(lorenzo): change to one-shot timer
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import VisionClass
from vision_msgs.msg import LabelInfo
from rclpy.lifecycle import Node, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from cv_bridge import CvBridge
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

from image_processing.segmenters import BaseSegmenter, Bypass, SegmenterDict

CONFIG_PACKAGE = "image_processing"
MODEL_DIR = "models"


class SemanticSegmenter(Node):
    def __init__(self):
        super().__init__("semantic_segmentation")

        self.declare_parameter("input_uri", "")
        self.declare_parameter("segmenter_config_file", "")

        self.segmenter = None
        self.bridge = CvBridge()
        self.offline_process_thread = None
        self.cancel_event = threading.Event()  # TODO(lorenzo): change to one-shot timer
        self.get_logger().info("Lifecycle Segmenter Initialized (Unconfigured)")

    def on_configure(self, state):
        self.input_uri = self.get_parameter("input_uri").get_parameter_value().string_value
        self.segmenter_config_file = (
            self.get_parameter("segmenter_config_file").get_parameter_value().string_value
        )
        self.reader, self.writer = None, None
        if self.input_uri:
            self.output_uri = self.input_uri.rstrip("/") + "_seg"
            self.get_logger().info("Creating bag I/O...")
            try:
                self.reader = rosbag2_py.SequentialReader()
                self.reader.open(
                    rosbag2_py.StorageOptions(uri=self.input_uri, storage_id="mcap"),
                    rosbag2_py.ConverterOptions(
                        input_serialization_format="cdr", output_serialization_format="cdr"
                    ),
                )

                self.writer = rosbag2_py.SequentialWriter()
                self.writer.open(
                    rosbag2_py.StorageOptions(uri=self.output_uri, storage_id="mcap"),
                    rosbag2_py.ConverterOptions(
                        input_serialization_format="cdr", output_serialization_format="cdr"
                    ),
                )

                topic_info = rosbag2_py.TopicMetadata(
                    id=0, name="/mask", type="sensor_msgs/msg/Image", serialization_format="cdr"
                )
                self.writer.create_topic(topic_info)

                label_info_topic = rosbag2_py.TopicMetadata(
                    id=1,
                    name="/label_info",
                    type="vision_msgs/msg/LabelInfo",
                    serialization_format="cdr",
                )
                self.writer.create_topic(label_info_topic)

                class_palette_topic = rosbag2_py.TopicMetadata(
                    id=2,
                    name="/class_palette",
                    type="sensor_msgs/msg/Image",
                    serialization_format="cdr",
                )
                self.writer.create_topic(class_palette_topic)

                self.get_logger().info("Bag I/O created!")

            except Exception as e:
                self.get_logger().info(f"Failed to create bag I/O: {e}")

        if self.segmenter_config_file:
            self.get_logger().info(f"Loading class config '{self.segmenter_config_file}'")
            try:
                config = BaseSegmenter.read_config(self.segmenter_config_file)

            except Exception as e:
                self.get_logger().error(
                    f"Failed to read '{self.segmenter_config_file}' class config: {e}"
                )
                return TransitionCallbackReturn.FAILURE

            self.architecture = config.get("architecture", "")

        else:
            self.architecture = Bypass.__name__
            self.get_logger().warn("No class config given, bypassing images")

        self.get_logger().info("Loading AI Model and I/O...")
        self.segmenter_class = SegmenterDict().get_model(self.architecture)

        if self.segmenter_class == Bypass and self.architecture != "Bypass":
            self.get_logger().warn(
                f"Could not find '{self.architecture}' segmenter implementation, bypassing images"
            )

        else:
            self.get_logger().info(f"Using '{self.architecture}' segmenter implementation")

        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                self.segmenter = self.segmenter_class(
                    config_file=self.segmenter_config_file,
                    model_dir=os.path.join(get_package_share_directory(CONFIG_PACKAGE), MODEL_DIR),
                )

        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            return TransitionCallbackReturn.FAILURE

        providers = getattr(self.segmenter, "providers", None)
        if providers:
            execution_provider = self.segmenter.execution_provider
            self.get_logger().info(f"Inference running on {providers[0]}")
            if providers[0] != execution_provider:
                self.get_logger().warn(
                    f"'{execution_provider}' unavailable, fell back to '{providers[0]}'"
                )
            if providers[0] == "CPUExecutionProvider":
                self.get_logger().warn("GPU acceleration unavailable!")

        self.mask_publisher = self.create_lifecycle_publisher(Image, "mask", 10)
        self.image_subscriber = self.create_subscription(
            Image, "image_input", self.image_callback, 1
        )
        qos_latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.labels_publisher = self.create_publisher(LabelInfo, "label_info", qos_latched)
        self.palette_publisher = self.create_publisher(Image, "class_palette", qos_latched)
        self.publish_label_info()
        self.publish_class_palette()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("Activating node...")

        if not self.input_uri:
            self.get_logger().info("Online mode operation started")
            return super().on_activate(state)

        self.get_logger().info("Offline mode operation started")
        self.cancel_event.clear()
        self.offline_process_thread = threading.Thread(target=self.process_bag)
        self.offline_process_thread.start()

        return super().on_activate(state)

    def process_bag(self):
        type_map = {topic.name: topic.type for topic in self.reader.get_all_topics_and_types()}
        while self.reader.has_next() and not self.cancel_event.is_set():
            topic, data, stamp = self.reader.read_next()

            try:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)

            except Exception as e:
                self.get_logger().error(f"Could not desserialize message from {topic}: {e}")
                continue

            if topic == self.image_subscriber.topic_name:
                try:
                    mask_msg = self.process_image(msg)
                    if mask_msg is None:
                        continue

                    self.writer.write("/mask", serialize_message(mask_msg), stamp)

                    self.get_logger().debug(f"Processed frame at {stamp}")

                except Exception as e:
                    self.get_logger().error(f"Could not process frame: {e}")

        self.executor.create_task(self.trigger_self_deactivation)

    def trigger_self_deactivation(self):
        client = self.create_client(ChangeState, f"/{self.get_name()}/change_state")

        if client.wait_for_service(timeout_sec=2.0):
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_DEACTIVATE
            client.call_async(req)
        else:
            self.get_logger().error("Could not find own change_state service!")

    def on_deactivate(self, state):
        self.get_logger().info("Deactivating node...")

        self.cancel_event.set()
        if self.offline_process_thread and self.offline_process_thread.is_alive():
            self.offline_process_thread.join()

        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info("Cleaning up...")
        self.reader = None
        self.writer = None
        self.segmenter = None
        self.destroy_subscription(self.image_subscriber)
        self.destroy_publisher(self.mask_publisher)
        self.destroy_publisher(self.labels_publisher)
        self.destroy_publisher(self.palette_publisher)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        return TransitionCallbackReturn.SUCCESS

    def publish_label_info(self):
        msg = LabelInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        for id, name in self.segmenter.class_map.items():
            label = VisionClass()
            label.class_id = id
            label.class_name = name
            msg.class_map.append(label)
        msg.threshold = 1.0

        self.labels_publisher.publish(msg)
        self.get_logger().info("Published LabelInfo")

        if self.writer:
            stamp = self.get_clock().now().nanoseconds
            self.writer.write("/label_info", serialize_message(msg), stamp)

    def publish_class_palette(self):
        if not len(self.segmenter.palette):
            self.get_logger().info("Segmenter colorizes no class, publishing no palette")
            return

        palette = self.segmenter.palette.reshape(1, -1, 3)
        msg = self.bridge.cv2_to_imgmsg(palette, encoding="rgb8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        self.palette_publisher.publish(msg)
        self.get_logger().info("Published class palette")

        if self.writer:
            stamp = self.get_clock().now().nanoseconds
            self.writer.write("/class_palette", serialize_message(msg), stamp)

    def process_image(self, img_msg):
        img_bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        inference_start_time = self.get_clock().now()

        img_rgb = img_bgr[:, :, ::-1]
        mask_rgb = self.segmenter.run_segmentation(img_rgb)
        mask_bgr = mask_rgb[:, :, ::-1]

        inference_end_time = self.get_clock().now()
        inference_time = inference_end_time - inference_start_time
        self.get_logger().debug(f"Inference Time: {inference_time.nanoseconds * 1e-6:.3f} ms")

        mask_msg = self.bridge.cv2_to_imgmsg(mask_bgr, header=img_msg.header, encoding="bgr8")

        return mask_msg

    def image_callback(self, img_msg):
        mask_msg = self.process_image(img_msg)

        if mask_msg is not None and self.mask_publisher.is_activated:
            self.mask_publisher.publish(mask_msg)


def main(args=None):
    rclpy.init(args=args)
    segmenter = SemanticSegmenter()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(segmenter)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            segmenter.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
