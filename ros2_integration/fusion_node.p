class MultiModalFusionNode(Node):
def __init__(self):
super().__init__(’fusion_node’)
# Subscribers
self.vision_sub = self.create_subscription(
Image, ’/camera/color/image_raw’,
self.vision_callback, 10)
self.ft_sub = self.create_subscription(
WrenchStamped, ’/ft_sensor/wrench’,
self.ft_callback, 10)
self.tactile_sub = self.create_subscription(
TactileData, ’/tactile/data’,
self.tactile_callback, 10)
# Publisher
self.fusion_pub = self.create_publisher(
MultiModalState, ’/fusion/state’, 10)
# Synchronization
self.sync = ApproximateTimeSynchronizer(
[self.vision_sub, self.ft_sub, self.tactile_sub],
queue_size=10, slop=0.1)
self.sync.registerCallback(self.fusion_callback)
def fusion_callback(self, vision_msg, ft_msg,
tactile_msg):
# Extract features
visual_features = self.process_vision(vision_msg)
wrench = ft_msg.wrench
pressure = tactile_msg.pressure_values
# Multi-modal fusion
state = self.fuse_modalities(
visual_features, wrench, pressure)
# Publish fused state
self.fusion_pub.publish(state)
