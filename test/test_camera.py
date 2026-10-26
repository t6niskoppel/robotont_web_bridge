import unittest

from robotont_web_bridge.camera import camera_topics


class CameraTopicsTests(unittest.TestCase):
    def test_gazebo_raw_camera_does_not_require_compressed_publisher(self):
        topics, transport, generation = camera_topics({'/camera/image_raw'}.__contains__)
        self.assertEqual(topics, {'rgb': '/camera/image_raw'})
        self.assertEqual((transport, generation), ('raw', '3lite'))

    def test_lite_prefers_advertised_compressed_transport(self):
        topics, transport, generation = camera_topics({
            '/camera/image_raw', '/camera/image_raw/compressed',
        }.__contains__)
        self.assertEqual(topics['rgb'], '/camera/image_raw')
        self.assertEqual(transport, 'compressed')

    def test_compressed_only_publisher_is_available(self):
        topics, transport, generation = camera_topics({'/camera/image_raw/compressed'}.__contains__)
        self.assertEqual(topics['rgb'], '/camera/image_raw')
        self.assertEqual(transport, 'compressed')

    def test_realsense_uses_existing_aligned_or_rectified_depth(self):
        publishers = {'/camera/color/image_raw', '/camera/depth/image_rect_raw',
                      '/camera/aligned_depth_to_color/image_raw'}
        topics, transport, generation = camera_topics(publishers.__contains__)
        self.assertEqual(generation, '3')
        self.assertEqual(topics['depth'], '/camera/aligned_depth_to_color/image_raw')
        publishers.remove('/camera/aligned_depth_to_color/image_raw')
        self.assertEqual(camera_topics(publishers.__contains__)[0]['depth'],
                         '/camera/depth/image_rect_raw')

    def test_no_camera_does_not_invent_topics(self):
        self.assertEqual(camera_topics(lambda topic: False), ({}, 'raw', 'unknown'))


if __name__ == '__main__':
    unittest.main()
