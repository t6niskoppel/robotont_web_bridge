"""Select camera topics from publishers actually present in a robot namespace."""


def camera_topics(has):
    rgb = next((topic for topic in ('/camera/color/image_raw', '/camera/image_raw')
                if has(topic) or has(topic + '/compressed')), None)
    depth = next((topic for topic in ('/camera/aligned_depth_to_color/image_raw',
                                     '/camera/depth/image_rect_raw', '/camera/depth/image_raw')
                  if has(topic)), None)
    topics = {}
    if rgb:
        topics['rgb'] = rgb
    if depth:
        topics['depth'] = depth
    if has('/camera/depth/points'):
        topics['points'] = '/camera/depth/points'
    transport = 'compressed' if rgb and has(rgb + '/compressed') else 'raw'
    generation = '3' if rgb == '/camera/color/image_raw' else '3lite' if rgb else 'unknown'
    return topics, transport, generation
