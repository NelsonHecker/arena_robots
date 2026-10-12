"""Field layout of the audio messages in arena_robots_msgs."""


def test_audio_frame_msg():
    from arena_robots_msgs.msg import AudioFrame
    from geometry_msgs.msg import Point

    msg = AudioFrame()
    msg.header.frame_id = "jackal/base_link"
    msg.sample_rate = 16000
    msg.channel_count = 4
    msg.frame_count = 2
    msg.encoding = "32FC1"
    msg.interleaved = True
    msg.channel_names = ["front_left", "front_right", "rear_left", "rear_right"]
    msg.frame_ids = ["jackal/mic_front_left", "jackal/mic_front_right", "jackal/mic_rear_left", "jackal/mic_rear_right"]
    msg.microphone_positions = [Point(x=0.19, y=0.135, z=0.22)] * 4
    msg.microphone_yaw_rad = [0.785398, -0.785398, 2.356194, -2.356194]
    msg.data = [0.0] * 8

    assert msg.sample_rate == 16000
    assert msg.channel_count == 4
    assert msg.frame_count == 2
    assert len(msg.data) == msg.channel_count * msg.frame_count


def test_sound_detection_msg():
    from arena_robots_msgs.msg import SoundDetection

    msg = SoundDetection()
    msg.header.frame_id = "env_0/jackal/base_link"
    msg.robot = "jackal"
    msg.frontend = "bus"
    msg.kind = "footstep"
    msg.event_id = "ped_3:17"
    msg.azimuth_rad = 1.25
    msg.elevation_rad = float("nan")
    msg.level_db = 54.0
    msg.confidence = 0.75

    assert msg.robot == "jackal"
    assert msg.frontend == "bus"
    assert msg.kind == "footstep"
    assert msg.azimuth_rad == 1.25
    assert msg.confidence == 0.75
    assert "listener_id" not in SoundDetection.get_fields_and_field_types()
