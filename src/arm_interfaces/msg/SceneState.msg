# 현재 씬의 스냅삿 - 인지가 본 물체들의 목록
# M3 카메라가 발행할 것
# 이미 받은 스냅샷은 안 바뀐다. 
std_msgs/Header header # stamp = 스냅샷 시각, frame_id = base_link
DetectedObject[] objects # 검출된 물체들 (없으면 빈 배열)