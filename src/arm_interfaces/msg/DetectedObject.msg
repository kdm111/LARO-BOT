string object_id # 물체 식별자 Pick/Place goal의 object_id와 매칭
geometry_msgs/PoseStamped pose # 물체 포즈. header.frame_id = base_link
builtin_interfaces/Time last_seen # 마지막으로 관측된 시간. 오래되면 신뢰 하지 않음