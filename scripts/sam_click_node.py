#!/usr/bin/env python3
"""
SAM Click Node
- OpenCV 창에서 마우스 클릭 → SAM 마스크 생성
- 마스크 depth 최솟값(10th percentile) → 물체 윗면 추정
- approach_point, grasp_point 를 각각 발행
"""

import rospy
import cv2
import numpy as np
import torch
from segment_anything import sam_model_registry, SamPredictor
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

# ── 설정 ──────────────────────────────────────────────────────────────
SAM_CHECKPOINT  = "/home/gyu/sam_weights/sam_vit_b_01ec64.pth"
SAM_MODEL_TYPE  = "vit_b"
DEVICE          = "cuda" if torch.cuda.is_available() else "cpu"

APPROACH_OFFSET = 0.20   # 물체 윗면에서 카메라 방향으로 20cm 위 (approach)
GRASP_MARGIN    = 0.01   # 물체 윗면 1cm 위 (grasp, 충돌 방지 마진)

RGB_TOPIC   = "/mujoco_ros/mujoco_ros_interface/camera/rgb/image_raw"
DEPTH_TOPIC = "/mujoco_ros/mujoco_ros_interface/camera/depth/image_raw"
INFO_TOPIC  = "/mujoco_ros/mujoco_ros_interface/camera/camera_info"
# ─────────────────────────────────────────────────────────────────────

bridge      = CvBridge()
rgb_image   = None
depth_image = None
camera_info = None
click_point = None
predictor   = None


def rgb_callback(msg):
    global rgb_image
    rgb_image = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")


def depth_callback(msg):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")


def info_callback(msg):
    global camera_info
    camera_info = msg


def mouse_callback(event, x, y, flags, param):
    global click_point
    if event == cv2.EVENT_LBUTTONDOWN:
        click_point = (x, y)
        rospy.loginfo(f"Clicked: ({x}, {y})")


def pixel_to_3d(u, v, depth_m, camera_info):
    """픽셀 좌표 + depth → 카메라 좌표계 3D 점
    MuJoCo 카메라 규약 (OpenGL): -Z 방향을 봄
      X = right (+u),  Y = up (-v),  Z = -depth
    """
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]
    X =  (u - cx) * depth_m / fx
    Y = -(v - cy) * depth_m / fy
    Z = -depth_m
    return X, Y, Z


def run_sam(image_rgb, click_xy):
    predictor.set_image(image_rgb)
    masks, scores, _ = predictor.predict(
        point_coords=np.array([click_xy]),
        point_labels=np.array([1]),
        multimask_output=True,
    )
    return masks[np.argmax(scores)]


def make_point_msg(X, Y, Z):
    msg = PointStamped()
    msg.header.stamp    = rospy.Time.now()
    msg.header.frame_id = "wrist_cam"
    msg.point.x = X
    msg.point.y = Y
    msg.point.z = Z
    return msg


def main():
    global predictor

    rospy.init_node("sam_click_node")

    rospy.loginfo(f"Loading SAM ({SAM_MODEL_TYPE}) on {DEVICE}...")
    sam = sam_model_registry[SAM_MODEL_TYPE](checkpoint=SAM_CHECKPOINT)
    sam.to(device=DEVICE)
    predictor = SamPredictor(sam)
    rospy.loginfo("SAM ready. Click on an object.")

    rospy.Subscriber(RGB_TOPIC,   Image,      rgb_callback)
    rospy.Subscriber(DEPTH_TOPIC, Image,      depth_callback)
    rospy.Subscriber(INFO_TOPIC,  CameraInfo, info_callback)

    approach_pub = rospy.Publisher("/sam/approach_point", PointStamped, queue_size=1)
    grasp_pub    = rospy.Publisher("/sam/grasp_point",    PointStamped, queue_size=1)

    cv2.namedWindow("SAM Click", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("SAM Click", mouse_callback)

    overlay = None
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        global click_point

        if rgb_image is None:
            rate.sleep()
            continue

        display = cv2.cvtColor(rgb_image.copy(), cv2.COLOR_RGB2BGR)

        if click_point is not None and depth_image is not None and camera_info is not None:
            u, v = click_point
            click_point = None

            rospy.loginfo("Running SAM...")
            mask = run_sam(rgb_image, (u, v))

            overlay = np.zeros_like(display)
            overlay[mask] = [0, 255, 0]

            ys, xs = np.where(mask)
            if len(xs) == 0:
                rospy.logwarn("Empty mask")
            else:
                # 마스크 중심 (XY 기준점)
                cx_mask = int(xs.mean())
                cy_mask = int(ys.mean())

                # 마스크 내 depth 값 수집 → 물체 윗면 추정
                depths = depth_image[mask].flatten()
                valid  = depths[(depths > 0.05) & (depths < 2.0)]

                if len(valid) == 0:
                    rospy.logwarn("No valid depth in mask")
                else:
                    # 10th percentile = 물체 가장 높은 면(카메라에 가까운 쪽)
                    top_depth = float(np.percentile(valid, 10))
                    rospy.loginfo(f"Object top surface depth: {top_depth:.3f} m")

                    # approach point: 물체 윗면에서 APPROACH_OFFSET 만큼 위
                    approach_depth = top_depth - APPROACH_OFFSET
                    if approach_depth < 0.02:
                        rospy.logwarn("Approach depth too small, skipping")
                    else:
                        aX, aY, aZ = pixel_to_3d(cx_mask, cy_mask, approach_depth, camera_info)
                        gX, gY, gZ = pixel_to_3d(cx_mask, cy_mask,
                                                   top_depth - GRASP_MARGIN, camera_info)

                        rospy.loginfo(f"Approach (cam): X={aX:.3f} Y={aY:.3f} Z={aZ:.3f}")
                        rospy.loginfo(f"Grasp    (cam): X={gX:.3f} Y={gY:.3f} Z={gZ:.3f}")

                        approach_pub.publish(make_point_msg(aX, aY, aZ))
                        grasp_pub.publish(make_point_msg(gX, gY, gZ))

        if overlay is not None:
            display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)

        cv2.imshow("SAM Click", display)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:
            break
        if key == ord('r'):
            overlay = None

        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
