#!/usr/bin/env python3
"""
EduBot Lane Follower  –  v4
============================

Tuned from real camera image (gym floor, window reflections, polished concrete).

What I can see in your image
-----------------------------
  - White solid tape  : thin, runs diagonally right-side (perspective), RIGHT boundary
  - Yellow dashed tape: vivid yellow, clearly visible, LEFT boundary
  - Window reflections: large bright diagonal streaks on the LEFT of the floor
  - Floor is polished concrete → very specular, reflects window light strongly
  - All lane content is in the BOTTOM ~45% of the frame
  - The robot body is visible bottom-right → camera offset is significant

Key fixes in v4
---------------

1.  ROI raised to 0.55 (bottom 45% only) — cuts out ALL wall/window content.
    Reflections that reach into the floor are handled by shape filtering below.

2.  SHAPE FILTER for white contours (kills reflections):
    Window glare streaks are TALL and THIN (high, diagonal blobs).
    Real white tape seen from a low camera is WIDE and LOW (low aspect ratio
    in the bounding rect, roughly horizontal).
    Filter: bounding-rect width > height  (tape is wider than tall in ROI)
            aspect_ratio (w/h) > 1.2
            solidity (area/convexHull) > 0.35  (tape is solid, glare is wispy)
    These three together eliminate the diagonal window reflection blobs.

3.  FLOOR-ZONE MASK: the ROI is further split into a NEAR zone (bottom 60%
    of ROI) and a FAR zone (top 40% of ROI).  White detections are only
    trusted in the NEAR zone unless the shape filter passes cleanly in FAR.
    This stops distant wall reflections from ever counting.

4.  TEMPORAL CONFIDENCE: white line must appear in 2 consecutive frames
    before it steers.  Reflections flash for 1 frame; real tape persists.

5.  PERSPECTIVE WARP (look-ahead):
    A bird's-eye warp stretches the top of the ROI so the robot effectively
    sees further down the lane.  Src/dst points are tuned for a low-mounted
    forward camera on a small indoor robot.  All tunable via params so you
    can adjust without restarting.

6.  Yellow HSV tightened for vivid indoor yellow tape:
    H 18-38, S 120-255, V 120-255  (brighter than before, matches your tape).

7.  camera_offset_px default raised to 40 px — your robot looks wide relative
    to the lane in the image.

All parameters remain live-tunable via ros2 param set.

Quick-start tuning
------------------
  # If reflections still get through, raise these:
  ros2 param set /lane_follower white_aspect_min 1.5
  ros2 param set /lane_follower white_solidity_min 0.45

  # If white tape is missed (it's quite thin in your image):
  ros2 param set /lane_follower min_area 200
  ros2 param set /lane_follower white_aspect_min 1.0

  # Body offset (increase if robot still clips white line):
  ros2 param set /lane_follower camera_offset_px 50

  # Perspective warp strength (0.0 = off, 1.0 = full warp):
  ros2 param set /lane_follower warp_strength 0.6
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import cv2
import numpy as np

CAMERA_TOPIC  = '/camera_1/image_raw'
CMD_VEL_TOPIC = '/cmd_vel'
DEBUG_TOPIC   = '/lane_debug/image_raw'


class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_follower')
        self.bridge = CvBridge()

        # Temporal confidence state
        self._white_confirm_count  = 0   # frames white has been seen continuously
        self._white_confirmed      = False
        self._prev_white_cx        = None
        self._prev_yellow_cx       = None

        def p(desc):
            return ParameterDescriptor(description=desc)

        # ── White HSV ─────────────────────────────────────────────────────────
        # Your white tape on polished concrete: high V, low-mid S
        # Keep s_max at 55 — raising it risks letting in the yellow floor glare
        self.declare_parameter('white_h_min',  0,   p('White hue min'))
        self.declare_parameter('white_h_max',  180, p('White hue max'))
        self.declare_parameter('white_s_min',  0,   p('White sat min (0 = pure white, raise kills glare)'))
        self.declare_parameter('white_s_max',  55,  p('White sat max'))
        self.declare_parameter('white_v_min',  200, p('White val min (raise to reject dim reflections)'))
        self.declare_parameter('white_v_max',  255, p('White val max'))

        # ── Yellow HSV ────────────────────────────────────────────────────────
        # Your yellow tape is vivid/bright — tighter range is fine and avoids
        # orange floor markings or warm lighting
        self.declare_parameter('yellow_h_min', 18,  p('Yellow hue min'))
        self.declare_parameter('yellow_h_max', 38,  p('Yellow hue max'))
        self.declare_parameter('yellow_s_min', 120, p('Yellow sat min'))
        self.declare_parameter('yellow_s_max', 255, p('Yellow sat max'))
        self.declare_parameter('yellow_v_min', 120, p('Yellow val min'))
        self.declare_parameter('yellow_v_max', 255, p('Yellow val max'))

        # ── ROI ───────────────────────────────────────────────────────────────
        # From your image: floor starts at ~55% down. Set to 0.55 to exclude
        # all walls, windows, flags, signs from processing entirely.
        self.declare_parameter('roi_top_frac', 0.55,
            p('Top of ROI as fraction of frame height. 0.55 cuts above the floor horizon.'))

        # ── Shape filters (reflection killer) ─────────────────────────────────
        # Window reflections: tall diagonal blobs  (height > width, aspect < 1)
        # Real white tape  : wide flat blobs       (width > height, aspect > 1.2)
        self.declare_parameter('white_aspect_min', 1.2,
            p('Min bounding-rect aspect ratio (w/h) for white contours. Kills tall reflection streaks.'))
        self.declare_parameter('white_solidity_min', 0.40,
            p('Min solidity (area/convexHull) for white. Glare is wispy (<0.3), tape is solid.'))
        self.declare_parameter('white_confirm_frames', 2,
            p('White must appear for this many consecutive frames before steering. Kills flash reflections.'))

        # ── Size filters ──────────────────────────────────────────────────────
        self.declare_parameter('min_area',   200, p('Min contour area px2'))
        self.declare_parameter('min_width',  10,  p('Min bounding-box width px'))

        # ── Perspective warp (look-ahead) ──────────────────────────────────────
        # 0.0 = disabled, 1.0 = full warp. Start at 0.5 and raise if needed.
        # The warp stretches the top of the ROI to simulate a higher viewpoint,
        # making far-away tape visible and easier to track.
        self.declare_parameter('warp_enabled', True, p('Enable bird-eye perspective warp'))
        self.declare_parameter('warp_strength', 0.45,
            p('Warp strength 0.0-1.0. Higher = more perspective correction = sees further.'))

        # ── Robot body offset ─────────────────────────────────────────────────
        # Your robot looks wide in the image. Default 40px shifts target left.
        self.declare_parameter('camera_offset_px', 40,
            p('Pixels to shift steering target LEFT for robot body width. Increase if robot clips white line.'))

        # ── Lane geometry ──────────────────────────────────────────────────────
        self.declare_parameter('white_only_gap_frac', 0.20,
            p('When only white visible: target this fraction of ROI width to its LEFT'))
        self.declare_parameter('yellow_only_gap_frac', 0.20,
            p('When only yellow visible: target this fraction of ROI width to its RIGHT'))

        # ── Driving ───────────────────────────────────────────────────────────
        self.declare_parameter('linear_speed',  0.12,  p('Normal speed m/s'))
        self.declare_parameter('turn_speed',     0.07,  p('Speed during turns m/s'))
        self.declare_parameter('kp',             0.004, p('Proportional steering gain'))
        self.declare_parameter('turn_kp_scale',  1.8,   p('kp multiplier during turns'))
        self.declare_parameter('max_angular',    0.65,  p('Max angular velocity rad/s'))

        # ── Turn detection ─────────────────────────────────────────────────────
        self.declare_parameter('turn_white_frac',  0.50,
            p('White cx below this x-fraction = right-turn'))
        self.declare_parameter('turn_yellow_frac', 0.55,
            p('Yellow cx above this x-fraction = left-turn'))

        # ── Jump rejection ─────────────────────────────────────────────────────
        self.declare_parameter('max_cx_jump_px', 100,
            p('Max white cx change per frame before discarding as reflection flash'))

        self.add_on_set_parameters_callback(self.param_callback)
        self.load_params()

        self.sub = self.create_subscription(Image, CAMERA_TOPIC, self.image_cb, 1)
        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, DEBUG_TOPIC, 1)

        self.get_logger().info(
            'EduBot lane follower v4 ready\n'
            f'  Camera : {CAMERA_TOPIC}\n'
            f'  Debug  : {DEBUG_TOPIC}\n'
            '  Tune   : ros2 param set /lane_follower <param> <value>'
        )

    # ── Parameters ────────────────────────────────────────────────────────────

    def load_params(self):
        gp = self.get_parameter
        self.white_lower  = np.array([gp('white_h_min').value, gp('white_s_min').value,
                                       gp('white_v_min').value], dtype=np.uint8)
        self.white_upper  = np.array([gp('white_h_max').value, gp('white_s_max').value,
                                       gp('white_v_max').value], dtype=np.uint8)
        self.yellow_lower = np.array([gp('yellow_h_min').value, gp('yellow_s_min').value,
                                       gp('yellow_v_min').value], dtype=np.uint8)
        self.yellow_upper = np.array([gp('yellow_h_max').value, gp('yellow_s_max').value,
                                       gp('yellow_v_max').value], dtype=np.uint8)

        self.roi_top_frac          = float(gp('roi_top_frac').value)
        self.white_aspect_min      = float(gp('white_aspect_min').value)
        self.white_solidity_min    = float(gp('white_solidity_min').value)
        self.white_confirm_frames  = int(gp('white_confirm_frames').value)
        self.min_area              = int(gp('min_area').value)
        self.min_width             = int(gp('min_width').value)
        self.warp_enabled          = bool(gp('warp_enabled').value)
        self.warp_strength         = float(gp('warp_strength').value)
        self.camera_offset_px      = int(gp('camera_offset_px').value)
        self.white_only_gap_frac   = float(gp('white_only_gap_frac').value)
        self.yellow_only_gap_frac  = float(gp('yellow_only_gap_frac').value)
        self.linear_speed          = float(gp('linear_speed').value)
        self.turn_speed            = float(gp('turn_speed').value)
        self.kp                    = float(gp('kp').value)
        self.turn_kp_scale         = float(gp('turn_kp_scale').value)
        self.max_angular           = float(gp('max_angular').value)
        self.turn_white_frac       = float(gp('turn_white_frac').value)
        self.turn_yellow_frac      = float(gp('turn_yellow_frac').value)
        self.max_cx_jump_px        = int(gp('max_cx_jump_px').value)

    def param_callback(self, params):
        self.load_params()
        return SetParametersResult(successful=True)

    # ── Perspective warp ───────────────────────────────────────────────────────

    def _warp_birdseye(self, roi):
        """
        Warp the ROI to a bird's-eye view so the robot sees further ahead.

        For a low-mounted camera looking along the floor:
          - The bottom of the ROI is close to the robot (wide, reliable).
          - The top of the ROI is far away (narrow, converging perspective).

        The warp spreads the top out to match the bottom width, making
        far-away tape appear at the correct horizontal position.

        warp_strength controls how aggressively the top corners are pulled out.
        0.0 = identity (no warp), 1.0 = full correction.
        """
        h, w = roi.shape[:2]
        s = self.warp_strength

        # Source: trapezoid in original perspective image
        #   top edge pulled inward (tape converges in distance)
        #   bottom edge is full width
        inset = int(w * 0.30 * s)   # how much the top is narrower than bottom
        src = np.float32([
            [inset,     0],          # top-left
            [w - inset, 0],          # top-right
            [w,         h],          # bottom-right
            [0,         h],          # bottom-left
        ])

        # Destination: full rectangle (bird's-eye)
        dst = np.float32([
            [0,  0],
            [w,  0],
            [w,  h],
            [0,  h],
        ])

        M = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(roi, M, (w, h),
                                   flags=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_REPLICATE)

    # ── Mask cleanup ──────────────────────────────────────────────────────────

    def _clean_mask(self, mask, close_iters=3):
        k3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        k7 = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k3, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k7, iterations=close_iters)
        return mask

    # ── White line detection (with shape + solidity filter) ───────────────────

    def get_cx_white(self, mask):
        """
        Find the white lane line centroid.

        Shape filter:
          aspect_ratio = bounding_w / bounding_h  must be > white_aspect_min
            → Window reflections are tall diagonal streaks  (aspect < 1)
            → Real tape seen from low camera is wide/flat   (aspect > 1.2)

          solidity = contour_area / convex_hull_area  must be > white_solidity_min
            → Glare is wispy/irregular  (solidity ~0.2-0.3)
            → Solid tape is compact     (solidity ~0.6-0.9)
        """
        mask = self._clean_mask(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x, y, cw, ch = cv2.boundingRect(c)
            if cw < self.min_width:
                continue
            # Aspect ratio check — reject tall reflection streaks
            aspect = cw / float(ch) if ch > 0 else 0
            if aspect < self.white_aspect_min:
                continue
            # Solidity check — reject wispy glare blobs
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            if solidity < self.white_solidity_min:
                continue
            valid.append(c)

        if not valid:
            return None, []
        biggest = max(valid, key=cv2.contourArea)
        M = cv2.moments(biggest)
        if M['m00'] == 0:
            return None, valid
        return int(M['m10'] / M['m00']), valid

    # ── Yellow line detection (weighted centroid across all dashes) ────────────

    def get_cx_yellow(self, mask):
        """
        Find the yellow dashed line centroid using area-weighted average
        across all qualifying blobs so gaps between dashes don't cause jumps.
        """
        mask = self._clean_mask(mask, close_iters=4)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid = []
        for c in contours:
            if cv2.contourArea(c) < self.min_area:
                continue
            x, y, cw, ch = cv2.boundingRect(c)
            if cw < self.min_width:
                continue
            valid.append(c)

        if not valid:
            return None, []

        total_area  = 0.0
        weighted_cx = 0.0
        for c in valid:
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            weighted_cx += (M['m10'] / M['m00']) * M['m00']
            total_area  += M['m00']

        if total_area == 0:
            return None, valid
        return int(weighted_cx / total_area), valid

    # ── Main image callback ────────────────────────────────────────────────────

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w  = frame.shape[:2]

        # ── 1. Slice ROI — floor only, no walls/windows ───────────────────────
        roi_y = int(h * self.roi_top_frac)
        roi   = frame[roi_y:, :]
        rh, rw = roi.shape[:2]

        # ── 2. Perspective warp for look-ahead ────────────────────────────────
        if self.warp_enabled:
            warped = self._warp_birdseye(roi)
        else:
            warped = roi

        # ── 3. HSV masking ────────────────────────────────────────────────────
        blurred     = cv2.GaussianBlur(warped, (5, 5), 0)
        hsv         = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        white_mask  = cv2.inRange(hsv, self.white_lower,  self.white_upper)
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)

        raw_white_cx,  white_contours  = self.get_cx_white(white_mask)
        raw_yellow_cx, yellow_contours = self.get_cx_yellow(yellow_mask)

        # ── 4. Jump rejection (reflection flash guard) ─────────────────────────
        white_cx = raw_white_cx
        if (raw_white_cx is not None
                and self._prev_white_cx is not None
                and abs(raw_white_cx - self._prev_white_cx) > self.max_cx_jump_px):
            white_cx = self._prev_white_cx
            self.get_logger().warn(
                f'White cx jump {self._prev_white_cx}->{raw_white_cx} suppressed',
                throttle_duration_sec=0.5)

        # ── 5. Temporal confidence filter ─────────────────────────────────────
        # White must be detected for N consecutive frames before we trust it.
        # Single-frame glare flashes never reach the required count.
        if white_cx is not None:
            self._white_confirm_count = min(
                self._white_confirm_count + 1, self.white_confirm_frames + 1)
        else:
            self._white_confirm_count = 0

        self._white_confirmed = (self._white_confirm_count >= self.white_confirm_frames)

        # Use confirmed white only; unconfirmed acts as None for steering
        trusted_white_cx = white_cx if self._white_confirmed else None

        yellow_cx = raw_yellow_cx

        if raw_white_cx  is not None: self._prev_white_cx  = raw_white_cx
        if raw_yellow_cx is not None: self._prev_yellow_cx = raw_yellow_cx

        # ── 6. Turn detection ──────────────────────────────────────────────────
        in_right_turn = (trusted_white_cx is not None and
                         trusted_white_cx < int(rw * self.turn_white_frac))
        in_left_turn  = (yellow_cx is not None and
                         yellow_cx > int(rw * self.turn_yellow_frac))
        in_turn = in_right_turn or in_left_turn

        # ── 7. Build debug image ───────────────────────────────────────────────
        debug = frame.copy()

        # Shade the excluded zone (above ROI) so it's obvious what's ignored
        overlay = debug.copy()
        cv2.rectangle(overlay, (0, 0), (w, roi_y), (30, 30, 30), -1)
        cv2.addWeighted(overlay, 0.45, debug, 0.55, 0, debug)
        cv2.line(debug, (0, roi_y), (w, roi_y), (0, 255, 255), 2)
        cv2.putText(debug, 'IGNORED (above floor)', (10, roi_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)

        # Draw contours on the debug frame at ROI offset
        if white_contours:
            col = (0, 200, 255) if self._white_confirmed else (80, 80, 255)
            shifted = [c + np.array([[[0, roi_y]]]) for c in white_contours]
            cv2.drawContours(debug, shifted, -1, col, 3)

        if yellow_contours:
            shifted = [c + np.array([[[0, roi_y]]]) for c in yellow_contours]
            cv2.drawContours(debug, shifted, -1, (0, 220, 255), 3)

        mid_y = roi_y + rh // 2

        # White centroid — orange if unconfirmed, bright cyan if confirmed
        if white_cx is not None:
            col = (0, 200, 255) if self._white_confirmed else (60, 60, 255)
            label = f'W={white_cx}' + ('' if self._white_confirmed else ' (unconfirmed)')
            cv2.circle(debug, (white_cx, mid_y), 14, col, -1)
            cv2.putText(debug, label, (max(0, white_cx - 60), mid_y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)

        if yellow_cx is not None:
            cv2.circle(debug, (yellow_cx, mid_y), 14, (0, 220, 255), -1)
            cv2.putText(debug, f'Y={yellow_cx}', (max(0, yellow_cx - 60), mid_y + 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)

        cv2.line(debug, (w // 2, roi_y), (w // 2, h), (160, 160, 160), 1)

        # ── 8. Steering logic ──────────────────────────────────────────────────
        twist    = Twist()
        error    = None
        target_x = None
        status   = 'NO LINES'

        if trusted_white_cx is not None and yellow_cx is not None:
            if trusted_white_cx > yellow_cx:
                lane_center = (trusted_white_cx + yellow_cx) / 2.0
                target_x    = lane_center - self.camera_offset_px
                turn_tag    = ' R-TURN' if in_right_turn else (' L-TURN' if in_left_turn else '')
                status      = f'BOTH{turn_tag}'
            else:
                # Geometry crossed (tight corner) — trust yellow
                target_x = yellow_cx + int(rw * self.yellow_only_gap_frac) - self.camera_offset_px
                status   = 'CROSSED->Y'

        elif trusted_white_cx is not None:
            gap      = int(rw * self.white_only_gap_frac)
            target_x = trusted_white_cx - gap - self.camera_offset_px
            status   = 'WHITE-ONLY' + (' R-TURN' if in_right_turn else '')

        elif yellow_cx is not None:
            gap      = int(rw * self.yellow_only_gap_frac)
            target_x = yellow_cx + gap - self.camera_offset_px
            status   = 'YELLOW-ONLY' + (' L-TURN' if in_left_turn else '')

        if target_x is not None:
            error = target_x - (rw / 2.0)

        # Draw target line
        if target_x is not None:
            tx = int(np.clip(target_x, 0, w - 1))
            cv2.circle(debug, (tx, mid_y), 10, (0, 255, 0), -1)
            cv2.line(debug, (tx, roi_y), (tx, h), (0, 255, 0), 2)

        # Status bar
        err_str     = f'  err={error:+.0f}' if error is not None else ''
        full_status = status + err_str
        s_color = (0, 255, 0) if (error is not None and abs(error) < 60) else (0, 80, 255)
        cv2.putText(debug, full_status, (15, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, s_color, 2)

        # Confirmation counter HUD
        cv2.putText(debug,
            f'W-confirm:{self._white_confirm_count}/{self.white_confirm_frames}  '
            f'warp:{"ON" if self.warp_enabled else "OFF"}({self.warp_strength:.2f})',
            (15, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        # HSV readout
        cv2.putText(debug,
            f'W s:{self.white_lower[1]}-{self.white_upper[1]} v:{self.white_lower[2]}+'
            f'  Y h:{self.yellow_lower[0]}-{self.yellow_upper[0]} v:{self.yellow_lower[2]}+',
            (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

        # ── 9. Publish cmd_vel ─────────────────────────────────────────────────
        if error is None:
            self.get_logger().warn('No lines — sweeping', throttle_duration_sec=2.0)
            twist.angular.z = 0.18
            self.pub.publish(twist)
            return

        self.get_logger().info(full_status, throttle_duration_sec=0.5)

        effective_kp = self.kp * (self.turn_kp_scale if in_turn else 1.0)
        fwd_speed    = self.turn_speed if in_turn else self.linear_speed

        # Positive error = target right of centre = steer LEFT = negative z
        angular = float(np.clip(-effective_kp * error, -self.max_angular, self.max_angular))

        twist.linear.x  = fwd_speed
        twist.angular.z = angular
        self.pub.publish(twist)


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
